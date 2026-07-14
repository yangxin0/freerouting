//! Port of `datastructures/UndoableObjects.java`.
//!
//! A database of objects supporting undo and redo via per-object version
//! chains and per-snapshot-level delete lists. Works only for values
//! containing no references (they are cloned on `save_for_undo`).
//!
//! The Java version keys the map by the object itself (via `compareTo`) and
//! links nodes with pointers; this port separates an explicit key `K` from
//! the stored value `V` and keeps the version chains in an arena.

use std::collections::BTreeMap;

type NodeId = usize;

#[derive(Debug)]
struct Node<K, V> {
    key: K,
    object: V,
    /// the level in the undo stack where this node was inserted
    level: usize,
    /// the node to restore in an undo, if any
    undo_object: Option<NodeId>,
    /// the node to restore in a redo, if any
    redo_object: Option<NodeId>,
}

#[derive(Debug)]
pub struct UndoableObjects<K: Ord + Clone, V: Clone + PartialEq> {
    objects: BTreeMap<K, NodeId>,
    nodes: Vec<Node<K, V>>,
    /// deleted objects per undo level which existed before the previous
    /// snapshot
    deleted_objects_stack: Vec<Vec<NodeId>>,
    stack_level: usize,
    redo_possible: bool,
}

impl<K: Ord + Clone, V: Clone + PartialEq> Default for UndoableObjects<K, V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Ord + Clone, V: Clone + PartialEq> UndoableObjects<K, V> {
    pub fn new() -> Self {
        UndoableObjects {
            objects: BTreeMap::new(),
            nodes: Vec::new(),
            deleted_objects_stack: Vec::new(),
            stack_level: 0,
            redo_possible: false,
        }
    }

    fn alloc(&mut self, node: Node<K, V>) -> NodeId {
        self.nodes.push(node);
        self.nodes.len() - 1
    }

    /// Iterates over the currently alive objects (objects alive only by
    /// redo are skipped, like Java's `read_object`).
    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.objects.iter().filter_map(|(key, &node)| {
            let node = &self.nodes[node];
            if node.level <= self.stack_level {
                Some((key, &node.object))
            } else {
                None
            }
        })
    }

    /// The current value stored for `key`, if alive.
    pub fn get(&self, key: &K) -> Option<&V> {
        let &node = self.objects.get(key)?;
        let node = &self.nodes[node];
        if node.level <= self.stack_level {
            Some(&node.object)
        } else {
            None
        }
    }

    /// Mutable access to the current value for `key`. Call
    /// [`Self::save_for_undo`] first if the object may predate the last
    /// snapshot.
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        let &node = self.objects.get(key)?;
        if self.nodes[node].level <= self.stack_level {
            Some(&mut self.nodes[node].object)
        } else {
            None
        }
    }

    /// Adds an object to the database.
    pub fn insert(&mut self, key: K, value: V) {
        self.disable_redo();
        let level = self.stack_level;
        let node = self.alloc(Node {
            key: key.clone(),
            object: value,
            level,
            undo_object: None,
            redo_object: None,
        });
        self.objects.insert(key, node);
    }

    /// Removes an object from the top level. Returns false if `key` was
    /// not found.
    pub fn delete(&mut self, key: &K) -> bool {
        self.disable_redo();
        let Some(&object_node) = self.objects.get(key) else {
            return false;
        };
        if !self.deleted_objects_stack.is_empty() {
            let node = &self.nodes[object_node];
            let to_remember = if node.level < self.stack_level {
                Some(object_node)
            } else {
                // remember the previous version to make undo possible
                node.undo_object
            };
            if let Some(remembered) = to_remember {
                self.deleted_objects_stack
                    .last_mut()
                    .unwrap()
                    .push(remembered);
            }
        }
        self.objects.remove(key);
        true
    }

    /// Makes the current state restorable by undo.
    pub fn generate_snapshot(&mut self) {
        self.disable_redo();
        self.deleted_objects_stack.push(Vec::new());
        self.stack_level += 1;
    }

    /// Must be called before an object is modified for the first time
    /// after a snapshot, if it may have existed before that snapshot.
    pub fn save_for_undo(&mut self, key: &K) {
        self.disable_redo();
        let Some(&curr_node) = self.objects.get(key) else {
            return;
        };
        if self.nodes[curr_node].level < self.stack_level {
            let old_node = self.alloc(Node {
                key: self.nodes[curr_node].key.clone(),
                object: self.nodes[curr_node].object.clone(),
                level: self.nodes[curr_node].level,
                undo_object: self.nodes[curr_node].undo_object,
                redo_object: Some(curr_node),
            });
            self.nodes[curr_node].undo_object = Some(old_node);
            self.nodes[curr_node].level = self.stack_level;
        }
    }

    /// Restores the situation before the last snapshot, reporting the
    /// cancelled and restored values. Returns false if no undo is possible.
    pub fn undo(&mut self, cancelled_objects: &mut Vec<V>, restored_objects: &mut Vec<V>) -> bool {
        if self.stack_level == 0 {
            return false;
        }
        let keys: Vec<K> = self.objects.keys().cloned().collect();
        for key in keys {
            let curr_node = self.objects[&key];
            if self.nodes[curr_node].level == self.stack_level {
                if let Some(undo_node) = self.nodes[curr_node].undo_object {
                    // replace the current object by its previous state
                    self.nodes[undo_node].redo_object = Some(curr_node);
                    self.objects.insert(key, undo_node);
                    restored_objects.push(self.nodes[undo_node].object.clone());
                }
                cancelled_objects.push(self.nodes[curr_node].object.clone());
            }
        }
        // restore the deleted objects
        let delete_list = self.deleted_objects_stack[self.stack_level - 1].clone();
        for deleted_node in delete_list {
            let key = self.nodes[deleted_node].key.clone();
            self.objects.insert(key, deleted_node);
            restored_objects.push(self.nodes[deleted_node].object.clone());
        }
        self.stack_level -= 1;
        self.redo_possible = true;
        true
    }

    /// Restores the situation before the last undo. Returns false if no
    /// redo is possible.
    pub fn redo(&mut self, cancelled_objects: &mut Vec<V>, restored_objects: &mut Vec<V>) -> bool {
        if self.stack_level >= self.deleted_objects_stack.len() {
            return false; // already at the top level
        }
        self.stack_level += 1;
        let keys: Vec<K> = self.objects.keys().cloned().collect();
        for key in keys {
            let curr_node = self.objects[&key];
            let redo = self.nodes[curr_node].redo_object;
            if let Some(redo_node) =
                redo.filter(|&r| self.nodes[r].level == self.stack_level)
            {
                // object was changed on the current level: replace it by
                // the newer version
                self.objects.insert(key, redo_node);
                cancelled_objects.push(self.nodes[curr_node].object.clone());
                restored_objects.push(self.nodes[redo_node].object.clone());
            } else if self.nodes[curr_node].level == self.stack_level {
                // object was created on the current level
                restored_objects.push(self.nodes[curr_node].object.clone());
            }
        }
        // delete the objects which were deleted on the current level again
        let delete_list = self.deleted_objects_stack[self.stack_level - 1].clone();
        for mut deleted_node in delete_list {
            while let Some(redo_node) = self.nodes[deleted_node]
                .redo_object
                .filter(|&r| self.nodes[r].level <= self.stack_level)
            {
                deleted_node = redo_node;
            }
            let key = self.nodes[deleted_node].key.clone();
            self.objects.remove(&key);
            let object = self.nodes[deleted_node].object.clone();
            if let Some(pos) = restored_objects.iter().position(|v| *v == object) {
                restored_objects.remove(pos);
            } else {
                // the object needs only be cancelled if it is already in
                // the board
                cancelled_objects.push(object);
            }
        }
        true
    }

    /// Removes the top snapshot from the undo stack, so that its situation
    /// cannot be restored anymore. Returns false if there is no snapshot.
    pub fn pop_snapshot(&mut self) -> bool {
        self.disable_redo();
        if self.stack_level == 0 {
            return false;
        }
        let node_ids: Vec<NodeId> = self.objects.values().copied().collect();
        for curr_node in node_ids {
            let level = self.nodes[curr_node].level;
            if level == self.stack_level - 1 {
                if let Some(redo_node) = self.nodes[curr_node]
                    .redo_object
                    .filter(|&r| self.nodes[r].level == self.stack_level)
                {
                    let undo_node = self.nodes[curr_node].undo_object;
                    self.nodes[redo_node].undo_object = undo_node;
                    if let Some(undo_node) = undo_node {
                        self.nodes[undo_node].redo_object = Some(redo_node);
                    }
                }
            } else if level >= self.stack_level {
                self.nodes[curr_node].level -= 1;
                // Deviation from Java: if the object was also saved on the
                // popped level, splice that version out of the undo chain.
                // Java leaves it in (its pop_snapshot can only reach nodes
                // via the live map), which makes a later undo restore a
                // node whose level is above the new stack level — the
                // object then disappears from iteration.
                if let Some(undo_node) = self.nodes[curr_node].undo_object {
                    if self.nodes[undo_node].level == self.nodes[curr_node].level {
                        let older = self.nodes[undo_node].undo_object;
                        self.nodes[curr_node].undo_object = older;
                        if let Some(older) = older {
                            self.nodes[older].redo_object = Some(curr_node);
                        }
                    }
                }
            }
        }
        let stack_size = self.deleted_objects_stack.len();
        if stack_size >= 2 {
            // join the top delete list with the delete list below it
            let from_delete_list = self.deleted_objects_stack[stack_size - 1].clone();
            for deleted_node in from_delete_list {
                let to_add = if self.nodes[deleted_node].level < self.stack_level - 1 {
                    Some(deleted_node)
                } else {
                    self.nodes[deleted_node].undo_object
                };
                if let Some(node) = to_add {
                    self.deleted_objects_stack[stack_size - 2].push(node);
                }
            }
        }
        self.deleted_objects_stack.pop();
        self.stack_level -= 1;
        true
    }

    /// Must be called when objects are changed for the first time after an
    /// undo: drops all redo state.
    fn disable_redo(&mut self) {
        if !self.redo_possible {
            return;
        }
        self.redo_possible = false;
        self.deleted_objects_stack.truncate(self.stack_level);
        let keys: Vec<K> = self.objects.keys().cloned().collect();
        for key in keys {
            let curr_node = self.objects[&key];
            let level = self.nodes[curr_node].level;
            if level > self.stack_level {
                self.objects.remove(&key);
            } else if level == self.stack_level {
                self.nodes[curr_node].redo_object = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alive(db: &UndoableObjects<u32, String>) -> Vec<(u32, String)> {
        db.iter().map(|(k, v)| (*k, v.clone())).collect()
    }

    #[test]
    fn insert_modify_undo_redo() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "a1".into());
        db.insert(2, "b1".into());
        db.generate_snapshot();

        // modify object 1 after the snapshot
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "a2".into();
        // insert a new object after the snapshot
        db.insert(3, "c1".into());
        assert_eq!(
            alive(&db),
            vec![(1, "a2".into()), (2, "b1".into()), (3, "c1".into())]
        );

        let (mut cancelled, mut restored) = (Vec::new(), Vec::new());
        assert!(db.undo(&mut cancelled, &mut restored));
        assert_eq!(alive(&db), vec![(1, "a1".into()), (2, "b1".into())]);
        assert!(cancelled.contains(&"a2".to_string()));
        assert!(cancelled.contains(&"c1".to_string()));
        assert_eq!(restored, vec!["a1".to_string()]);

        let (mut cancelled, mut restored) = (Vec::new(), Vec::new());
        assert!(db.redo(&mut cancelled, &mut restored));
        assert_eq!(
            alive(&db),
            vec![(1, "a2".into()), (2, "b1".into()), (3, "c1".into())]
        );
        assert!(restored.contains(&"a2".to_string()));
        assert!(restored.contains(&"c1".to_string()));

        // no further redo
        assert!(!db.redo(&mut Vec::new(), &mut Vec::new()));
    }

    #[test]
    fn delete_and_undo_restores() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "a1".into());
        db.generate_snapshot();
        assert!(db.delete(&1));
        assert!(!db.delete(&1));
        assert!(alive(&db).is_empty());

        let (mut cancelled, mut restored) = (Vec::new(), Vec::new());
        assert!(db.undo(&mut cancelled, &mut restored));
        assert_eq!(alive(&db), vec![(1, "a1".into())]);
        assert_eq!(restored, vec!["a1".to_string()]);

        // redo deletes it again
        let (mut cancelled, mut restored) = (Vec::new(), Vec::new());
        assert!(db.redo(&mut cancelled, &mut restored));
        assert!(alive(&db).is_empty());
        assert_eq!(cancelled, vec!["a1".to_string()]);
        assert!(restored.is_empty());
    }

    #[test]
    fn modify_then_delete_then_undo() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "a1".into());
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "a2".into();
        assert!(db.delete(&1));
        let (mut cancelled, mut restored) = (Vec::new(), Vec::new());
        assert!(db.undo(&mut cancelled, &mut restored));
        // the version before the snapshot comes back
        assert_eq!(alive(&db), vec![(1, "a1".into())]);
    }

    #[test]
    fn multiple_snapshot_levels() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "v1".into());
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "v2".into();
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "v3".into();

        assert!(db.undo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v2".into())]);
        assert!(db.undo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v1".into())]);
        assert!(!db.undo(&mut Vec::new(), &mut Vec::new()));
        // redo twice forward again
        assert!(db.redo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v2".into())]);
        assert!(db.redo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v3".into())]);
    }

    #[test]
    fn new_change_disables_redo() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "v1".into());
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "v2".into();
        assert!(db.undo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v1".into())]);
        // a new insert kills the redo branch
        db.insert(2, "w1".into());
        assert!(!db.redo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v1".into()), (2, "w1".into())]);
    }

    #[test]
    fn pop_snapshot_merges_levels() {
        let mut db: UndoableObjects<u32, String> = UndoableObjects::new();
        db.insert(1, "v1".into());
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "v2".into();
        db.generate_snapshot();
        db.save_for_undo(&1);
        *db.get_mut(&1).unwrap() = "v3".into();
        // Dropping the top snapshot removes the v2 version from the undo
        // chain (see the documented deviation from Java in pop_snapshot):
        // undo now goes straight back to v1.
        assert!(db.pop_snapshot());
        assert_eq!(alive(&db), vec![(1, "v3".into())]);
        assert!(db.undo(&mut Vec::new(), &mut Vec::new()));
        assert_eq!(alive(&db), vec![(1, "v1".into())]);
        assert!(!db.pop_snapshot());
    }
}
