//! Port of `datastructures/ShapeTree.java` + `MinAreaTree.java`.
//!
//! A binary search tree for shapes in the plane; the shapes are stored in
//! the leaves. A new shape is inserted by walking from the root towards the
//! child whose bounding shape grows least (by area) when united with the
//! new shape, until a leaf is reached, which is then forked.
//!
//! The Java pointer structure (TreeNode/InnerNode/Leaf with parent links)
//! becomes an arena of nodes addressed by [`LeafId`]/indices. Java's
//! `ShapeBoundingDirections` type parameter collapses to [`IntOctagon`]
//! bounds, which represent both the orthogonal (box) and the 45-degree
//! bounding variants exactly.

use crate::geometry::planar::IntOctagon;

/// Handle to a leaf in the tree, stable until the leaf is removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LeafId(usize);

#[derive(Debug, Clone)]
enum NodeKind<T> {
    Inner { first: usize, second: usize },
    Leaf { object: T, shape_index: usize },
    /// Slot on the free list.
    Free { next_free: Option<usize> },
}

#[derive(Debug, Clone)]
struct Node<T> {
    bound: IntOctagon,
    parent: Option<usize>,
    kind: NodeKind<T>,
}

/// The entry stored in a leaf: the object handle plus the index of the
/// shape within the object (Java: `ShapeTree.TreeEntry`).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct TreeEntry<T> {
    pub object: T,
    pub shape_index_in_object: usize,
}

#[derive(Debug, Clone)]
pub struct MinAreaTree<T> {
    nodes: Vec<Node<T>>,
    first_free: Option<usize>,
    root: Option<usize>,
    leaf_count: usize,
}

impl<T> Default for MinAreaTree<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> MinAreaTree<T> {
    pub fn new() -> Self {
        MinAreaTree {
            nodes: Vec::new(),
            first_free: None,
            root: None,
            leaf_count: 0,
        }
    }

    /// The number of entries stored in the tree.
    pub fn size(&self) -> usize {
        self.leaf_count
    }

    pub fn is_empty(&self) -> bool {
        self.leaf_count == 0
    }

    fn alloc(&mut self, node: Node<T>) -> usize {
        if let Some(free) = self.first_free {
            let NodeKind::Free { next_free } = self.nodes[free].kind else {
                unreachable!("free list corrupt");
            };
            self.first_free = next_free;
            self.nodes[free] = node;
            free
        } else {
            self.nodes.push(node);
            self.nodes.len() - 1
        }
    }

    fn release(&mut self, index: usize) {
        self.nodes[index].kind = NodeKind::Free {
            next_free: self.first_free,
        };
        self.nodes[index].parent = None;
        self.first_free = Some(index);
    }

    /// Inserts one shape with its precomputed bounding shape; returns the
    /// leaf handle for later removal.
    pub fn insert(
        &mut self,
        object: T,
        shape_index: usize,
        bounding_shape: IntOctagon,
    ) -> LeafId {
        let leaf = self.alloc(Node {
            bound: bounding_shape,
            parent: None,
            kind: NodeKind::Leaf {
                object,
                shape_index,
            },
        });
        self.leaf_count += 1;

        let Some(root) = self.root else {
            // tree is empty: the new leaf becomes the root
            self.root = Some(leaf);
            return LeafId(leaf);
        };

        let leaf_to_replace = self.position_locate(root, bounding_shape);

        // Construct a new inner node forking the found leaf and the new
        // leaf.
        let new_bounds = bounding_shape.union(self.nodes[leaf_to_replace].bound);
        let curr_parent = self.nodes[leaf_to_replace].parent;
        let new_node = self.alloc(Node {
            bound: new_bounds,
            parent: curr_parent,
            kind: NodeKind::Inner {
                first: leaf_to_replace,
                second: leaf,
            },
        });
        if let Some(parent) = curr_parent {
            let NodeKind::Inner { first, second } = &mut self.nodes[parent].kind else {
                unreachable!("parent must be inner");
            };
            if *first == leaf_to_replace {
                *first = new_node;
            } else {
                *second = new_node;
            }
        }
        self.nodes[leaf_to_replace].parent = Some(new_node);
        self.nodes[leaf].parent = Some(new_node);
        if self.root == Some(leaf_to_replace) {
            self.root = Some(new_node);
        }
        LeafId(leaf)
    }

    /// Walks down from `start` to the leaf whose bounding shape grows least
    /// when united with `insert_bound`, enlarging the bounds along the path.
    fn position_locate(&mut self, start: usize, insert_bound: IntOctagon) -> usize {
        let mut curr = start;
        loop {
            let (first, second) = match self.nodes[curr].kind {
                NodeKind::Leaf { .. } => return curr,
                NodeKind::Inner { first, second } => (first, second),
                NodeKind::Free { .. } => unreachable!("free node reached"),
            };
            self.nodes[curr].bound = insert_bound.union(self.nodes[curr].bound);

            // Choose the child with minimal area increase after taking the
            // union with the shape to insert.
            let first_shape = self.nodes[first].bound;
            let first_area_increase =
                insert_bound.union(first_shape).area() - first_shape.area();
            let second_shape = self.nodes[second].bound;
            let second_area_increase =
                insert_bound.union(second_shape).area() - second_shape.area();
            curr = if first_area_increase <= second_area_increase {
                first
            } else {
                second
            };
        }
    }

    /// Removes a leaf from the tree.
    pub fn remove_leaf(&mut self, leaf: LeafId) {
        let leaf = leaf.0;
        debug_assert!(matches!(self.nodes[leaf].kind, NodeKind::Leaf { .. }));
        let parent = self.nodes[leaf].parent;
        self.release(leaf);
        self.leaf_count -= 1;
        let Some(parent) = parent else {
            // the tree becomes empty
            self.root = None;
            return;
        };
        // find the other child of the parent
        let NodeKind::Inner { first, second } = self.nodes[parent].kind else {
            unreachable!("parent must be inner");
        };
        let other_child = if second == leaf { first } else { second };
        // link the other child to the grandparent, dropping the parent
        let grand_parent = self.nodes[parent].parent;
        self.nodes[other_child].parent = grand_parent;
        self.release(parent);
        let Some(grand_parent) = grand_parent else {
            // only one child left in the tree
            self.root = Some(other_child);
            return;
        };
        {
            let NodeKind::Inner { first, second } = &mut self.nodes[grand_parent].kind else {
                unreachable!("grandparent must be inner");
            };
            if *second == parent {
                *second = other_child;
            } else {
                *first = other_child;
            }
        }
        // Recalculate the bounding shapes of the ancestors as long as they
        // shrink.
        let mut node_to_recalculate = Some(grand_parent);
        while let Some(curr) = node_to_recalculate {
            let NodeKind::Inner { first, second } = self.nodes[curr].kind else {
                unreachable!("ancestor must be inner");
            };
            let new_bounds = self.nodes[second].bound.union(self.nodes[first].bound);
            if self.nodes[curr].bound.is_contained_in(new_bounds) {
                // the new bounds are not smaller: stop recalculating
                break;
            }
            self.nodes[curr].bound = new_bounds;
            node_to_recalculate = self.nodes[curr].parent;
        }
    }

    /// Removes several leaves (Java: `remove(Leaf[])`).
    pub fn remove(&mut self, entries: &[LeafId]) {
        for leaf in entries {
            self.remove_leaf(*leaf);
        }
    }

    /// The leaves of this tree in depth-first order.
    pub fn to_array(&self) -> Vec<LeafId> {
        let mut result = Vec::with_capacity(self.leaf_count);
        let Some(root) = self.root else {
            return result;
        };
        let mut stack = vec![root];
        while let Some(curr) = stack.pop() {
            match self.nodes[curr].kind {
                NodeKind::Leaf { .. } => result.push(LeafId(curr)),
                NodeKind::Inner { first, second } => {
                    // push second first so first is visited first
                    stack.push(second);
                    stack.push(first);
                }
                NodeKind::Free { .. } => unreachable!("free node reached"),
            }
        }
        result
    }

    /// The object and shape index stored in a leaf.
    pub fn entry(&self, leaf: LeafId) -> TreeEntry<&T> {
        let NodeKind::Leaf {
            ref object,
            shape_index,
        } = self.nodes[leaf.0].kind
        else {
            panic!("MinAreaTree::entry: not a leaf");
        };
        TreeEntry {
            object,
            shape_index_in_object: shape_index,
        }
    }

    /// The bounding shape of a leaf.
    pub fn bounding_shape(&self, leaf: LeafId) -> IntOctagon {
        self.nodes[leaf.0].bound
    }

    /// The number of nodes between the leaf and the root.
    pub fn distance_to_root(&self, leaf: LeafId) -> usize {
        let mut result = 0;
        let mut curr = self.nodes[leaf.0].parent;
        while let Some(node) = curr {
            result += 1;
            curr = self.nodes[node].parent;
        }
        result
    }

    /// The leaves of this tree whose bounding shapes overlap `shape`.
    pub fn overlaps(&self, shape: IntOctagon) -> Vec<LeafId> {
        let mut found = Vec::new();
        self.overlaps_with(shape, |leaf| found.push(leaf));
        found
    }

    /// Allocation-free query: calls `f` for every overlapping leaf using
    /// a reusable traversal stack (tree queries dominated the big-board
    /// profile largely through per-query allocations).
    pub fn overlaps_with(&self, shape: IntOctagon, mut f: impl FnMut(LeafId)) {
        thread_local! {
            static STACK: std::cell::RefCell<Vec<usize>> =
                const { std::cell::RefCell::new(Vec::new()) };
        }
        let Some(root) = self.root else {
            return;
        };
        STACK.with(|cell| {
            let mut stack = cell.borrow_mut();
            stack.clear();
            stack.push(root);
            while let Some(curr) = stack.pop() {
                if !self.nodes[curr].bound.intersects(shape) {
                    continue;
                }
                match self.nodes[curr].kind {
                    NodeKind::Leaf { .. } => f(LeafId(curr)),
                    NodeKind::Inner { first, second } => {
                        stack.push(first);
                        stack.push(second);
                    }
                    NodeKind::Free { .. } => unreachable!("free node reached"),
                }
            }
        });
    }
}

impl<T: Clone + Ord> MinAreaTree<T> {
    /// Like [`MinAreaTree::overlaps`], but returning the stored entries in
    /// sorted order (Java returns a `TreeSet<Leaf>`).
    pub fn overlapping_entries(&self, shape: IntOctagon) -> Vec<TreeEntry<T>> {
        let mut result: Vec<TreeEntry<T>> = self
            .overlaps(shape)
            .into_iter()
            .map(|leaf| {
                let e = self.entry(leaf);
                TreeEntry {
                    object: e.object.clone(),
                    shape_index_in_object: e.shape_index_in_object,
                }
            })
            .collect();
        result.sort();
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntBox;

    fn oct(llx: i32, lly: i32, urx: i32, ury: i32) -> IntOctagon {
        IntBox::from_coords(llx, lly, urx, ury).to_int_octagon()
    }

    #[test]
    fn insert_query_remove() {
        let mut tree: MinAreaTree<u32> = MinAreaTree::new();
        assert!(tree.is_empty());
        assert!(tree.overlaps(oct(0, 0, 100, 100)).is_empty());

        let mut leaves = Vec::new();
        // a 10x10 grid of 4x4 boxes spaced 10 apart
        for i in 0..10 {
            for j in 0..10 {
                let id = (i * 10 + j) as u32;
                let bound = oct(i * 10, j * 10, i * 10 + 4, j * 10 + 4);
                leaves.push(tree.insert(id, 0, bound));
            }
        }
        assert_eq!(tree.size(), 100);
        assert_eq!(tree.to_array().len(), 100);

        // query a window covering exactly the 4 boxes at (0,0),(0,10),(10,0),(10,10)
        let found = tree.overlapping_entries(oct(0, 0, 12, 12));
        let ids: Vec<u32> = found.iter().map(|e| e.object).collect();
        assert_eq!(ids, vec![0, 1, 10, 11]);

        // a query between the boxes finds nothing
        assert!(tree.overlaps(oct(5, 5, 9, 9)).is_empty());

        // remove one of them and re-query
        tree.remove_leaf(leaves[0]);
        assert_eq!(tree.size(), 99);
        let found = tree.overlapping_entries(oct(0, 0, 12, 12));
        let ids: Vec<u32> = found.iter().map(|e| e.object).collect();
        assert_eq!(ids, vec![1, 10, 11]);

        // remove everything
        tree.remove(&leaves[1..]);
        assert!(tree.is_empty());
        assert!(tree.overlaps(oct(0, 0, 100, 100)).is_empty());

        // arena slots are reused after removals
        let node_count_before = tree.nodes.len();
        tree.insert(7, 0, oct(0, 0, 1, 1));
        assert_eq!(tree.nodes.len(), node_count_before);
    }

    #[test]
    fn bounds_shrink_after_removal() {
        let mut tree: MinAreaTree<u32> = MinAreaTree::new();
        let far_leaf = tree.insert(1, 0, oct(100, 100, 104, 104));
        tree.insert(2, 0, oct(0, 0, 4, 4));
        tree.insert(3, 0, oct(2, 2, 6, 6));
        // Query near the far leaf: only it matches.
        assert_eq!(tree.overlaps(oct(99, 99, 105, 105)).len(), 1);
        tree.remove_leaf(far_leaf);
        // After removal the root bound must have shrunk so the far query
        // misses everything.
        assert!(tree.overlaps(oct(99, 99, 105, 105)).is_empty());
        assert_eq!(tree.overlaps(oct(0, 0, 10, 10)).len(), 2);
    }

    #[test]
    fn multi_shape_objects() {
        let mut tree: MinAreaTree<&'static str> = MinAreaTree::new();
        // an object with 3 shapes, like a trace with 3 segments
        let bounds = [oct(0, 0, 10, 2), oct(10, 0, 12, 10), oct(10, 10, 20, 12)];
        let leaves: Vec<LeafId> = bounds
            .iter()
            .enumerate()
            .map(|(i, b)| tree.insert("trace", i, *b))
            .collect();
        let found = tree.overlapping_entries(oct(11, 5, 15, 15));
        assert_eq!(found.len(), 2);
        assert_eq!(found[0].shape_index_in_object, 1);
        assert_eq!(found[1].shape_index_in_object, 2);
        assert_eq!(tree.entry(leaves[0]).shape_index_in_object, 0);
        assert!(tree.distance_to_root(leaves[0]) >= 1);
    }

    #[test]
    fn many_random_like_insertions_stay_consistent() {
        let mut tree: MinAreaTree<usize> = MinAreaTree::new();
        let mut leaves = Vec::new();
        // deterministic pseudo-random placement
        let mut x = 17_i32;
        for i in 0..500 {
            x = (x.wrapping_mul(1103515245).wrapping_add(12345)) % 1000;
            let cx = x.abs() % 900;
            let cy = (x.abs() / 7) % 900;
            leaves.push(tree.insert(i, 0, oct(cx, cy, cx + 20, cy + 20)));
        }
        assert_eq!(tree.size(), 500);
        // brute-force check one window query
        let window = oct(200, 200, 400, 400);
        let expected: Vec<usize> = leaves
            .iter()
            .enumerate()
            .filter(|(_, leaf)| tree.bounding_shape(**leaf).intersects(window))
            .map(|(i, _)| i)
            .collect();
        let mut found: Vec<usize> = tree
            .overlaps(window)
            .into_iter()
            .map(|leaf| *tree.entry(leaf).object)
            .collect();
        found.sort();
        assert_eq!(found, expected);
        // remove every second leaf and re-verify
        for (i, leaf) in leaves.iter().enumerate() {
            if i % 2 == 0 {
                tree.remove_leaf(*leaf);
            }
        }
        assert_eq!(tree.size(), 250);
        let mut found: Vec<usize> = tree
            .overlaps(window)
            .into_iter()
            .map(|leaf| *tree.entry(leaf).object)
            .collect();
        found.sort();
        let expected: Vec<usize> = expected.into_iter().filter(|i| i % 2 == 1).collect();
        assert_eq!(found, expected);
    }
}
