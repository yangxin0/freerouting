//! S-expression reader for Specctra DSN/SES files (port of the scanner
//! layer under `io/specctra/parser`).
//!
//! DSN files are parenthesized token trees. The `string_quote` parser
//! setting names the quote character (KiCad emits `"`; Specctra also permits
//! `'` and `$`); a quote character
//! immediately following `(string_quote ` is itself a token. Quoted
//! strings may contain spaces when `space_in_quoted_tokens` is on; this
//! reader always allows them. At token start, `"` and `'` start a quoted
//! string like the Java scanner; `$` does so only after `(string_quote $)` has
//! been read, because EasyEDA also emits ordinary leading-dollar names such
//! as `$3N1206`. Inside an atom quote characters remain ordinary identifier
//! data unless they follow a composite-rule separator.

use std::fmt;
use std::ops::Range;

#[derive(Debug, Clone, PartialEq)]
pub enum SExpr {
    Atom(String),
    /// An atom that was delimited by a quote in the source.  Keeping this
    /// bit of lexical provenance matters for clearance composite names: a
    /// quoted `"A-B"` is one class name, whereas an unquoted `A-B` is the
    /// legacy pair spelling.
    Quoted(String, char),
    List(Vec<SExpr>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub message: String,
    pub position: usize,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "DSN parse error at byte {}: {}",
            self.position, self.message
        )
    }
}

impl std::error::Error for ParseError {}

impl SExpr {
    pub fn as_atom(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) => Some(s),
            SExpr::Quoted(s, _) => Some(s),
            SExpr::List(_) => None,
        }
    }

    /// Whether this atom was quoted in the source document.
    pub fn is_quoted(&self) -> bool {
        matches!(self, SExpr::Quoted(_, _))
    }

    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::Atom(_) | SExpr::Quoted(_, _) => None,
            SExpr::List(items) => Some(items),
        }
    }

    /// The keyword of a list node: its first atom (case-insensitive
    /// matching is the caller's business; DSN keywords are usually
    /// lowercase).
    pub fn name(&self) -> Option<&str> {
        self.as_list()?.first()?.as_atom()
    }

    /// The child lists with the given keyword.
    pub fn children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a SExpr> + 'a {
        self.as_list()
            .unwrap_or(&[])
            .iter()
            .filter(move |c| c.name().is_some_and(|n| n.eq_ignore_ascii_case(name)))
    }

    /// The first child list with the given keyword.
    pub fn child<'a>(&'a self, name: &'a str) -> Option<&'a SExpr> {
        self.children(name).next()
    }

    /// The atoms of this list after the keyword.
    pub fn args(&self) -> impl Iterator<Item = &str> {
        self.as_list()
            .unwrap_or(&[])
            .iter()
            .skip(1)
            .filter_map(|c| c.as_atom())
    }

    /// The first argument atom.
    pub fn arg(&self) -> Option<&str> {
        self.args().next()
    }

    pub fn arg_f64(&self) -> Option<f64> {
        self.arg()?.parse().ok()
    }
}

/// Parses a DSN file into its top-level S-expression.
pub fn parse_dsn(input: &str) -> Result<SExpr, ParseError> {
    let bytes = input.as_bytes();
    let mut pos = 0usize;
    let mut dollar_quote = false;
    let expr = parse_expr(bytes, &mut pos, &mut dollar_quote, None)?;
    skip_trivia(bytes, &mut pos, false)?;
    if pos != bytes.len() {
        return Err(ParseError {
            message: "unexpected trailing data".into(),
            position: pos,
        });
    }
    Ok(expr)
}

/// Returns the byte offset of the root list's closing parenthesis and the
/// spans of its direct child lists with `child_name`.  This follows the same
/// quote/comment/token rules as [`parse_dsn`], so callers doing source surgery
/// never mistake text inside an identifier or comment for a real scope.
pub(crate) fn document_structure(
    input: &str,
    child_name: &str,
) -> Result<(usize, Vec<Range<usize>>), ParseError> {
    fn scan_list(
        bytes: &[u8],
        pos: &mut usize,
        depth: usize,
        child_name: &str,
        matches: &mut Vec<Range<usize>>,
        dollar_quote: &mut bool,
        parent_head: Option<&str>,
    ) -> Result<Range<usize>, ParseError> {
        skip_trivia(bytes, pos, false)?;
        let start = *pos;
        if bytes.get(*pos) != Some(&b'(') {
            return Err(ParseError {
                message: "expected '('".into(),
                position: *pos,
            });
        }
        *pos += 1;
        let mut head: Option<String> = None;
        let mut head_quoted = false;
        let mut item_count = 0usize;
        // Number of direct atom tokens seen, including the head.  Nested
        // scopes do not advance this counter; `pin` re-enters NAME after its
        // optional rotate child for exactly its first two arguments.
        let mut direct_atom_count = 0usize;
        let mut saw_child = false;
        loop {
            let hash_is_identifier = hash_identifier_position(
                head.as_deref().map(|name| (name, head_quoted)),
                parent_head,
                item_count,
                saw_child,
                direct_atom_count,
            );
            skip_trivia(bytes, pos, hash_is_identifier)?;
            let Some(&byte) = bytes.get(*pos) else {
                return Err(ParseError {
                    message: "unterminated list".into(),
                    position: *pos,
                });
            };
            match byte {
                b')' => {
                    *pos += 1;
                    let span = start..*pos;
                    if depth == 1
                        && head
                            .as_deref()
                            .is_some_and(|name| name.eq_ignore_ascii_case(child_name))
                    {
                        matches.push(span.clone());
                    }
                    return Ok(span);
                }
                b'(' => {
                    scan_list(
                        bytes,
                        pos,
                        depth + 1,
                        child_name,
                        matches,
                        dollar_quote,
                        head.as_deref(),
                    )?;
                    item_count += 1;
                    saw_child = true;
                }
                quote @ (b'"' | b'\'') => {
                    let next = bytes.get(*pos + 1);
                    let is_string_quote_scope = head
                        .as_deref()
                        .is_some_and(|name| name.eq_ignore_ascii_case("string_quote"));
                    let is_quote_declaration = is_string_quote_scope
                        && next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')');
                    let value = if is_quote_declaration {
                        *dollar_quote = false;
                        *pos += 1;
                        (quote as char).to_string()
                    } else {
                        parse_quoted(bytes, pos, quote)?
                            .as_atom()
                            .unwrap_or_default()
                            .to_string()
                    };
                    if item_count == 0 {
                        head = Some(value);
                        head_quoted = !is_quote_declaration;
                    }
                    item_count += 1;
                    direct_atom_count += 1;
                }
                b'$' => {
                    let next = bytes.get(*pos + 1);
                    let is_string_quote_scope = head
                        .as_deref()
                        .is_some_and(|name| name.eq_ignore_ascii_case("string_quote"));
                    let is_quote_declaration = is_string_quote_scope
                        && next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')');
                    let atom = if is_quote_declaration {
                        *dollar_quote = true;
                        *pos += 1;
                        SExpr::Atom("$".to_string())
                    } else if *dollar_quote {
                        parse_quoted(bytes, pos, b'$')?
                    } else {
                        parse_atom(bytes, pos, false)
                    };
                    if item_count == 0 {
                        head = atom.as_atom().map(str::to_string);
                        head_quoted = atom.is_quoted();
                    }
                    item_count += 1;
                    direct_atom_count += 1;
                }
                separator @ (b'_' | b'-')
                    if matches!(bytes.get(*pos + 1), Some(&b'"') | Some(&b'\''))
                        || (*dollar_quote && bytes.get(*pos + 1) == Some(&b'$')) =>
                {
                    *pos += 1;
                    if item_count == 0 {
                        head = Some((separator as char).to_string());
                        head_quoted = false;
                    }
                    item_count += 1;
                    direct_atom_count += 1;
                }
                _ => {
                    let atom = parse_atom(bytes, pos, *dollar_quote);
                    if item_count == 0 {
                        head = atom.as_atom().map(str::to_string);
                        head_quoted = atom.is_quoted();
                    }
                    item_count += 1;
                    direct_atom_count += 1;
                }
            }
        }
    }

    let bytes = input.as_bytes();
    let mut pos = 0usize;
    let mut matches = Vec::new();
    let mut dollar_quote = false;
    let root = scan_list(
        bytes,
        &mut pos,
        0,
        child_name,
        &mut matches,
        &mut dollar_quote,
        None,
    )?;
    skip_trivia(bytes, &mut pos, false)?;
    if pos != bytes.len() {
        return Err(ParseError {
            message: "unexpected trailing data".into(),
            position: pos,
        });
    }
    Ok((root.end - 1, matches))
}

/// Java's scanner treats `#` as a line comment in its ordinary state, but as
/// an identifier character in its NAME/LAYER_NAME states.  The latter is how
/// unquoted KiCad power-net names such as `#PWR01` are read immediately after
/// `(net`, `(class`, and the other name-taking keywords.
fn skip_trivia(bytes: &[u8], pos: &mut usize, hash_is_identifier: bool) -> Result<(), ParseError> {
    loop {
        while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
            *pos += 1;
        }
        // In YYINITIAL the Java lexer accepts `#comment` as well as
        // `# comment`. Name-taking scopes are the only state where a leading
        // hash belongs to the token (for example `#PWR01`).
        if !hash_is_identifier && bytes.get(*pos) == Some(&b'#') {
            while *pos < bytes.len() && !matches!(bytes[*pos], b'\r' | b'\n') {
                *pos += 1;
            }
            continue;
        }
        if bytes.get(*pos..*pos + 2) == Some(b"/*") {
            let start = *pos;
            *pos += 2;
            while bytes.get(*pos..*pos + 2) != Some(b"*/") {
                if *pos >= bytes.len() {
                    return Err(ParseError {
                        message: "unterminated comment".into(),
                        position: start,
                    });
                }
                *pos += 1;
            }
            *pos += 2;
            continue;
        }
        return Ok(());
    }
}

/// Whether the next atom is scanned in Java's NAME/LAYER_NAME state.
///
/// Most scanner transitions in `SpecctraFileDescription.flex` last for one
/// atom.  A handful of parser routines then deliberately bypass the scanner
/// and call `next_string`/`next_string_list` (or explicitly re-enter `NAME`)
/// for the remainder of a scope.  Those scopes are important for names such
/// as `#PWR01`: treating `#` as a comment there either drops the rest of the
/// line or creates an unterminated S-expression.  Keep the two kinds of
/// state explicit here: ordinary keyword transitions allow only their first
/// argument, while the raw-name scopes allow every direct atom.
fn hash_identifier_position(
    head: Option<(&str, bool)>,
    parent_head: Option<&str>,
    item_count: usize,
    saw_child: bool,
    direct_atom_count: usize,
) -> bool {
    let Some((head, quoted)) = head else {
        return false;
    };
    if quoted {
        return false;
    }
    let head = head.to_ascii_lowercase();

    // `pin` has three parent-specific grammars. Package.read_pin_info reads a
    // padstack and pin name (with an optional rotate child between them), a
    // logical-part pin names direct arguments 1/3/5, and a placement override
    // names only its first argument. Track direct atoms so nested children do
    // not consume a NAME position.
    if head == "pin" {
        return match parent_head.map(str::to_ascii_lowercase).as_deref() {
            Some("logical_part") => matches!(direct_atom_count, 1 | 3 | 5),
            Some("place") => direct_atom_count == 1,
            _ => (1..=2).contains(&direct_atom_count),
        };
    }

    // A top-level network/rules via declaration has three named fields;
    // wiring and SES route vias name only their padstack. The same keyword is
    // used for both grammars, so its scanner state depends on the parent.
    if head == "via" {
        return match parent_head.map(str::to_ascii_lowercase).as_deref() {
            Some("network" | "rules") => !saw_child && matches!(direct_atom_count, 1..=3),
            _ => direct_atom_count == 1,
        };
    }

    // Logical-part mappings explicitly re-enter NAME for every component.
    // Ordinary placement/component scopes use the lexer's one-token `comp`
    // transition and then return to comment-aware YYINITIAL.
    if head == "comp" {
        return if parent_head
            .is_some_and(|parent| parent.eq_ignore_ascii_case("logical_part_mapping"))
        {
            !saw_child && direct_atom_count >= 1
        } else {
            direct_atom_count == 1
        };
    }

    // Net-class rules accept a list of layers; autoroute settings accept one
    // layer followed by setting children. Both spell the scope `layer_rule`.
    if head == "layer_rule" {
        return if parent_head
            .is_some_and(|parent| parent.eq_ignore_ascii_case("autoroute_settings"))
        {
            direct_atom_count == 1
        } else {
            !saw_child && direct_atom_count >= 1
        };
    }

    // Fixed-arity readers explicitly enter NAME for only these positions.
    // A wiring `(via PADSTACK X Y # comment)` is back in YYINITIAL after the
    // coordinates; treating `via` as an unbounded raw scope turns that legal
    // comment into a fourth argument and makes the strict importer reject it.
    if head == "constant" {
        return !saw_child && matches!(direct_atom_count, 1 | 2);
    }

    // Raw `next_string_list` readers stop at the first nested scope.  Once a
    // child has been consumed, Java is back in YYINITIAL and `#` resumes its
    // ordinary end-of-line-comment meaning.
    if saw_child {
        return false;
    }

    // These parser methods consume a raw string list or explicitly put the
    // scanner in NAME before every token.  In particular, `pins`, `order`,
    // and `fromto` are not scanner state transitions in the .flex file even
    // though their component/pin names are parsed by `next_string`.
    let raw_name_scope = matches!(
        head.as_str(),
        "class"
            | "classes"
            | "fromto"
            | "order"
            | "pins"
            | "type"
            | "use_layer"
            | "use_net"
            | "use_via"
            | "via_rule"
    );
    if raw_name_scope {
        return item_count >= 1;
    }

    // The file readers explicitly enter NAME for their document header even
    // though `pcb`, `session`, and `rules` do not switch state in the lexer.
    // A rules header has the extra literal `PCB` before its design name.
    if matches!(head.as_str(), "pcb" | "session") && item_count == 1 {
        return true;
    }
    if head == "rules" && item_count == 2 {
        return true;
    }

    // YYINITIAL -> NAME/LAYER_NAME transitions in
    // `SpecctraFileDescription.flex`; each transition lasts for exactly one
    // atom.  Keeping this table narrow preserves ordinary `#` comments in
    // all other scanner states.
    item_count == 1
        && matches!(
            head.as_str(),
            "clearance_class"
                | "comp"
                | "component"
                | "image"
                | "host_cad"
                | "host_version"
                | "keepout"
                | "layer"
                | "logical_part"
                | "logical_part_mapping"
                | "net"
                | "padstack"
                | "place"
                | "place_keepout"
                | "plane"
                | "use_net"
                | "via_keepout"
                | "wire"
                // YYINITIAL -> LAYER_NAME
                | "circ"
                | "circle"
                | "path"
                | "poly"
                | "polygon"
                | "polyline_path"
                | "rect"
                | "rectangle"
        )
}

fn parse_expr(
    bytes: &[u8],
    pos: &mut usize,
    dollar_quote: &mut bool,
    parent_head: Option<&str>,
) -> Result<SExpr, ParseError> {
    skip_trivia(bytes, pos, false)?;
    if *pos >= bytes.len() {
        return Err(ParseError {
            message: "unexpected end of input".into(),
            position: *pos,
        });
    }
    if bytes[*pos] != b'(' {
        return Err(ParseError {
            message: "expected '('".into(),
            position: *pos,
        });
    }
    *pos += 1;
    let mut items: Vec<SExpr> = Vec::new();
    let mut direct_atom_count = 0usize;
    let mut saw_child = false;
    loop {
        let hash_is_identifier = hash_identifier_position(
            items
                .first()
                .and_then(|item| item.as_atom().map(|name| (name, item.is_quoted()))),
            parent_head,
            items.len(),
            saw_child,
            direct_atom_count,
        );
        skip_trivia(bytes, pos, hash_is_identifier)?;
        if *pos >= bytes.len() {
            return Err(ParseError {
                message: "unterminated list".into(),
                position: *pos,
            });
        }
        match bytes[*pos] {
            b')' => {
                *pos += 1;
                return Ok(SExpr::List(items));
            }
            b'(' => {
                let child = parse_expr(
                    bytes,
                    pos,
                    dollar_quote,
                    items.first().and_then(SExpr::as_atom),
                )?;
                items.push(child);
                saw_child = true;
            }
            q @ (b'"' | b'\'') => {
                // Special case: `(string_quote ")` (or `'`) names the quote
                // char itself. This exception is scoped to that keyword;
                // otherwise a quote followed by whitespace starts a valid
                // quoted identifier whose first character is whitespace.
                let next = bytes.get(*pos + 1);
                let is_string_quote_scope = items
                    .first()
                    .and_then(SExpr::as_atom)
                    .is_some_and(|name| name.eq_ignore_ascii_case("string_quote"));
                if is_string_quote_scope
                    && next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')')
                {
                    *dollar_quote = false;
                    items.push(SExpr::Atom((q as char).to_string()));
                    *pos += 1;
                } else {
                    items.push(parse_quoted(bytes, pos, q)?);
                }
                direct_atom_count += 1;
            }
            b'$' => {
                let next = bytes.get(*pos + 1);
                let is_string_quote_scope = items
                    .first()
                    .and_then(SExpr::as_atom)
                    .is_some_and(|name| name.eq_ignore_ascii_case("string_quote"));
                if is_string_quote_scope
                    && next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')')
                {
                    *dollar_quote = true;
                    items.push(SExpr::Atom("$".to_string()));
                    *pos += 1;
                } else if *dollar_quote {
                    items.push(parse_quoted(bytes, pos, b'$')?);
                } else {
                    items.push(parse_atom(bytes, pos, false));
                }
                direct_atom_count += 1;
            }
            // Composite clearance types glue quoted names with a bare
            // separator: `(type "A B"_"C D")` (Java writes `_`, scans
            // with `-`). The separator must be its own token or the
            // following quote would be swallowed into an atom, fragmenting
            // the quoted name at its spaces (Issue029's class names).
            // All Specctra quote characters count (a `(string_quote ')` or
            // `(string_quote $)` document glues with its delimiter).
            s @ (b'_' | b'-')
                if matches!(bytes.get(*pos + 1), Some(&b'"') | Some(&b'\''))
                    || (*dollar_quote && bytes.get(*pos + 1) == Some(&b'$')) =>
            {
                items.push(SExpr::Atom((s as char).to_string()));
                *pos += 1;
                direct_atom_count += 1;
            }
            _ => {
                items.push(parse_atom(bytes, pos, *dollar_quote));
                direct_atom_count += 1;
            }
        }
    }
}

fn parse_quoted(bytes: &[u8], pos: &mut usize, quote: u8) -> Result<SExpr, ParseError> {
    debug_assert_eq!(bytes[*pos], quote);
    let start = *pos;
    *pos += 1;
    let content_start = *pos;
    while *pos < bytes.len() && bytes[*pos] != quote {
        // Specctra has no escape production inside quoted identifiers:
        // backslashes are ordinary data in both the Java lexer and the file
        // format. In particular, `\"` is a backslash followed by the closing
        // double quote, not an escaped quote.
        *pos += 1;
    }
    if *pos >= bytes.len() {
        return Err(ParseError {
            message: "unterminated quoted string".into(),
            position: start,
        });
    }
    let content = String::from_utf8_lossy(&bytes[content_start..*pos]).into_owned();
    *pos += 1;
    Ok(SExpr::Quoted(content, quote as char))
}

fn parse_atom(bytes: &[u8], pos: &mut usize, dollar_quote: bool) -> SExpr {
    let start = *pos;
    // A quote inside an ordinary scanner atom is data.  The one exception is
    // a composite-clearance separator immediately followed by a quoted class
    // name (`default_"1A EXTERNAL 1oz"`); there the separator finishes the
    // bare atom and the quote starts the next token.
    while *pos < bytes.len() {
        let byte = bytes[*pos];
        if byte.is_ascii_whitespace() || byte == b'(' || byte == b')' {
            break;
        }
        if (matches!(byte, b'"' | b'\'') || (dollar_quote && byte == b'$'))
            && *pos > start
            && matches!(bytes[*pos - 1], b'_' | b'-')
            && bytes[*pos + 1..].contains(&byte)
        {
            break;
        }
        *pos += 1;
    }
    SExpr::Atom(String::from_utf8_lossy(&bytes[start..*pos]).into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_lists_and_quotes() {
        let expr = parse_dsn(
            r#"(pcb "my board.dsn"
                 (parser (string_quote ") (space_in_quoted_tokens on))
                 (structure (layer F.Cu (type signal)))
               )"#,
        )
        .unwrap();
        assert_eq!(expr.name(), Some("pcb"));
        assert_eq!(expr.as_list().unwrap()[1].as_atom(), Some("my board.dsn"));
        let parser = expr.child("parser").unwrap();
        assert_eq!(parser.child("string_quote").unwrap().arg(), Some("\""));
        let structure = expr.child("structure").unwrap();
        let layer = structure.child("layer").unwrap();
        assert_eq!(layer.arg(), Some("F.Cu"));
        assert_eq!(layer.child("type").unwrap().arg(), Some("signal"));
    }

    #[test]
    fn errors_are_reported() {
        assert!(parse_dsn("").is_err());
        assert!(parse_dsn("atom").is_err());
        assert!(parse_dsn("(unterminated").is_err());
        assert!(parse_dsn("(bad \"unterminated)").is_err());
    }

    #[test]
    fn parses_real_fixture_files() {
        // the repo fixtures are real KiCad exports
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        for fixture in ["empty_board.dsn", "Issue093-interf_u.dsn"] {
            let path = format!("{root}/fixtures/{fixture}");
            let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
            let expr =
                parse_dsn(&content).unwrap_or_else(|e| panic!("failed to parse {fixture}: {e}"));
            assert_eq!(expr.name(), Some("pcb"), "{fixture} root");
            let structure = expr.child("structure").expect("structure");
            assert!(structure.children("layer").count() >= 2, "{fixture} layers");
            assert!(structure.child("boundary").is_some(), "{fixture} boundary");
            // the boundary path has coordinates
            let path_node = structure.child("boundary").unwrap().child("path");
            if let Some(p) = path_node {
                let coords: Vec<f64> = p.args().skip(2).filter_map(|a| a.parse().ok()).collect();
                assert!(coords.len() >= 8, "{fixture} boundary coords");
            }
        }
    }

    #[test]
    fn preserves_quote_provenance_for_composite_names() {
        let expr = parse_dsn(
            r#"(rule (clearance 1 (type default_'A B'))
                    (clearance 2 (type "A-B" "C")))"#,
        )
        .unwrap();
        let rules = expr.as_list().unwrap();
        let first_type = rules[1].as_list().unwrap()[2].as_list().unwrap();
        assert_eq!(first_type[1].as_atom(), Some("default_"));
        assert!(!first_type[1].is_quoted());
        assert_eq!(first_type[2].as_atom(), Some("A B"));
        assert!(first_type[2].is_quoted());
        let second_type = rules[2].as_list().unwrap()[2].as_list().unwrap();
        assert_eq!(second_type[1].as_atom(), Some("A-B"));
        assert!(second_type[1].is_quoted());
    }

    #[test]
    fn apostrophe_inside_unquoted_atom_is_not_a_quote() {
        let expr = parse_dsn("(net can't)").unwrap();
        assert_eq!(expr.as_list().unwrap()[1].as_atom(), Some("can't"));
        let expr = parse_dsn("(net rock'n'roll)").unwrap();
        assert_eq!(expr.as_list().unwrap()[1].as_atom(), Some("rock'n'roll"));
    }

    #[test]
    fn supports_specctra_dollar_quote_delimiter() {
        let expr = parse_dsn(
            r#"(pcb board
  (parser (string_quote $) (space_in_quoted_tokens on))
  (network (net $N 1$))
)"#,
        )
        .expect("dollar is a legal Specctra quote delimiter");
        assert_eq!(expr.arg(), Some("board"));
        assert_eq!(
            expr.child("parser")
                .and_then(|parser| parser.child("string_quote"))
                .and_then(SExpr::arg),
            Some("$")
        );
        assert_eq!(
            expr.child("network")
                .and_then(|network| network.child("net"))
                .and_then(SExpr::arg),
            Some("N 1")
        );
    }

    #[test]
    fn quoted_identifiers_may_start_with_whitespace() {
        let expr = parse_dsn("(net \" leading and trailing \t\")").unwrap();
        assert_eq!(expr.arg(), Some(" leading and trailing \t"));
    }

    #[test]
    fn quoted_backslashes_are_literal_specctra_data() {
        let expr = parse_dsn(r#"(net "a\\b")"#).unwrap();
        assert_eq!(expr.arg(), Some(r"a\\b"));
        let expr = parse_dsn(r#"(net "a\" tail)"#).unwrap();
        assert_eq!(expr.arg(), Some(r"a\"));
        assert_eq!(expr.as_list().unwrap()[2].as_atom(), Some("tail"));
    }

    #[test]
    fn rejects_a_second_top_level_document() {
        let error = parse_dsn("(pcb one) (pcb two)").expect_err("trailing scope must fail");
        assert_eq!(error.position, 10);
        assert_eq!(error.message, "unexpected trailing data");
    }

    #[test]
    fn specctra_comments_are_grammar_trivia() {
        let expr = parse_dsn(
            "#heading\n(pcb /* before name */ board\n  #before structure\n  (structure)) /* trailing */\n",
        )
        .expect("comments are ignored between tokens and after the document");
        assert_eq!(expr.name(), Some("pcb"));
        assert_eq!(expr.arg(), Some("board"));
        assert!(expr.child("structure").is_some());
        let error = parse_dsn("(pcb board) /* never closed")
            .expect_err("an unterminated comment must be diagnosed");
        assert_eq!(error.message, "unterminated comment");
    }

    #[test]
    fn hash_prefixed_names_use_java_name_state() {
        let expr = parse_dsn(
            "(pcb board\n  (network\n    (net #PWR01 (pins U1-1 U2-1))\n    (class #POWER '#PWR01' (circuit (use_net #PWR01)))\n  )\n  # ordinary scanner-state comment\n  (wiring (wire (path #Signal 100 0 0 10 10) (net #PWR01)))\n)",
        )
        .expect("hash-prefixed identifiers must not become comments");
        let network = expr.child("network").expect("network");
        assert_eq!(network.child("net").and_then(SExpr::arg), Some("#PWR01"));
        let class = network.child("class").expect("class");
        assert_eq!(class.args().collect::<Vec<_>>(), vec!["#POWER", "#PWR01"]);
        assert_eq!(
            class
                .child("circuit")
                .and_then(|circuit| circuit.child("use_net"))
                .and_then(SExpr::arg),
            Some("#PWR01")
        );
        assert_eq!(
            expr.child("wiring")
                .and_then(|wiring| wiring.child("wire"))
                .and_then(|wire| wire.child("net"))
                .and_then(SExpr::arg),
            Some("#PWR01")
        );
        assert_eq!(
            expr.child("wiring")
                .and_then(|wiring| wiring.child("wire"))
                .and_then(|wire| wire.child("path"))
                .and_then(SExpr::arg),
            Some("#Signal")
        );
    }

    #[test]
    fn hash_prefixed_document_names_use_reader_name_state() {
        let pcb = parse_dsn("(pcb #BOARD\n  (structure))")
            .expect("DSN reader enters NAME for the board name");
        assert_eq!(pcb.arg(), Some("#BOARD"));

        let session = parse_dsn("(session #BOARD.ses\n  (routes))")
            .expect("SES reader enters NAME for the session name");
        assert_eq!(session.arg(), Some("#BOARD.ses"));

        let rules = parse_dsn("(rules PCB #BOARD\n  (rule (width 1)))")
            .expect("rules reader enters NAME for the design name");
        assert_eq!(rules.args().collect::<Vec<_>>(), vec!["PCB", "#BOARD"]);
    }

    #[test]
    fn hash_prefixed_names_survive_raw_name_scopes() {
        // The Java parser has several readers which deliberately bypass the
        // scanner (`next_string_list`) or re-enter NAME for every atom.  A
        // generic S-expression parser must retain that context; otherwise a
        // legal `#PWR...` component/pin/class name is mistaken for a comment.
        let expr = parse_dsn(
            r#"(pcb board
  (class #CLASS #NET_A #NET_B)
  (classes #CLASS_A #CLASS_B)
  (pins #U1-#1 #U2-#2)
  (order #U3-#3 #U4-#4)
  (fromto #U5-#5 #U6-#6)
  (via_rule #VR_MAIN #VIA_STACK)
  (pin #PADSTACK #PIN 0 0)
  (type #TRACE_CLASS #VIA_CLASS)
  (use_net #NET_A #NET_B)
  (constant #KEY #VALUE)
  (network (via #VIA_INFO #VIA_PAD #CLEARANCE_CLASS))
  (part_library
    (logical_part_mapping #MAPPING (comp #COMP_A #COMP_B)))
)"#,
        )
        .expect("raw name scopes must preserve hash-prefixed identifiers");

        for (scope, expected) in [
            ("class", vec!["#CLASS", "#NET_A", "#NET_B"]),
            ("classes", vec!["#CLASS_A", "#CLASS_B"]),
            ("pins", vec!["#U1-#1", "#U2-#2"]),
            ("order", vec!["#U3-#3", "#U4-#4"]),
            ("fromto", vec!["#U5-#5", "#U6-#6"]),
            ("via_rule", vec!["#VR_MAIN", "#VIA_STACK"]),
            ("pin", vec!["#PADSTACK", "#PIN", "0", "0"]),
            ("type", vec!["#TRACE_CLASS", "#VIA_CLASS"]),
            ("use_net", vec!["#NET_A", "#NET_B"]),
            ("constant", vec!["#KEY", "#VALUE"]),
        ] {
            let node = expr.child(scope).expect("scope present");
            assert_eq!(node.args().collect::<Vec<_>>(), expected, "scope {scope}");
        }
        let via = expr
            .child("network")
            .and_then(|network| network.child("via"))
            .expect("network via declaration");
        assert_eq!(
            via.args().collect::<Vec<_>>(),
            vec!["#VIA_INFO", "#VIA_PAD", "#CLEARANCE_CLASS"]
        );
        let comp = expr
            .child("part_library")
            .and_then(|library| library.child("logical_part_mapping"))
            .and_then(|mapping| mapping.child("comp"))
            .expect("logical-part component list");
        assert_eq!(comp.args().collect::<Vec<_>>(), vec!["#COMP_A", "#COMP_B"]);
    }

    #[test]
    fn pin_hash_state_is_positional_around_rotation_and_coordinates() {
        let comment = parse_dsn("(pin Pad 1 0 0 #comment\n)")
            .expect("a trailing hash is a comment after pin coordinates");
        assert_eq!(
            comment.args().collect::<Vec<_>>(),
            vec!["Pad", "1", "0", "0"]
        );

        let rotated = parse_dsn("(pin Pad (rotate 90) #PIN 0 0)")
            .expect("Java re-enters NAME for the pin name after rotate");
        assert_eq!(
            rotated.args().collect::<Vec<_>>(),
            vec!["Pad", "#PIN", "0", "0"]
        );
        assert_eq!(rotated.child("rotate").and_then(SExpr::arg), Some("90"));
    }

    #[test]
    fn pin_name_state_follows_its_parent_grammar() {
        let input = r#"(pcb board
  (part_library
    (logical_part #LP
      (pin #P 0 #G 1 #GP 2 # trailing logical-part comment
      )
    )
  )
  (placement
    (component #IMAGE
      (place #U1 0 0 front 0
        (pin #P (clearance_class #STRICT) # trailing placement comment
        )
      )
    )
  )
)"#;
        let expr = parse_dsn(input).expect("parent-specific pin NAME states must parse");
        let logical_pin = expr
            .child("part_library")
            .and_then(|part_library| part_library.child("logical_part"))
            .and_then(|logical_part| logical_part.child("pin"))
            .expect("logical-part pin");
        assert_eq!(
            logical_pin.args().collect::<Vec<_>>(),
            vec!["#P", "0", "#G", "1", "#GP", "2"]
        );

        let placement_pin = expr
            .child("placement")
            .and_then(|placement| placement.child("component"))
            .and_then(|component| component.child("place"))
            .and_then(|place| place.child("pin"))
            .expect("placement pin override");
        assert_eq!(placement_pin.args().collect::<Vec<_>>(), vec!["#P"]);
        assert_eq!(
            placement_pin.child("clearance_class").and_then(SExpr::arg),
            Some("#STRICT")
        );

        let (root_close, wiring) = document_structure(input, "wiring")
            .expect("source-span scanner must use the same parent-specific states");
        assert_eq!(&input[root_close..=root_close], ")");
        assert!(wiring.is_empty());
    }

    #[test]
    fn layer_rule_keeps_every_hash_prefixed_layer_until_its_rule_child() {
        let expr = parse_dsn("(class C (layer_rule F.Cu #Inner.Cu\n  (rule (width 10))))")
            .expect("Java re-enters LAYER_NAME for every layer_rule name");
        let layer_rule = expr.child("layer_rule").expect("layer rule");

        assert_eq!(
            layer_rule.args().collect::<Vec<_>>(),
            vec!["F.Cu", "#Inner.Cu"]
        );
        assert!(layer_rule.child("rule").is_some());
    }

    #[test]
    fn fixed_arity_name_scopes_resume_hash_comments_after_their_arguments() {
        let expr = parse_dsn("(wiring (via #PAD 10 20 # trailing comment\n))")
            .expect("a trailing wiring-via comment is ordinary trivia");
        let via = expr.child("via").expect("wiring via");
        assert_eq!(via.args().collect::<Vec<_>>(), vec!["#PAD", "10", "20"]);

        let expr = parse_dsn("(network (via #INFO #PAD #CLASS attach # trailing comment\n))")
            .expect("the three via declaration names use NAME state");
        let declaration = expr.child("via").expect("network via declaration");
        assert_eq!(
            declaration.args().collect::<Vec<_>>(),
            vec!["#INFO", "#PAD", "#CLASS", "attach"]
        );

        let constant = parse_dsn("(constant #KEY #VALUE # trailing comment\n)")
            .expect("a parser constant has exactly two NAME-state arguments");
        assert_eq!(constant.args().collect::<Vec<_>>(), vec!["#KEY", "#VALUE"]);
    }

    #[test]
    fn overloaded_name_scopes_restore_comments_in_their_short_forms() {
        let placement =
            parse_dsn("(placement (comp #IMAGE # trailing comment\n  (place U1 0 0 front 0)))")
                .expect("placement comp names only its image");
        let comp = placement.child("comp").expect("component placement");
        assert_eq!(comp.args().collect::<Vec<_>>(), vec!["#IMAGE"]);
        assert!(comp.child("place").is_some());

        let autoroute = parse_dsn(
            "(autoroute_settings (layer_rule #Signal # trailing comment\n  (active on)))",
        )
        .expect("autoroute layer_rule names one layer");
        let layer_rule = autoroute.child("layer_rule").expect("autoroute layer rule");
        assert_eq!(layer_rule.args().collect::<Vec<_>>(), vec!["#Signal"]);
        assert!(layer_rule.child("active").is_some());
    }

    #[test]
    fn raw_name_scope_returns_to_comment_state_after_child_scope() {
        let expr = parse_dsn(
            "(pcb board\n  (class #CLASS (rule (clearance 1)) # comment after the rule\n)\n)\n",
        )
        .expect("a comment after a nested raw scope remains legal trivia");
        let class = expr.child("class").expect("class scope");
        assert_eq!(class.arg(), Some("#CLASS"));
        assert!(class.child("rule").is_some());
        assert_eq!(class.args().collect::<Vec<_>>(), vec!["#CLASS"]);
    }

    #[test]
    fn structural_spans_ignore_quoted_and_commented_wiring_text() {
        let input = concat!(
            "(pcb \"(wiring quoted)\"\n",
            "  # (wiring line-comment)\n",
            "  /* (wiring block-comment) */\n",
            "  (network (net \"N\"))\n",
            "  (wiring (wire route))\n",
            ")\n",
            "# trailing close )\n",
        );
        let (root_close, spans) = document_structure(input, "wiring").expect("scan");
        assert_eq!(&input[root_close..=root_close], ")");
        assert!(input[root_close + 1..].contains("trailing close )"));
        assert_eq!(spans.len(), 1, "only the real top-level scope matches");
        assert_eq!(&input[spans[0].clone()], "(wiring (wire route))");
    }
}
