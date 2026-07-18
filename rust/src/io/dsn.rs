//! S-expression reader for Specctra DSN/SES files (port of the scanner
//! layer under `io/specctra/parser`).
//!
//! DSN files are parenthesized token trees. The `string_quote` parser
//! setting names the quote character (KiCad emits `"`); a quote character
//! immediately following `(string_quote ` is itself a token. Quoted
//! strings may contain spaces when `space_in_quoted_tokens` is on; this
//! reader always allows them. Like the Java scanner (STRING1/STRING2
//! states), BOTH `"` and `'` start a quoted string at token start,
//! independent of the declared `string_quote`; inside an atom either
//! quote char is an ordinary character (SpecChar3).

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
    let expr = parse_expr(bytes, &mut pos)?;
    skip_trivia(bytes, &mut pos)?;
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
    ) -> Result<Range<usize>, ParseError> {
        skip_trivia(bytes, pos)?;
        let start = *pos;
        if bytes.get(*pos) != Some(&b'(') {
            return Err(ParseError {
                message: "expected '('".into(),
                position: *pos,
            });
        }
        *pos += 1;
        let mut head: Option<String> = None;
        let mut item_count = 0usize;
        loop {
            skip_trivia(bytes, pos)?;
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
                    scan_list(bytes, pos, depth + 1, child_name, matches)?;
                    item_count += 1;
                }
                quote @ (b'"' | b'\'') => {
                    let next = bytes.get(*pos + 1);
                    let is_string_quote_scope = head
                        .as_deref()
                        .is_some_and(|name| name.eq_ignore_ascii_case("string_quote"));
                    let value = if is_string_quote_scope
                        && next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')')
                    {
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
                    }
                    item_count += 1;
                }
                separator @ (b'_' | b'-')
                    if matches!(bytes.get(*pos + 1), Some(&b'"') | Some(&b'\'')) =>
                {
                    *pos += 1;
                    if item_count == 0 {
                        head = Some((separator as char).to_string());
                    }
                    item_count += 1;
                }
                _ => {
                    let atom = parse_atom(bytes, pos);
                    if item_count == 0 {
                        head = atom.as_atom().map(str::to_string);
                    }
                    item_count += 1;
                }
            }
        }
    }

    let bytes = input.as_bytes();
    let mut pos = 0usize;
    let mut matches = Vec::new();
    let root = scan_list(bytes, &mut pos, 0, child_name, &mut matches)?;
    skip_trivia(bytes, &mut pos)?;
    if pos != bytes.len() {
        return Err(ParseError {
            message: "unexpected trailing data".into(),
            position: pos,
        });
    }
    Ok((root.end - 1, matches))
}

fn skip_trivia(bytes: &[u8], pos: &mut usize) -> Result<(), ParseError> {
    loop {
        while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
            *pos += 1;
        }
        if bytes.get(*pos) == Some(&b'#') {
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

fn parse_expr(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    skip_trivia(bytes, pos)?;
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
    let mut items = Vec::new();
    loop {
        skip_trivia(bytes, pos)?;
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
                items.push(parse_expr(bytes, pos)?);
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
                    items.push(SExpr::Atom((q as char).to_string()));
                    *pos += 1;
                } else {
                    items.push(parse_quoted(bytes, pos, q)?);
                }
            }
            // Composite clearance types glue quoted names with a bare
            // separator: `(type "A B"_"C D")` (Java writes `_`, scans
            // with `-`). The separator must be its own token or the
            // following quote would be swallowed into an atom, fragmenting
            // the quoted name at its spaces (Issue029's class names).
            // Both quote characters count (a `(string_quote ')` document
            // glues with single quotes).
            s @ (b'_' | b'-') if matches!(bytes.get(*pos + 1), Some(&b'"') | Some(&b'\'')) => {
                items.push(SExpr::Atom((s as char).to_string()));
                *pos += 1;
            }
            _ => {
                items.push(parse_atom(bytes, pos));
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

fn parse_atom(bytes: &[u8], pos: &mut usize) -> SExpr {
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
        if matches!(byte, b'"' | b'\'')
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
            "# heading\n(pcb /* before name */ board\n  # before structure\n  (structure)) /* trailing */\n",
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
