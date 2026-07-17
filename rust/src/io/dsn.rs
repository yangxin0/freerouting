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

#[derive(Debug, Clone, PartialEq)]
pub enum SExpr {
    Atom(String),
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
            SExpr::List(_) => None,
        }
    }

    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::Atom(_) => None,
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
    Ok(expr)
}

fn skip_whitespace(bytes: &[u8], pos: &mut usize) {
    while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
        *pos += 1;
    }
}

fn parse_expr(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    skip_whitespace(bytes, pos);
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
        skip_whitespace(bytes, pos);
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
                // char itself; a lone quote followed by whitespace/`)` is
                // an atom.
                let next = bytes.get(*pos + 1);
                if next.is_none_or(|c| c.is_ascii_whitespace() || *c == b')') {
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
        *pos += 1;
    }
    if *pos >= bytes.len() {
        return Err(ParseError {
            message: "unterminated quoted string".into(),
            position: start,
        });
    }
    let content = String::from_utf8_lossy(&bytes[content_start..*pos]).into_owned();
    *pos += 1; // closing quote
    Ok(SExpr::Atom(content))
}

fn parse_atom(bytes: &[u8], pos: &mut usize) -> SExpr {
    let start = *pos;
    // an atom also ends at a double quote: composite clearance types glue
    // a bare name to a quoted one (`default_"1A EXTERNAL 1oz"`), and
    // swallowing the quote fragmented the quoted name at its spaces
    while *pos < bytes.len()
        && !bytes[*pos].is_ascii_whitespace()
        && bytes[*pos] != b'('
        && bytes[*pos] != b')'
        && bytes[*pos] != b'"'
    {
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
}
