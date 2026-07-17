//! Minimal JSON parser for the KiCad board JSON reader (the crate has no
//! external dependencies; this covers the subset the format uses), plus
//! the one string escaper every JSON writer in the crate shares.

use std::collections::HashMap;

/// Escapes a string for embedding in a JSON document: quotes, backslashes,
/// and ALL control characters (RFC 8259 requires U+0000..U+001F escaped —
/// a net name with an embedded newline or tab must not corrupt the file).
pub fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(HashMap<String, Json>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.get(key),
            _ => None,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn as_arr(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }
    pub fn num(&self, key: &str) -> f64 {
        self.get(key).and_then(|v| v.as_f64()).unwrap_or(0.0)
    }
    pub fn str_or(&self, key: &str, default: &str) -> String {
        self.get(key)
            .and_then(|v| v.as_str())
            .unwrap_or(default)
            .to_string()
    }
    pub fn arr(&self, key: &str) -> &[Json] {
        self.get(key).and_then(|v| v.as_arr()).unwrap_or(&[])
    }
}

pub fn parse_json(input: &str) -> Result<Json, String> {
    let mut p = Parser {
        bytes: input.as_bytes(),
        pos: 0,
    };
    p.skip_ws();
    let v = p.value()?;
    p.skip_ws();
    if p.pos != p.bytes.len() {
        return Err(format!("trailing data at byte {}", p.pos));
    }
    Ok(v)
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn skip_ws(&mut self) {
        while self.pos < self.bytes.len()
            && matches!(self.bytes[self.pos], b' ' | b'\t' | b'\n' | b'\r')
        {
            self.pos += 1;
        }
    }
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }
    fn expect(&mut self, c: u8) -> Result<(), String> {
        if self.peek() == Some(c) {
            self.pos += 1;
            Ok(())
        } else {
            Err(format!("expected '{}' at byte {}", c as char, self.pos))
        }
    }
    fn value(&mut self) -> Result<Json, String> {
        self.skip_ws();
        match self.peek() {
            Some(b'{') => self.object(),
            Some(b'[') => self.array(),
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => self.literal("true", Json::Bool(true)),
            Some(b'f') => self.literal("false", Json::Bool(false)),
            Some(b'n') => self.literal("null", Json::Null),
            Some(_) => self.number(),
            None => Err("unexpected end".into()),
        }
    }
    fn literal(&mut self, lit: &str, v: Json) -> Result<Json, String> {
        if self.bytes[self.pos..].starts_with(lit.as_bytes()) {
            self.pos += lit.len();
            Ok(v)
        } else {
            Err(format!("bad literal at byte {}", self.pos))
        }
    }
    fn number(&mut self) -> Result<Json, String> {
        let start = self.pos;
        while self
            .peek()
            .is_some_and(|c| c.is_ascii_digit() || matches!(c, b'-' | b'+' | b'.' | b'e' | b'E'))
        {
            self.pos += 1;
        }
        std::str::from_utf8(&self.bytes[start..self.pos])
            .ok()
            .and_then(|s| s.parse().ok())
            .map(Json::Num)
            .ok_or_else(|| format!("bad number at byte {start}"))
    }
    fn string(&mut self) -> Result<String, String> {
        self.expect(b'"')?;
        let mut out = String::new();
        loop {
            match self.peek() {
                Some(b'"') => {
                    self.pos += 1;
                    return Ok(out);
                }
                Some(b'\\') => {
                    self.pos += 1;
                    match self.peek() {
                        Some(b'n') => out.push('\n'),
                        Some(b't') => out.push('\t'),
                        // the remaining RFC 8259 escapes — falling through
                        // to the identity case turned \r, \b and \f into
                        // literal 'r', 'b' and 'f'
                        Some(b'r') => out.push('\r'),
                        Some(b'b') => out.push('\u{0008}'),
                        Some(b'f') => out.push('\u{000c}'),
                        Some(b'u') => {
                            // \uXXXX, with UTF-16 surrogate PAIRS combined
                            // (decoding each half separately corrupted
                            // e.g. 😀 into two '?')
                            let read_hex = |bytes: &[u8], pos: usize| -> Option<u32> {
                                bytes
                                    .get(pos..pos + 4)
                                    .and_then(|h| std::str::from_utf8(h).ok())
                                    .and_then(|h| u32::from_str_radix(h, 16).ok())
                            };
                            let Some(unit) = read_hex(self.bytes, self.pos + 1) else {
                                return Err(format!("bad \\u escape at byte {}", self.pos));
                            };
                            self.pos += 4;
                            let scalar = if (0xd800..0xdc00).contains(&unit) {
                                // high surrogate: the low half must follow
                                if self.bytes.get(self.pos + 1) != Some(&b'\\')
                                    || self.bytes.get(self.pos + 2) != Some(&b'u')
                                {
                                    return Err("lone high surrogate in \\u escape".into());
                                }
                                let Some(low) = read_hex(self.bytes, self.pos + 3) else {
                                    return Err(format!("bad \\u escape at byte {}", self.pos));
                                };
                                if !(0xdc00..0xe000).contains(&low) {
                                    return Err("invalid low surrogate in \\u escape".into());
                                }
                                self.pos += 6;
                                0x10000 + ((unit - 0xd800) << 10) + (low - 0xdc00)
                            } else if (0xdc00..0xe000).contains(&unit) {
                                return Err("lone low surrogate in \\u escape".into());
                            } else {
                                unit
                            };
                            match char::from_u32(scalar) {
                                Some(c) => out.push(c),
                                None => return Err("invalid \\u scalar".into()),
                            }
                        }
                        // only the RFC 8259 escapes are valid; anything
                        // else is a malformed document, not an identity
                        Some(c @ (b'"' | b'\\' | b'/')) => out.push(c as char),
                        Some(c) => {
                            return Err(format!("unknown escape '\\{}'", c as char));
                        }
                        None => return Err("unterminated escape".into()),
                    }
                    self.pos += 1;
                }
                Some(_) => {
                    // consume one UTF-8 scalar
                    let s = &self.bytes[self.pos..];
                    let ch_len = match s[0] {
                        0..=0x7f => 1,
                        0xc0..=0xdf => 2,
                        0xe0..=0xef => 3,
                        _ => 4,
                    };
                    out.push_str(
                        std::str::from_utf8(&s[..ch_len.min(s.len())])
                            .map_err(|_| "bad utf8".to_string())?,
                    );
                    self.pos += ch_len;
                }
                None => return Err("unterminated string".into()),
            }
        }
    }
    fn array(&mut self) -> Result<Json, String> {
        self.expect(b'[')?;
        let mut out = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(Json::Arr(out));
        }
        loop {
            out.push(self.value()?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b']') => {
                    self.pos += 1;
                    return Ok(Json::Arr(out));
                }
                _ => return Err(format!("bad array at byte {}", self.pos)),
            }
        }
    }
    fn object(&mut self) -> Result<Json, String> {
        self.expect(b'{')?;
        let mut out = HashMap::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(Json::Obj(out));
        }
        loop {
            self.skip_ws();
            let key = self.string()?;
            self.skip_ws();
            self.expect(b':')?;
            let val = self.value()?;
            out.insert(key, val);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(Json::Obj(out));
                }
                _ => return Err(format!("bad object at byte {}", self.pos)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_kicad_subset() {
        let v = parse_json(
            r#"{"unit": "MM", "n": -1.5e2, "ok": true, "list": [1, "two", null], "o": {}}"#,
        )
        .unwrap();
        assert_eq!(v.str_or("unit", ""), "MM");
        assert_eq!(v.num("n"), -150.0);
        assert_eq!(v.arr("list").len(), 3);
    }

    #[test]
    fn string_escapes_follow_rfc_8259() {
        // all simple escapes decode; \uXXXX surrogate PAIRS combine into
        // one scalar (each half decoded separately became '?'); unknown
        // escapes and lone surrogates are rejected, not passed through
        let v = parse_json(r#"{"s": "a\r\b\f\n\t\"\\\/z"}"#).unwrap();
        assert_eq!(
            v.str_or("s", ""),
            "a\r\u{8}\u{c}\n\t\"\\/z",
            "simple escapes"
        );
        let v = parse_json("{\"s\": \"\\uD83D\\uDE00\"}").unwrap();
        assert_eq!(v.str_or("s", ""), "😀", "surrogate pair combines");
        let v = parse_json("{\"s\": \"\\u0041\"}").unwrap();
        assert_eq!(v.str_or("s", ""), "A");
        assert!(parse_json(r#"{"s": "\q"}"#).is_err(), "unknown escape");
        assert!(
            parse_json(r#"{"s": "\uD83D x"}"#).is_err(),
            "lone high surrogate"
        );
        assert!(
            parse_json(r#"{"s": "\uDE00"}"#).is_err(),
            "lone low surrogate"
        );
    }
}
