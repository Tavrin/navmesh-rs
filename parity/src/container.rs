//! Record files: the container the reference driver and the port both write.
//!
//! Layout (all integers little-endian):
//!
//! ```text
//! magic        8 bytes  "NAVREC" 0x00 0x01
//! meta_len     u32, then meta_len bytes of UTF-8 "key=value\n" lines
//! per record:  u32 name_len (> 0), name (UTF-8)
//!              u64 payload_len, payload
//!              u64 FNV-1a 64 of the payload
//! end:         u32 0, then u32 record count
//! ```

use std::collections::HashMap;
use std::fmt;
use std::fs::File;
use std::io::{self, BufReader, Cursor, Read, Seek, SeekFrom};
use std::path::Path;

/// File magic: `NAVREC`, a zero byte and the format version (1).
pub const MAGIC: [u8; 8] = *b"NAVREC\x00\x01";

/// Longest record name accepted when reading.
const MAX_NAME_LEN: u32 = 4096;
/// Largest metadata block accepted when reading.
const MAX_META_LEN: u32 = 1 << 20;

/// FNV-1a 64-bit digest, as the reference driver computes it.
pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

/// An I/O or format error while reading or comparing record files.
#[derive(Debug)]
pub struct Error(pub String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error(e.to_string())
    }
}

/// One record's framing: its name and where its payload lies in the file.
#[derive(Clone, Debug)]
pub struct Entry {
    /// Record name, such as `poly_mesh` or `tile_1_2.contours`.
    pub name: String,
    /// Byte offset of the payload in the file.
    pub offset: u64,
    /// Payload length in bytes.
    pub len: u64,
    /// FNV-1a 64 digest stored after the payload.
    pub digest: u64,
}

trait Source: Read + Seek {}
impl<T: Read + Seek> Source for T {}

/// A record file opened for reading. Opening scans the framing only; payloads
/// are read on demand, so files much larger than memory can be compared.
pub struct RecordFile {
    label: String,
    source: Box<dyn Source>,
    /// Metadata lines in file order.
    pub meta: Vec<(String, String)>,
    /// Records in file order.
    pub entries: Vec<Entry>,
    index: HashMap<String, usize>,
}

impl fmt::Debug for RecordFile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RecordFile")
            .field("label", &self.label)
            .field("records", &self.entries.len())
            .finish()
    }
}

fn read_u32(r: &mut impl Read) -> io::Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

fn read_u64(r: &mut impl Read) -> io::Result<u64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}

impl RecordFile {
    /// Opens a record file from disk.
    pub fn open(path: &Path) -> Result<Self, Error> {
        let file = File::open(path).map_err(|e| Error(format!("{}: {e}", path.display())))?;
        Self::from_source(path.display().to_string(), Box::new(BufReader::new(file)))
            .map_err(|e| Error(format!("{}: {}", path.display(), e.0)))
    }

    /// Reads a record file held in memory (`label` names it in messages).
    pub fn from_bytes(label: &str, bytes: Vec<u8>) -> Result<Self, Error> {
        Self::from_source(label.to_string(), Box::new(Cursor::new(bytes)))
    }

    fn from_source(label: String, mut source: Box<dyn Source>) -> Result<Self, Error> {
        let total = source.seek(SeekFrom::End(0))?;
        source.seek(SeekFrom::Start(0))?;
        let mut magic = [0u8; 8];
        source
            .read_exact(&mut magic)
            .map_err(|_| Error("not a record file (too short)".into()))?;
        if magic != MAGIC {
            return Err(Error("not a record file (bad magic or version)".into()));
        }
        let meta_len = read_u32(&mut source)?;
        if meta_len > MAX_META_LEN {
            return Err(Error(format!(
                "metadata block of {meta_len} bytes is too large"
            )));
        }
        let mut meta_bytes = vec![0u8; meta_len as usize];
        source.read_exact(&mut meta_bytes)?;
        let meta_text =
            String::from_utf8(meta_bytes).map_err(|_| Error("metadata is not UTF-8".into()))?;
        let meta = meta_text
            .lines()
            .filter(|l| !l.is_empty())
            .map(|l| match l.split_once('=') {
                Some((k, v)) => (k.to_string(), v.to_string()),
                None => (l.to_string(), String::new()),
            })
            .collect();

        let mut entries = Vec::new();
        let mut index = HashMap::new();
        let mut pos = 8 + 4 + u64::from(meta_len);
        loop {
            let name_len = read_u32(&mut source)
                .map_err(|_| Error(format!("truncated at byte {pos}: missing end marker")))?;
            pos += 4;
            if name_len == 0 {
                let count = read_u32(&mut source)
                    .map_err(|_| Error("truncated: missing record count".into()))?;
                pos += 4;
                if count as usize != entries.len() {
                    return Err(Error(format!(
                        "end marker says {count} records, file holds {}",
                        entries.len()
                    )));
                }
                if pos != total {
                    return Err(Error(format!(
                        "{} trailing bytes after the end marker",
                        total - pos
                    )));
                }
                break;
            }
            if name_len > MAX_NAME_LEN {
                return Err(Error(format!(
                    "record name of {name_len} bytes at byte {pos}"
                )));
            }
            let mut name = vec![0u8; name_len as usize];
            source
                .read_exact(&mut name)
                .map_err(|_| Error(format!("truncated record name at byte {pos}")))?;
            let name = String::from_utf8(name)
                .map_err(|_| Error(format!("record name at byte {pos} is not UTF-8")))?;
            pos += u64::from(name_len);
            let len =
                read_u64(&mut source).map_err(|_| Error(format!("truncated record '{name}'")))?;
            pos += 8;
            let offset = pos;
            if len > total.saturating_sub(pos) {
                return Err(Error(format!(
                    "record '{name}' claims {len} bytes past the end of the file"
                )));
            }
            source.seek(SeekFrom::Current(len as i64))?;
            pos += len;
            let digest =
                read_u64(&mut source).map_err(|_| Error(format!("truncated record '{name}'")))?;
            pos += 8;
            if index.insert(name.clone(), entries.len()).is_some() {
                return Err(Error(format!("record '{name}' appears twice")));
            }
            entries.push(Entry {
                name,
                offset,
                len,
                digest,
            });
        }
        Ok(RecordFile {
            label,
            source,
            meta,
            entries,
            index,
        })
    }

    /// The name this file was opened under (its path, or the in-memory label).
    pub fn label(&self) -> &str {
        &self.label
    }

    /// The value of a metadata key, if present.
    pub fn meta_value(&self, key: &str) -> Option<&str> {
        self.meta
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    /// Index of the record with this name.
    pub fn find(&self, name: &str) -> Option<usize> {
        self.index.get(name).copied()
    }

    /// Reads a record's payload and checks it against the stored digest.
    pub fn payload(&mut self, index: usize) -> Result<Vec<u8>, Error> {
        let entry = &self.entries[index];
        let len = usize::try_from(entry.len).map_err(|_| {
            Error(format!(
                "record '{}' is too large for this platform",
                entry.name
            ))
        })?;
        let mut buf = vec![0u8; len];
        self.source.seek(SeekFrom::Start(entry.offset))?;
        self.source.read_exact(&mut buf)?;
        let digest = fnv1a64(&buf);
        if digest != entry.digest {
            return Err(Error(format!(
                "record '{}': payload digest {digest:016x} does not match the stored {:016x}",
                entry.name, entry.digest
            )));
        }
        Ok(buf)
    }
}
