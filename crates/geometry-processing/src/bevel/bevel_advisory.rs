//! One thing the kernel has to say - a refusal or a warning - as a code and the values it quotes,
//! for the application to put into words (`BevelAdvisoryText`).
//!
//! Every argument any code documents is an `int`, a `double`, a `Vector2`, a `Vector3` or a
//! `string`, so each one is a [`BevelAdvisoryArg`].

use std::fmt;
use std::hash::{Hash, Hasher};

use vector_math::{float_hash, Vector2, Vector3};

use super::bevel_advisory_code::BevelAdvisoryCode;

/// One value an advisory quotes, as one of the kinds the codes actually carry.
///
/// Equality is by kind first: an `Int` and a `Double` of the same number are NOT equal
/// (deliberate - the formatter's templates care which one they get), and two doubles compare by
/// value (NaN equals NaN, 0.0 equals -0.0).
#[derive(Debug, Clone)]
pub enum BevelAdvisoryArg {
    /// A count.
    Int(i32),
    /// A size, an angle, a distance.
    Double(f64),
    /// A point of a path.
    Vector2(Vector2),
    /// Where on the part.
    Vector3(Vector3),
    /// A name the sentence quotes.
    String(String),
}

impl PartialEq for BevelAdvisoryArg {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (BevelAdvisoryArg::Int(a), BevelAdvisoryArg::Int(b)) => a == b,
            (BevelAdvisoryArg::Double(a), BevelAdvisoryArg::Double(b)) => {
                a == b || (a.is_nan() && b.is_nan())
            }
            (BevelAdvisoryArg::Vector2(a), BevelAdvisoryArg::Vector2(b)) => a == b,
            (BevelAdvisoryArg::Vector3(a), BevelAdvisoryArg::Vector3(b)) => a == b,
            (BevelAdvisoryArg::String(a), BevelAdvisoryArg::String(b)) => a == b,
            _ => false,
        }
    }
}

impl Hash for BevelAdvisoryArg {
    fn hash<H: Hasher>(&self, state: &mut H) {
        std::mem::discriminant(self).hash(state);
        match self {
            BevelAdvisoryArg::Int(value) => value.hash(state),
            BevelAdvisoryArg::Double(value) => float_hash::hash_f64(*value, state),
            BevelAdvisoryArg::Vector2(value) => value.hash(state),
            BevelAdvisoryArg::Vector3(value) => value.hash(state),
            BevelAdvisoryArg::String(value) => value.hash(state),
        }
    }
}

impl fmt::Display for BevelAdvisoryArg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BevelAdvisoryArg::Int(value) => write!(f, "{value}"),
            BevelAdvisoryArg::Double(value) => write!(f, "{value}"),
            BevelAdvisoryArg::Vector2(value) => write!(f, "{value}"),
            BevelAdvisoryArg::Vector3(value) => write!(f, "{value}"),
            BevelAdvisoryArg::String(value) => write!(f, "{value}"),
        }
    }
}

impl From<i32> for BevelAdvisoryArg {
    fn from(value: i32) -> Self {
        BevelAdvisoryArg::Int(value)
    }
}

impl From<f64> for BevelAdvisoryArg {
    fn from(value: f64) -> Self {
        BevelAdvisoryArg::Double(value)
    }
}

impl From<Vector2> for BevelAdvisoryArg {
    fn from(value: Vector2) -> Self {
        BevelAdvisoryArg::Vector2(value)
    }
}

impl From<Vector3> for BevelAdvisoryArg {
    fn from(value: Vector3) -> Self {
        BevelAdvisoryArg::Vector3(value)
    }
}

impl From<String> for BevelAdvisoryArg {
    fn from(value: String) -> Self {
        BevelAdvisoryArg::String(value)
    }
}

impl From<&str> for BevelAdvisoryArg {
    fn from(value: &str) -> Self {
        BevelAdvisoryArg::String(value.to_string())
    }
}

/// A refusal or a warning: which sentence ([`BevelAdvisoryCode`]) and the values it quotes.
///
/// Value equality over the code and the arguments, element for element, so a test can compare
/// against `BevelAdvisory::new(code, args)`. [`BevelAdvisory::mesh_edge_index`] is deliberately no
/// part of equality, of the hash or of `Display`: it says where the sentence came from, not what
/// the sentence is.
#[derive(Debug, Clone)]
pub struct BevelAdvisory {
    code: BevelAdvisoryCode,
    args: Vec<BevelAdvisoryArg>,
    mesh_edge_index: i32,
}

impl BevelAdvisory {
    /// The value [`BevelAdvisory::mesh_edge_index`] takes where nothing named one edge.
    pub const NO_MESH_EDGE: i32 = -1;

    /// An advisory belonging to the pass rather than to one edge of it.
    pub fn new(code: BevelAdvisoryCode, args: Vec<BevelAdvisoryArg>) -> Self {
        BevelAdvisory {
            code,
            args,
            mesh_edge_index: Self::NO_MESH_EDGE,
        }
    }

    /// Which sentence this is.
    pub fn code(&self) -> BevelAdvisoryCode {
        self.code
    }

    /// The values the sentence quotes, in the order the code documents.
    pub fn args(&self) -> &[BevelAdvisoryArg] {
        &self.args
    }

    /// The mesh edge of the pick that NAMED this refusal, or [`BevelAdvisory::NO_MESH_EDGE`] where
    /// the refusal belongs to the pass rather than to one edge of it. What it is for is blame: a
    /// pass that plans several groups' chains together refuses ONCE, and without this the only
    /// honest thing it could do is write that one answer against every group in the pass.
    pub fn mesh_edge_index(&self) -> i32 {
        self.mesh_edge_index
    }

    /// The same advisory, told which mesh edge of the pick named it.
    pub fn on_mesh_edge(&self, mesh_edge_index: i32) -> BevelAdvisory {
        BevelAdvisory {
            code: self.code,
            args: self.args.clone(),
            mesh_edge_index,
        }
    }
}

impl PartialEq for BevelAdvisory {
    fn eq(&self, other: &Self) -> bool {
        self.code == other.code && self.args == other.args
    }
}

impl Hash for BevelAdvisory {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.code.hash(state);
        for arg in &self.args {
            arg.hash(state);
        }
    }
}

/// The code and its arguments, for a test failure to read - not a sentence for a user.
impl fmt::Display for BevelAdvisory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.args.is_empty() {
            return write!(f, "{}", self.code);
        }

        let args: Vec<String> = self.args.iter().map(|arg| arg.to_string()).collect();
        write!(f, "{}({})", self.code, args.join(", "))
    }
}
