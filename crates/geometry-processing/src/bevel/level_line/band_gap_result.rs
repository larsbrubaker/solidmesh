//! What the band's removal leaves ([`BandGapResult`]): the mesh with a gap in it, the field it was
//! cut to, and the gap's boundaries. Ports the type of `Bevel/LevelLine/BandGapResult.cs`; its
//! derived readings (`built`, `chain`) land with the band-gap method package.

use std::rc::Rc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::{
    BandGapRefusal, ChainField, ChainSideLoops, GapLoop, LevelLineMesh, LevelLineWorkPhase,
    RoomSource,
};

/// C# `BandGapResult`: what the band's removal leaves - the mesh with a gap in it, the field it was
/// cut to, and the gap's boundaries. The `-1` and `RoomSource.Nothing` defaults are C#'s
/// initializers, which is why `Default` is written out rather than derived.
#[derive(Debug, Clone)]
pub struct BandGapResult {
    /// Why the build refused, or [`BandGapRefusal::None`] when the band came out.
    pub refusal: BandGapRefusal,
    /// The largest size that BUILDS: the geometric room - the size whose band exactly reaches the
    /// foreign feature bounding the picked sides, or exactly swallows a side - less the margin a
    /// kept strip needs to have width at all. The same number whatever size was asked for, and
    /// positive infinity when the sides carry no foreign feature and cannot run out of themselves.
    /// Reported whether the build succeeded or refused, and a refusal's room is therefore always a
    /// size that builds.
    ///
    /// A PINCH refusal names something weaker, and deliberately: a size KNOWN to fit rather than
    /// the largest. The pinch room is found by probing - halving down from the size that pinched to
    /// one that leaves each side its loop, then three bisections - and the number named is the
    /// largest size a probe actually BUILT at, so "ask exactly the room named and it builds" holds
    /// by construction. The true limit may stand up to about an eighth (12.5%) higher.
    pub room: f64,
    /// The working mesh, cut and with the band's triangles removed. A removed triangle keeps its
    /// place in the triangle list, so the kept mesh and the band together are still exactly the
    /// solid that went in. A refusal raised once the working mesh exists carries it too -
    /// WorkBudgetExceeded included, where it is the mesh as it stood when the work ran out - so a
    /// refusal can be read for what the build did. It is a result only when the build succeeded:
    /// decide by that, never by whether this is `None`.
    pub mesh: Option<Rc<LevelLineMesh>>,
    /// The pick's chains as fields, in the order they were given: the setbacks, the parameters and
    /// the distances the gap was cut to. A pick of one chain holds exactly one.
    pub chains: Vec<Rc<ChainField>>,
    /// Which of the room's readings named it - see [`RoomSource`]. Kind `None` where the room was
    /// handed in through knownRoom, or where nothing bounded the pick.
    pub room_set_by: RoomSource,
    /// Set only on a band that FLOWED over an end face narrower than the setback: the room the pick
    /// had without the flow, which is the size refusal it answers with if the flowed band does not
    /// make a part. The flow is an improvement only when it builds.
    pub unflowed_room: Option<f64>,
    /// Both sides' runs made up off the mesh when the strip swallows both sides whole, side 1
    /// first, each as (point, parameter) in ascending chain parameter; `None` otherwise.
    pub swallowed_runs: Option<Vec<Vec<(Vector3, f64)>>>,
    /// The gap's boundaries - one run per side, more if a side's boundary comes apart.
    pub loops: Vec<GapLoop>,
    /// The group tags of a pick of several chains that are a free end's END REGION: their loops
    /// are boundary round that end, not a rail, so the corner fill leaves them to the cap.
    pub end_region_tags: Vec<i32>,
    /// For each side group's tag, the chains that are its members only through a free end
    /// standing on it: a loop such a chain owns there is the cut round that end, not a rail, so
    /// the corner fill leaves it to the cap as it does an end region's run.
    pub end_members_by_tag: IndexMap<i32, Vec<i32>>,
    /// For each side group's tag, the end members whose end there is GROWN OVER: a concave chain's
    /// oblique free end standing on the group's face. The cut round such an end is closed by the
    /// round's section in the end plane, not fanned from the end vertex.
    pub grow_over_by_tag: IndexMap<i32, Vec<i32>>,
    /// For each side group's tag, the chains CARRIED onto it past a rim step corner: the stretch
    /// such a chain owns there is its round's reach onto that face, the corner hole's rim, not a
    /// rail.
    pub carried_by_tag: IndexMap<i32, Vec<i32>>,
    /// What the cut left every picked (chain, side) - one row per picked side, filled wherever the
    /// loops were walked at all. A refusal for not leaving each side its one owned loop carries
    /// here WHICH sides failed and who they ran into.
    pub side_loops: Vec<ChainSideLoops>,
    /// The triangles of the band that was removed.
    pub removed_triangles: Vec<i32>,
    /// The triangle that would not settle, or -1.
    pub offending_triangle: i32,
    /// The chain edge that does not stand on exactly two triangles, or (-1, -1).
    pub offending_edge: (i32, i32),
    /// Which chain of the pick a refusal is about, or -1 when it is about none of them in
    /// particular. A single-chain pick is the list of one, so its chain refusals name chain 0.
    pub offending_chain: i32,
    /// The other chain of a chains-touch refusal, or -1.
    pub other_chain: i32,
    /// The vertex two touching chains share, or -1.
    pub offending_vertex: i32,
    /// How many band builds the pinch ladder ran to name this room - zero on every build and on
    /// every refusal that is not a pinch. Bounded by nine: at most six halvings and three
    /// bisections. A WorkBudgetExceeded carrying a count above zero is a ladder that ran out of
    /// work rather than a build that did.
    pub pinch_probes: i32,
    /// True when the ladder found no size at all that leaves each side one closed loop: every size
    /// it tried pinched, so the room is ZERO and nothing this pick is asked for will build. A
    /// search that ran out of work before it could answer sets this FALSE and leaves by
    /// WorkBudgetExceeded, because it looked at nothing.
    pub pinches_at_every_size_tried: bool,
    /// What was running when the work budget ran out; [`LevelLineWorkPhase::None`] otherwise.
    pub phase: LevelLineWorkPhase,
    /// How many units of work the build spent.
    pub work_spent: i64,
    /// How many units it was allowed.
    pub work_limit: i64,
}

impl Default for BandGapResult {
    fn default() -> Self {
        BandGapResult {
            refusal: BandGapRefusal::default(),
            room: 0.0,
            mesh: None,
            chains: Vec::new(),
            room_set_by: RoomSource::nothing(),
            unflowed_room: None,
            swallowed_runs: None,
            loops: Vec::new(),
            end_region_tags: Vec::new(),
            end_members_by_tag: IndexMap::new(),
            grow_over_by_tag: IndexMap::new(),
            carried_by_tag: IndexMap::new(),
            side_loops: Vec::new(),
            removed_triangles: Vec::new(),
            offending_triangle: -1,
            offending_edge: (-1, -1),
            offending_chain: -1,
            other_chain: -1,
            offending_vertex: -1,
            pinch_probes: 0,
            pinches_at_every_size_tried: false,
            phase: LevelLineWorkPhase::default(),
            work_spent: 0,
            work_limit: 0,
        }
    }
}
