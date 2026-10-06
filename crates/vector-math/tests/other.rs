// Ported tests from agg-sharp `Tests/Agg.Tests/Other` that exercise vector-math (CLAUDE.md
// "Testing framework", net 1), one file per C# test class. These `Vector2Tests` and
// `Vector3Tests` are the `MatterHackers.Agg.Tests` classes, distinct from the same-named
// `Agg.VectorMath` classes in the `vector_math` area.

#[path = "other/vector2_tests.rs"]
mod vector2_tests;
#[path = "other/vector3_tests.rs"]
mod vector3_tests;
