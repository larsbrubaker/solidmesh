# Third-party notices

solidmesh is BSD-2-Clause licensed (see `LICENSE`). Third-party code it ports or adapts is listed
below with its license, which requires these notices to stay with the source and with binary
redistributions; a dependency used as a crate (not copied) is listed for provenance. Each section
names the crate that carries it, at the folder the crate has here. Add a section in the same
change that ports or adapts anything from a new upstream (CLAUDE.md, "Never").

Each crate also ships its own section, in `crates/<folder>/THIRD_PARTY_NOTICES.md`, with a copy of
`LICENSE` beside it, because a published crate carries only its own folder. This file is the one
to edit; copy the changed section into the crate's file in the same change (the workspace-gates
`notices` test refuses a crate file that differs from its section here).

# solidmesh-math (`crates/vector-math`)

## The Open Toolkit library (MIT/X11)

Carried by: solidmesh-math (`crates/vector-math`)

Source: the vectors, matrices, quaternions and math helpers of MatterHackers' agg-sharp library
(BSD-2), which derive from OpenTK. Adapted in `crates/vector-math`.

```
Copyright (c) 2006 - 2008 The Open Toolkit library.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## ChevyRay BVH (MIT)

Carried by: solidmesh-math (`crates/vector-math`)

Source: the bounding volume hierarchy of MatterHackers' agg-sharp library (BSD-2): the tree, its
trade-off and locally ordered clustering builders and its ray hit types, which derive from
ChevyRay's BVH. Adapted in `crates/vector-math/src/bvh`.

```
The MIT License(MIT)
Copyright(c) 2015 ChevyRay

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## tween.js (MIT)

Carried by: solidmesh-math (`crates/vector-math`)

Source: the easing curves of MatterHackers' agg-sharp library (BSD-2), which are taken from
tween.js (https://github.com/sole/tween.js, MIT, Copyright (c) 2010-2012 Tween.js authors). Adapted in
`crates/vector-math/src/easing.rs`. The MIT terms are as in the sections above.

## .NET Runtime (MIT)

Carried by: solidmesh-math (`crates/vector-math`)

Source: `ArraySortHelper<T>.IntrospectiveSort` and the seeded `System.Random` from
https://github.com/dotnet/runtime (MIT, Copyright (c) .NET Foundation and Contributors). The
introspective sort is adapted in `crates/vector-math/src/fixed_tie_sort.rs`, so sorts (including Clipper
6's intersection list) place equal elements in the order saved designs and their rebuilt results
depend on; the seeded random generator (`CompatPrng`) is adapted in
`crates/vector-math/src/seeded_random.rs`, so seeded results repeat exactly. The MIT terms are as
in the sections above.

# solidmesh-clipper (`crates/clipper-library`)

## clipper2-rust (BSL-1.0) and Clipper 6.4.2 (BSL-1.0)

Carried by: solidmesh-clipper (`crates/clipper-library`); solidmesh-mesh (`crates/polygon-mesh`)
also depends on clipper2-rust

Source: https://github.com/larsbrubaker/clipper2-rust — the pure-Rust port of Clipper2
(https://github.com/AngusJohnson/Clipper2, Boost Software License 1.0, Copyright Angus Johnson),
used as a dependency (not copied) by `crates/clipper-library` and `crates/polygon-mesh` for its
integer point and path types (`Point64`, `Path64`, `Paths64`, `FillRule`, `ClipType`,
`JoinType`, `PolyTree64`), which clipper-library's helpers speak. Its license text is in that
repository's `LICENSE`.

`crates/clipper-library/src/clipper6/` is adapted from the Clipper Library 6.4.2 (Author Angus
Johnson, Copyright Angus Johnson 2010-2017, http://www.angusj.com) as carried in MatterHackers'
agg-sharp library (BSD-2): the Vatti clipper, `ClipperOffset`, `SimplifyPolygon(s)`, Minkowski
and the static helpers. `crates/clipper-library/src/clean_polygon.rs` adapts the modified
`Clipper.CleanPolygon(s)` that copy carries. The Clipper license:

```
The Clipper Library (including Delphi, C++ & C# source code, other accompanying
code, examples and documentation), hereafter called "the Software", has been 
released under the following license, terms and conditions:

Boost Software License - Version 1.0 - August 17th, 2003
http://www.boost.org/LICENSE_1_0.txt

Permission is hereby granted, free of charge, to any person or organization 
obtaining a copy of the Software covered by this license to use, reproduce, 
display, distribute, execute, and transmit the Software, and to prepare 
derivative works of the Software, and to permit third-parties to whom the 
Software is furnished to do so, all subject to the following:

The copyright notices in the Software and this entire statement, including the 
above license grant, this restriction and the following disclaimer, must be 
included in all copies of the Software, in whole or in part, and all derivative 
works of the Software, unless such copies or derivative works are solely in the 
form of machine-executable object code generated by a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL 
THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY 
DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING 
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
THE SOFTWARE.
```

# solidmesh-mesh (`crates/polygon-mesh`)

## MIConvexHull (MIT)

Carried by: solidmesh-mesh (`crates/polygon-mesh`)

Source: https://github.com/DesignEngrLab/MIConvexHull at d5f3a77 (the code the
`MIConvexHull 1.1.19.1019` package was built from). The 3D convex hull path is adapted in `crates/polygon-mesh/src/processors/mi_convex_hull/`.

The MIT License (MIT)

Copyright (c) 2010 David Sehnal, Matthew Campbell

The MIT terms are as in the sections above.

## geometry3Sharp marching cubes tables and signed distance grid (BSL-1.0)

Carried by: solidmesh-mesh (`crates/polygon-mesh`)

`crates/polygon-mesh/src/sdf/marching_cubes_tables.rs` copies the `edge_indices`, `edgeTable`
and `triTable` tables of geometry3Sharp's `mesh_generators/MarchingCubes.cs`
(https://github.com/gradientspace/geometry3Sharp, Copyright Ryan Schmidt / gradientspace, Boost
Software License 1.0, as carried in MatterHackers' agg-sharp library), which are Paul Bourke's
public marching cubes tables. The Boost terms are as in the clipper2-rust section above.

`crates/polygon-mesh/src/g3/` adapts the parts of geometry3Sharp that `MeshSdfGrid` and the mesh
repair reach: `DMesh3` (`mesh/DMesh3.cs`, `mesh/DMesh3_edge_operators.cs`: storage, appending,
the topology queries, and the remove/reverse/split/flip/collapse/merge/poke edits; `g3/dmesh3*.rs`)
with the `IndexUtil` helpers it uses (`math/IndexUtil.cs`), `RefCountVector`
(`core/RefCountVector.cs`) and `SmallListSet`'s list order (`core/SmallListSet.cs`),
`DMesh3.SetTriangle` / `GetOrientedBoundaryEdgeV` / `VtxAllBoundaryEdges`, `MeshBoundaryLoops`
(`mesh_selection/MeshBoundaryLoops.cs`), `EdgeLoop` / `EdgeSpan` (subset, `mesh/EdgeLoop.cs`,
`mesh/EdgeSpan.cs`), `MeshValidation.IsBoundaryLoop` (`queries/MeshValidation.cs`), `MeshEditor`
(the triangle removal, fan and bowtie subset, `mesh/MeshEditor.cs`), `MeshFaceSelection`
(add-only subset, `mesh_selection/MeshFaceSelection.cs`), `MeshIterators`
(`mesh/MeshIterators.cs`), `MathUtil.PlaneAngleSignedD` (`math/MathUtil.cs`), `SimpleHoleFiller`
(`mesh_ops/SimpleHoleFiller.cs`), `MeshRepairOrientation` (`mesh_ops/MeshRepairOrientation.cs`), the ray
queries `Ray3d`, `IntrRay3Triangle3.Intersects` and `IntrRay3AxisAlignedBox3.FindRayIntersectT`
(`intersection/`), `DMeshAABBTree3.FindNearestHitTriangle`, `MergeCoincidentEdges`
(`mesh_ops/MergeCoincidentEdges.cs`) with `PointSetHashtable` (`spatial/PointSetHashtable.cs`),
`ShiftGridIndexer3` (`spatial/GridIndexing.cs`), `MeshBoundaryEdgeMidpoints` (`mesh/MeshPointSets.cs`)
and `DynamicPriorityQueue` (`core/DynamicPriorityQueue.cs`),
`DMeshAABBTree3` (top-down midpoint build and
nearest-triangle query, `spatial/DMeshAABBTree.cs`), the point-triangle distance
(`queries/MeshQueries.cs`, `distance/DistPoint3Triangle3.cs`) and `MeshSignedDistanceGrid`'s
narrow-band flood and `point_in_triangle_2d` (`spatial/MeshSignedDistanceGrid.cs`, itself after
Christopher Batty's public-domain SDFGen). Same copyright and Boost terms as above.

For the implicit booleans (`csg::implicit_boolean`) it also adapts `MeshSignedDistanceGrid`'s
default signed narrow-band mode (`make_level_set3_parallel`, `compute_intersections`,
`compute_signs`, `point_triangle_distance`, `point_segment_distance`;
`g3/signed_distance_grid.rs`), `DenseGridTrilinearImplicit` (`implicit/GridImplicits3d.cs`), the
`BoundedImplicitFunction3d` interface and the `ImplicitUnion3d` / `ImplicitIntersection3d` /
`ImplicitDifference3d` / `ImplicitNaryUnion3d` / `ImplicitNaryIntersection3d` /
`ImplicitNaryDifference3d` operators (`implicit/Implicit3d.cs`; `g3/implicit3d.rs`),
`MarchingCubes` (`mesh_generators/MarchingCubes.cs`; `g3/marching_cubes.rs`),
`MarchingCubesPro` (`mesh_generators/MarchingCubesPro.cs`; `g3/marching_cubes_pro.rs`) and
`AxisAlignedBox3d`'s `Width`/`Height`/`Depth`/`MaxDim`/`Expand`/`Contain`/`Intersect`
(`math/AxisAlignedBox3d.cs`; `g3/dmesh3.rs`). These source files carry no header of their own; they
are geometry3Sharp's (`MarchingCubesPro.cs` names Ryan Schmidt and the Boost license). Same
copyright and Boost terms as above.

For the mesh repair's hole filling it also adapts the remesher `MinimalHoleFill` runs:
`MeshConstraints` and `MeshConstraintUtil.FixAllBoundaryEdges` (`mesh/MeshConstraints.cs`,
`mesh/MeshConstraintUtil.cs`) with the `IProjectionTarget` interface
(`spatial/SpatialInterfaces.cs`; `g3/mesh_constraints.rs`), `MeshRefinerBase`
(`mesh/MeshRefinerBase.cs`; `g3/mesh_refiner_base.rs`), `MeshUtil`'s uniform smoother,
Gaussian curvature and edge flip/collapse checks with `DMesh3.GetTriInternalAngleR` and
`VtxOneRingCentroid` (`mesh/MeshUtil.cs`, `mesh/DMesh3.cs`; `g3/mesh_util.rs`), `Remesher`
(`mesh/Remesher.cs`; `g3/remesher.rs`), `RemesherPro` (`mesh/RemesherPro.cs`;
`g3/remesher_pro.rs`), and the `Vector3d`/`MathUtil` arithmetic they use (`math/Vector3d.cs`,
`math/MathUtil.cs`; `g3/vector3d.rs`). `MeshConstraints.cs`, `MeshConstraintUtil.cs`,
`MeshRefinerBase.cs`, `MeshUtil.cs` and `Remesher.cs` carry no header; `RemesherPro.cs` names
Ryan Schmidt and the Boost license. Same copyright and Boost terms as above.

`MinimalHoleFill` edits its fill as a submesh, so it also adapts `DSubmesh3` (compute, boundary
info and vertex maps, `mesh/DSubmesh3.cs`; `g3/dsubmesh3.rs`), `RegionOperator`
(`mesh_ops/RegionOperator.cs`; `g3/region_operator.rs`), `MeshEditor.ReinsertSubmesh` and the
`IList` `RemoveTriangles` overload (`mesh/MeshEditor.cs`; `g3/mesh_editor.rs`),
`MeshQueries.EdgeLengthStats` / `EdgeLengthStatsFromEdges` (`queries/MeshQueries.cs`;
`g3/mesh_queries.rs`) and `MathUtil.Area` / `AspectRatio` (`g3/vector3d.rs`). These files carry
no header of their own; they are geometry3Sharp's. Same copyright and Boost terms as above.

`MinimalHoleFill` itself (`mesh_ops/MinimalHoleFill.cs`; `g3/minimal_hole_fill.rs`) names Ryan
Schmidt and the Boost license. Same copyright and Boost terms as above.

`MeshAutoRepair` (`mesh_ops/MeshAutoRepair.cs`, with `RemoveMode = None`; `g3/mesh_auto_repair.rs`)
and `MathUtil.ModuloIteration` (`math/MathUtil.cs`) drive them: the Repair tool's default pass.
`MeshAutoRepair.cs` names Ryan Schmidt and the Boost license. Same copyright and Boost terms as
above.

## Dual contouring QEF solver (public domain)

Carried by: solidmesh-mesh (`crates/polygon-mesh`)

`crates/polygon-mesh/src/dual_contouring/qef.rs` is adapted from the QEF solver carried in
MatterHackers' agg-sharp library: its `QefData`, `QefSolver`, `SVD`, `Givens` and `Schur2` files,
which carry the Unlicense dedication ("This is free and unencumbered software released into the
public domain", http://unlicense.org/), and the matrix helpers `Mat3`, `SMat3` and
`MatUtils` from the same QEF library, which carry no header. No notice is required; it is
recorded here for provenance.

The dual contouring octree itself (`dual_contouring/octree.rs`, `octree_contour.rs`) is not
adapted from any existing code: the octree agg-sharp carries is LGPL 2.1 (Tao Ju) and was not
used. It is written from the
published algorithm (Ju, Losasso, Schaefer, Warren, "Dual Contouring of Hermite Data", SIGGRAPH
2002).

## manifold-rust (Apache-2.0)

Carried by: solidmesh-mesh (`crates/polygon-mesh`)

Source: https://github.com/larsbrubaker/manifold-rust — the pure-Rust port of the Manifold
geometry library (https://github.com/elalish/manifold, Apache-2.0), used as a dependency (not
copied) by `crates/polygon-mesh` for booleans. Its license text is in that repository's
`LICENSE`. Its own dependencies (dashu, clipper2-rust, num-traits, rustc-hash, rayon) are
MIT/Apache-2.0/BSL-1.0 licensed.

## tess2-rust (SGI Free Software License B 2.0, MIT-style)

Carried by: solidmesh-mesh (`crates/polygon-mesh`)

Source: https://github.com/larsbrubaker/tess2-rust — the pure-Rust port of libtess2
(https://github.com/memononen/libtess2, itself the SGI GLU tessellator under SGI Free Software
License B 2.0, an MIT-style license), used as a dependency (not copied) by `crates/polygon-mesh`
for `CachedTesselator` (`data_converters_2d`). Its license text is in that repository.

## naga (MIT OR Apache-2.0, test-only)

Carried by: solidmesh-mesh (`crates/polygon-mesh`), as a dev-dependency only

Source: https://github.com/gfx-rs/wgpu (its `naga` crate), used as a crate (not copied) by the
tests alone, which parse and validate the flat tree's WGSL traversal: it does not reach the
library. Its license text is in that repository.

# solidmesh (`crates/geometry-processing`)

## indexmap (MIT OR Apache-2.0)

Carried by: solidmesh (`crates/geometry-processing`)

Source: https://github.com/indexmap-rs/indexmap — a crates.io dependency (not copied) of
`crates/geometry-processing`, which iterates maps and sets in insertion order with it where that
order reaches a result (CLAUDE.md, "Exactness").

## libm (MIT)

Carried by: solidmesh (`crates/geometry-processing`)

Source: https://github.com/rust-lang/compiler-builtins (its `libm` crate) — a crates.io dependency
(not copied) of `crates/geometry-processing`, whose 2D path bevel and topology readings take
`sin`, `cos`, `tan`, `acos` and `atan2` from it in pure Rust, so a design gives the same curves on
every platform. Its license text is in that repository.

## libc, windows-sys, sha2 and zip (test-only)

Carried by: solidmesh (`crates/geometry-processing`), as dev-dependencies only

Sources: https://github.com/rust-lang/libc and https://github.com/microsoft/windows-rs (MIT OR
Apache-2.0), read by the bevel cost tests for the calling thread's CPU time (`clock_gettime` on
Unix, `GetThreadTimes` on Windows); https://github.com/RustCrypto/hashes (`sha2`, MIT OR
Apache-2.0), the cost tests' mesh digests; https://github.com/zip-rs/zip2 (`zip`, MIT), which the
Thingi10K sweep reads the corpus' meshes out of their zip files with. Used as crates (not copied),
by the tests alone: none of them reaches the library. License texts are in each crate's
repository.
