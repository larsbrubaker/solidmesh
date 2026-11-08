# Third-party notices: solidmesh-math

solidmesh-math is BSD-2-Clause licensed (see `LICENSE`). Third-party code it ports or adapts is
listed below with its license, which requires these notices to stay with the source and with
binary redistributions; a dependency used as a crate (not copied) is listed for provenance. This
is the `solidmesh-math` section of the solidmesh repository's `THIRD_PARTY_NOTICES.md`, which the
workspace-gates `notices` test keeps identical to it.

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

