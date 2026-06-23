#ifndef XPBD_NARROWPHASE_HPP
#define XPBD_NARROWPHASE_HPP

// Narrowphase: the per-shape-pair geometry tests that turn an overlapping
// broadphase pair into a Contact (normal, penetration). Each cell lives in its
// own header so the dispatch matrix below grows one file at a time:
//
//   | typeA \ typeB | Sphere | Box | Capsule | Tetra |
//   |---------------|--------|-----|---------|-------|
//   | Sphere        | done   | --  | --      | --    |  sphere_sphere.hpp
//   | Box           |        | --  | --      | --    |
//   | Capsule       |        |     | --      | --    |
//   | Tetra         |        |     |         | --    |
//
// Convention (see DESIGN.md 5): keep the matrix lower-triangular by ordering
// the pair so typeA <= typeB and flipping the resulting normal if the caller
// swapped them. Convex-convex cells route through a shared GJK/EPA cell
// (gjk.hpp): one support-function-based implementation that covers every convex
// primitive (sphere / capsule / box / tetra / hull) via a core skeleton + a
// rounding radius. The contact is the canonical output of every cell;
// unimplemented cells return a non-touching Contact so partial shape support is
// always safe.

#include "xpbd/narrowphase/contact.hpp"
#include "xpbd/narrowphase/gjk.hpp"
#include "xpbd/narrowphase/sphere_sphere.hpp"

#endif  // XPBD_NARROWPHASE_HPP
