#ifndef XPBD_NARROWPHASE_CONTACT_HPP
#define XPBD_NARROWPHASE_CONTACT_HPP

#include "xpbd/math.hpp"

namespace xpbd {

// The canonical output of every narrowphase cell (sphere-sphere, sphere-box,
// GJK, ...). Geometry only: no entities, no bodies, no compliance. The contact
// solver turns this into a ContactConstraint by attaching the body references.
//
// `normal` is unit, pointing from shape A to shape B; pushing B by +normal and
// A by -normal separates them. `penetration` is the overlap depth (>= 0 when
// touching). `point` is the world-space contact point (the midpoint of the
// overlap region); particles ignore it, but a rigid body uses it as the lever
// arm for the angular part of the correction (see DESIGN.md 4.2).
struct Contact {
    Vec3 normal{};
    Vec3 point{};
    float penetration = 0.0f;
    bool touching = false;
};

}  // namespace xpbd

#endif  // XPBD_NARROWPHASE_CONTACT_HPP
