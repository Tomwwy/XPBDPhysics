#ifndef XPBD_PARTICLE_HPP
#define XPBD_PARTICLE_HPP

#include "xpbd/math.hpp"

namespace xpbd {

// A pure point degree-of-freedom integrated by XPBD. Collision geometry and
// filtering live on a separate Collider that references this particle, so the
// same particle can carry any shape (or none) without changing this struct.
struct Particle {
    Vec3 position;
    Vec3 previousPosition;
    Vec3 velocity;
    Vec3 externalAcceleration;
    float inverseMass = 1.0f;
};

}  // namespace xpbd

#endif  // XPBD_PARTICLE_HPP
