#ifndef XPBD_PARTICLE_HPP
#define XPBD_PARTICLE_HPP

#include "xpbd/math.hpp"

namespace xpbd {

struct Particle {
    Vec3 position;
    Vec3 previousPosition;
    Vec3 velocity;
    Vec3 externalAcceleration;
    float inverseMass = 1.0f;
    float radius = 0.025f;
};

}  // namespace xpbd

#endif  // XPBD_PARTICLE_HPP
