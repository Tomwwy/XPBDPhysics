#ifndef XPBD_DISTANCE_CONSTRAINT_HPP
#define XPBD_DISTANCE_CONSTRAINT_HPP

#include "xpbd/entity.hpp"

namespace xpbd {

struct DistanceConstraint {
    Entity particleA;
    Entity particleB;
    float restLength = 0.0f;
    float compliance = 0.0f;
    float lambda = 0.0f;
    bool enabled = true;
};

}  // namespace xpbd

#endif  // XPBD_DISTANCE_CONSTRAINT_HPP
