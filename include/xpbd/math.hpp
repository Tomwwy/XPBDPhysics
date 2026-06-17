#ifndef XPBD_MATH_HPP
#define XPBD_MATH_HPP

#include "utils/vec3.hpp"

namespace xpbd {

using Vec3 = utils::Vec3;

using utils::cross;
using utils::distance;
using utils::distanceSq;
using utils::distanceSquared;
using utils::dot;
using utils::length;
using utils::lengthSq;
using utils::lengthSquared;
using utils::normalized;
using utils::operator*;

}  // namespace xpbd

#endif  // XPBD_MATH_HPP
