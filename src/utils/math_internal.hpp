#ifndef UTILS_MATH_INTERNAL_HPP
#define UTILS_MATH_INTERNAL_HPP

#include "utils/vec3.hpp"

namespace utils::detail {

const MathOps& scalarMathOps();

#if defined(UTILS_HAS_X86_SIMD)
const MathOps& sse2MathOps();
const MathOps& avxMathOps();
const MathOps& avx2MathOps();
#endif

}  // namespace utils::detail

#endif  // UTILS_MATH_INTERNAL_HPP
