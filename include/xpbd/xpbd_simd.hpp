#ifndef XPBD_SIMD_HPP
#define XPBD_SIMD_HPP

#include "xpbd/math.hpp"

#include <cstddef>
#include <cstdint>

namespace xpbd {

struct Particle;

enum class SimdBackend {
    Scalar,
    Sse2,
};

enum class SimdPreference {
    Auto,
    Scalar,
    Sse2,
};

struct SimdInfo {
    SimdBackend backend = SimdBackend::Scalar;
    const char* backendName = "scalar";
    bool sse2Supported = false;
};

void initializeSimd(SimdPreference preference = SimdPreference::Auto);
const SimdInfo& simdInfo();
const char* simdBackendName();

namespace detail {

using IntegrateParticlesFn = void (*)(Particle* particles,
                                      const std::uint8_t* alive,
                                      std::size_t count,
                                      const Vec3& gravity,
                                      float damping,
                                      float dt);

using UpdateParticleVelocitiesFn = void (*)(Particle* particles,
                                            const std::uint8_t* alive,
                                            std::size_t count,
                                            float dt);

struct SimdDispatch {
    SimdBackend backend = SimdBackend::Scalar;
    const char* name = "scalar";
    IntegrateParticlesFn integrateParticles = nullptr;
    UpdateParticleVelocitiesFn updateParticleVelocities = nullptr;
};

const SimdDispatch& simdDispatch();

}  // namespace detail

}  // namespace xpbd

#endif  // XPBD_SIMD_HPP
