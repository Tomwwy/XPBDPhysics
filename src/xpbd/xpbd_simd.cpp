#include "xpbd/xpbd_simd.hpp"

#include "xpbd/xpbd_world.hpp"

#include <atomic>
#include <mutex>

#if defined(_M_IX86) || defined(_M_X64) || defined(__i386__) || defined(__x86_64__)
#define XPBD_CPU_X86 1
#else
#define XPBD_CPU_X86 0
#endif

#if XPBD_CPU_X86
#include <emmintrin.h>
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <cpuid.h>
#endif
#endif

namespace xpbd {
namespace {

void integrateOne(Particle& particle, const Vec3& gravity, float damping, float dt)
{
    if (particle.inverseMass <= 0.0f) {
        particle.previousPosition = particle.position;
        particle.velocity = {};
        particle.externalAcceleration = {};
        return;
    }

    const float dtSq = dt * dt;
    const Vec3 current = particle.position;
    const Vec3 velocityStep = (particle.position - particle.previousPosition) * damping;
    const Vec3 acceleration = gravity + particle.externalAcceleration;
    particle.position += velocityStep + acceleration * dtSq;
    particle.previousPosition = current;
    particle.velocity = (particle.position - particle.previousPosition) / dt;
    particle.externalAcceleration = {};
}

void updateVelocityOne(Particle& particle, float dt)
{
    particle.velocity = (particle.position - particle.previousPosition) / dt;
}

void integrateParticlesScalar(Particle* particles,
                              const std::uint8_t* alive,
                              std::size_t count,
                              const Vec3& gravity,
                              float damping,
                              float dt)
{
    if (particles == nullptr || alive == nullptr || !(dt > 0.0f)) {
        return;
    }

    for (std::size_t i = 0; i < count; ++i) {
        if (alive[i] == 0) {
            continue;
        }
        integrateOne(particles[i], gravity, damping, dt);
    }
}

void updateParticleVelocitiesScalar(Particle* particles,
                                    const std::uint8_t* alive,
                                    std::size_t count,
                                    float dt)
{
    if (particles == nullptr || alive == nullptr || !(dt > 0.0f)) {
        return;
    }

    for (std::size_t i = 0; i < count; ++i) {
        if (alive[i] == 0) {
            continue;
        }
        updateVelocityOne(particles[i], dt);
    }
}

#if XPBD_CPU_X86

bool cpuSupportsSse2()
{
#if defined(_MSC_VER)
    int regs[4] = {};
    __cpuid(regs, 1);
    return (regs[3] & (1 << 26)) != 0;
#else
    unsigned int eax = 0;
    unsigned int ebx = 0;
    unsigned int ecx = 0;
    unsigned int edx = 0;
    if (__get_cpuid(1, &eax, &ebx, &ecx, &edx) == 0) {
        return false;
    }
    return (edx & bit_SSE2) != 0;
#endif
}

bool canIntegrateFour(const Particle* particles, const std::uint8_t* alive, std::size_t index)
{
    for (std::size_t lane = 0; lane < 4; ++lane) {
        if (alive[index + lane] == 0 || particles[index + lane].inverseMass <= 0.0f) {
            return false;
        }
    }
    return true;
}

bool canUpdateVelocityFour(const std::uint8_t* alive, std::size_t index)
{
    return alive[index] != 0 &&
           alive[index + 1] != 0 &&
           alive[index + 2] != 0 &&
           alive[index + 3] != 0;
}

__m128 loadPositionX(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].position.x,
                      particles[index + 2].position.x,
                      particles[index + 1].position.x,
                      particles[index].position.x);
}

__m128 loadPositionY(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].position.y,
                      particles[index + 2].position.y,
                      particles[index + 1].position.y,
                      particles[index].position.y);
}

__m128 loadPositionZ(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].position.z,
                      particles[index + 2].position.z,
                      particles[index + 1].position.z,
                      particles[index].position.z);
}

__m128 loadPreviousX(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].previousPosition.x,
                      particles[index + 2].previousPosition.x,
                      particles[index + 1].previousPosition.x,
                      particles[index].previousPosition.x);
}

__m128 loadPreviousY(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].previousPosition.y,
                      particles[index + 2].previousPosition.y,
                      particles[index + 1].previousPosition.y,
                      particles[index].previousPosition.y);
}

__m128 loadPreviousZ(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].previousPosition.z,
                      particles[index + 2].previousPosition.z,
                      particles[index + 1].previousPosition.z,
                      particles[index].previousPosition.z);
}

__m128 loadExternalX(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].externalAcceleration.x,
                      particles[index + 2].externalAcceleration.x,
                      particles[index + 1].externalAcceleration.x,
                      particles[index].externalAcceleration.x);
}

__m128 loadExternalY(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].externalAcceleration.y,
                      particles[index + 2].externalAcceleration.y,
                      particles[index + 1].externalAcceleration.y,
                      particles[index].externalAcceleration.y);
}

__m128 loadExternalZ(const Particle* particles, std::size_t index)
{
    return _mm_set_ps(particles[index + 3].externalAcceleration.z,
                      particles[index + 2].externalAcceleration.z,
                      particles[index + 1].externalAcceleration.z,
                      particles[index].externalAcceleration.z);
}

void integrateFourSse2(Particle* particles,
                       std::size_t index,
                       const Vec3& gravity,
                       float damping,
                       float dt)
{
    const __m128 damp = _mm_set1_ps(damping);
    const __m128 dtSq = _mm_set1_ps(dt * dt);
    const __m128 invDt = _mm_set1_ps(1.0f / dt);

    const __m128 px = loadPositionX(particles, index);
    const __m128 py = loadPositionY(particles, index);
    const __m128 pz = loadPositionZ(particles, index);

    const __m128 vxStep = _mm_mul_ps(_mm_sub_ps(px, loadPreviousX(particles, index)), damp);
    const __m128 vyStep = _mm_mul_ps(_mm_sub_ps(py, loadPreviousY(particles, index)), damp);
    const __m128 vzStep = _mm_mul_ps(_mm_sub_ps(pz, loadPreviousZ(particles, index)), damp);

    const __m128 ax = _mm_add_ps(_mm_set1_ps(gravity.x), loadExternalX(particles, index));
    const __m128 ay = _mm_add_ps(_mm_set1_ps(gravity.y), loadExternalY(particles, index));
    const __m128 az = _mm_add_ps(_mm_set1_ps(gravity.z), loadExternalZ(particles, index));

    const __m128 nx = _mm_add_ps(px, _mm_add_ps(vxStep, _mm_mul_ps(ax, dtSq)));
    const __m128 ny = _mm_add_ps(py, _mm_add_ps(vyStep, _mm_mul_ps(ay, dtSq)));
    const __m128 nz = _mm_add_ps(pz, _mm_add_ps(vzStep, _mm_mul_ps(az, dtSq)));

    const __m128 velX = _mm_mul_ps(_mm_sub_ps(nx, px), invDt);
    const __m128 velY = _mm_mul_ps(_mm_sub_ps(ny, py), invDt);
    const __m128 velZ = _mm_mul_ps(_mm_sub_ps(nz, pz), invDt);

    alignas(16) float oldX[4] = {};
    alignas(16) float oldY[4] = {};
    alignas(16) float oldZ[4] = {};
    alignas(16) float newX[4] = {};
    alignas(16) float newY[4] = {};
    alignas(16) float newZ[4] = {};
    alignas(16) float outVelX[4] = {};
    alignas(16) float outVelY[4] = {};
    alignas(16) float outVelZ[4] = {};

    _mm_store_ps(oldX, px);
    _mm_store_ps(oldY, py);
    _mm_store_ps(oldZ, pz);
    _mm_store_ps(newX, nx);
    _mm_store_ps(newY, ny);
    _mm_store_ps(newZ, nz);
    _mm_store_ps(outVelX, velX);
    _mm_store_ps(outVelY, velY);
    _mm_store_ps(outVelZ, velZ);

    for (std::size_t lane = 0; lane < 4; ++lane) {
        Particle& particle = particles[index + lane];
        particle.previousPosition = {oldX[lane], oldY[lane], oldZ[lane]};
        particle.position = {newX[lane], newY[lane], newZ[lane]};
        particle.velocity = {outVelX[lane], outVelY[lane], outVelZ[lane]};
        particle.externalAcceleration = {};
    }
}

void updateVelocityFourSse2(Particle* particles, std::size_t index, float dt)
{
    const __m128 invDt = _mm_set1_ps(1.0f / dt);

    const __m128 vx = _mm_mul_ps(_mm_sub_ps(loadPositionX(particles, index), loadPreviousX(particles, index)), invDt);
    const __m128 vy = _mm_mul_ps(_mm_sub_ps(loadPositionY(particles, index), loadPreviousY(particles, index)), invDt);
    const __m128 vz = _mm_mul_ps(_mm_sub_ps(loadPositionZ(particles, index), loadPreviousZ(particles, index)), invDt);

    alignas(16) float outX[4] = {};
    alignas(16) float outY[4] = {};
    alignas(16) float outZ[4] = {};

    _mm_store_ps(outX, vx);
    _mm_store_ps(outY, vy);
    _mm_store_ps(outZ, vz);

    for (std::size_t lane = 0; lane < 4; ++lane) {
        particles[index + lane].velocity = {outX[lane], outY[lane], outZ[lane]};
    }
}

void integrateParticlesSse2(Particle* particles,
                            const std::uint8_t* alive,
                            std::size_t count,
                            const Vec3& gravity,
                            float damping,
                            float dt)
{
    if (particles == nullptr || alive == nullptr || !(dt > 0.0f)) {
        return;
    }

    std::size_t i = 0;
    for (; i + 3 < count; i += 4) {
        if (canIntegrateFour(particles, alive, i)) {
            integrateFourSse2(particles, i, gravity, damping, dt);
            continue;
        }

        for (std::size_t lane = 0; lane < 4; ++lane) {
            if (alive[i + lane] != 0) {
                integrateOne(particles[i + lane], gravity, damping, dt);
            }
        }
    }

    for (; i < count; ++i) {
        if (alive[i] != 0) {
            integrateOne(particles[i], gravity, damping, dt);
        }
    }
}

void updateParticleVelocitiesSse2(Particle* particles,
                                  const std::uint8_t* alive,
                                  std::size_t count,
                                  float dt)
{
    if (particles == nullptr || alive == nullptr || !(dt > 0.0f)) {
        return;
    }

    std::size_t i = 0;
    for (; i + 3 < count; i += 4) {
        if (canUpdateVelocityFour(alive, i)) {
            updateVelocityFourSse2(particles, i, dt);
            continue;
        }

        for (std::size_t lane = 0; lane < 4; ++lane) {
            if (alive[i + lane] != 0) {
                updateVelocityOne(particles[i + lane], dt);
            }
        }
    }

    for (; i < count; ++i) {
        if (alive[i] != 0) {
            updateVelocityOne(particles[i], dt);
        }
    }
}

#else

bool cpuSupportsSse2()
{
    return false;
}

#endif

detail::SimdDispatch scalarDispatch()
{
    detail::SimdDispatch dispatch;
    dispatch.backend = SimdBackend::Scalar;
    dispatch.name = "scalar";
    dispatch.integrateParticles = integrateParticlesScalar;
    dispatch.updateParticleVelocities = updateParticleVelocitiesScalar;
    return dispatch;
}

detail::SimdDispatch sse2Dispatch()
{
    detail::SimdDispatch dispatch = scalarDispatch();
#if XPBD_CPU_X86
    dispatch.backend = SimdBackend::Sse2;
    dispatch.name = "sse2";
    dispatch.integrateParticles = integrateParticlesSse2;
    dispatch.updateParticleVelocities = updateParticleVelocitiesSse2;
#endif
    return dispatch;
}

std::atomic<bool> gInitialized{false};
std::mutex gInitMutex;
SimdInfo gInfo;
detail::SimdDispatch gDispatch = scalarDispatch();

void selectBackend(SimdPreference preference)
{
    gInfo.sse2Supported = cpuSupportsSse2();

    const bool allowSse2 = preference == SimdPreference::Auto || preference == SimdPreference::Sse2;
    if (allowSse2 && gInfo.sse2Supported) {
        gDispatch = sse2Dispatch();
    } else {
        gDispatch = scalarDispatch();
    }

    gInfo.backend = gDispatch.backend;
    gInfo.backendName = gDispatch.name;
}

}  // namespace

void initializeSimd(SimdPreference preference)
{
    if (gInitialized.load(std::memory_order_acquire)) {
        return;
    }

    std::lock_guard<std::mutex> lock(gInitMutex);
    if (gInitialized.load(std::memory_order_relaxed)) {
        return;
    }

    selectBackend(preference);
    gInitialized.store(true, std::memory_order_release);
}

const SimdInfo& simdInfo()
{
    initializeSimd();
    return gInfo;
}

const char* simdBackendName()
{
    return simdInfo().backendName;
}

namespace detail {

const SimdDispatch& simdDispatch()
{
    initializeSimd();
    return gDispatch;
}

}  // namespace detail

}  // namespace xpbd
