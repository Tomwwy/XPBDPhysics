#include "math_internal.hpp"

#include <cstdint>
#include <mutex>

#if defined(UTILS_HAS_X86_SIMD)
#if defined(_MSC_VER)
#include <intrin.h>
#elif defined(__GNUC__) || defined(__clang__)
#include <cpuid.h>
#endif
#endif

namespace {

struct DispatchState {
    std::once_flag once;
    utils::SimdSupport support;
    const utils::detail::MathOps* ops = &utils::detail::scalarMathOps();
};

DispatchState& state() {
    static DispatchState s;
    return s;
}

#if defined(UTILS_HAS_X86_SIMD)

bool bitSet(int value, int bit) {
    return (value & (1 << bit)) != 0;
}

void cpuid(int leaf, int subleaf, int out[4]) {
#if defined(_MSC_VER)
    __cpuidex(out, leaf, subleaf);
#elif defined(__GNUC__) || defined(__clang__)
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    __cpuid_count(static_cast<unsigned int>(leaf), static_cast<unsigned int>(subleaf), a, b, c, d);
    out[0] = static_cast<int>(a);
    out[1] = static_cast<int>(b);
    out[2] = static_cast<int>(c);
    out[3] = static_cast<int>(d);
#else
    out[0] = out[1] = out[2] = out[3] = 0;
#endif
}

std::uint64_t xgetbv0() {
#if defined(_MSC_VER)
    return _xgetbv(0);
#elif defined(__GNUC__) || defined(__clang__)
    std::uint32_t eax = 0;
    std::uint32_t edx = 0;
    __asm__ volatile("xgetbv" : "=a"(eax), "=d"(edx) : "c"(0));
    return (static_cast<std::uint64_t>(edx) << 32) | eax;
#else
    return 0;
#endif
}

utils::SimdSupport detectSimdSupport() {
    utils::SimdSupport support;
    int regs[4] = {};
    cpuid(0, 0, regs);
    const int maxLeaf = regs[0];
    if (maxLeaf < 1) return support;

    cpuid(1, 0, regs);
    support.sse2 = bitSet(regs[3], 26);
    support.sse41 = bitSet(regs[2], 19);

    const bool cpuAvx = bitSet(regs[2], 28);
    const bool osXsave = bitSet(regs[2], 27);
    bool osAvx = false;
    if (cpuAvx && osXsave) {
        const std::uint64_t xcr0 = xgetbv0();
        osAvx = (xcr0 & 0x6) == 0x6;
    }
    support.avx = cpuAvx && osAvx;

    if (maxLeaf >= 7) {
        cpuid(7, 0, regs);
        support.avx2 = support.avx && bitSet(regs[1], 5);
    }

    return support;
}

#else

utils::SimdSupport detectSimdSupport() {
    return {};
}

#endif

void installBestBackend() {
    DispatchState& s = state();
    s.support = detectSimdSupport();
    s.ops = &utils::detail::scalarMathOps();
    s.support.active = utils::SimdLevel::Scalar;

#if defined(UTILS_HAS_X86_SIMD)
    if (s.support.avx2) {
        s.ops = &utils::detail::avx2MathOps();
        s.support.active = utils::SimdLevel::AVX2;
    } else if (s.support.avx) {
        s.ops = &utils::detail::avxMathOps();
        s.support.active = utils::SimdLevel::AVX;
    } else if (s.support.sse2) {
        s.ops = &utils::detail::sse2MathOps();
        s.support.active = utils::SimdLevel::SSE2;
    }
#endif
}

}  // namespace

namespace utils {

void initializeSimd() {
    std::call_once(state().once, installBestBackend);
}

SimdSupport simdSupport() {
    initializeSimd();
    return state().support;
}

SimdLevel activeSimdLevel() {
    return simdSupport().active;
}

const char* simdLevelName(SimdLevel level) {
    switch (level) {
        case SimdLevel::Scalar: return "scalar";
        case SimdLevel::SSE2: return "SSE2";
        case SimdLevel::AVX: return "AVX";
        case SimdLevel::AVX2: return "AVX2";
    }
    return "unknown";
}

namespace detail {

const MathOps& mathOps() {
    initializeSimd();
    return *state().ops;
}

}  // namespace detail
}  // namespace utils
