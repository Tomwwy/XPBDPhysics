#include "xpbd/xpbd_world.hpp"

namespace xpbd {

void XPBDWorld::verletIntegrationSystem(XPBDWorld& world, float dt)
{
    if (world.simd_ == nullptr || world.simd_->integrateParticles == nullptr) {
        return;
    }

    world.simd_->integrateParticles(world.particles_.data(),
                                    world.particles_.aliveData(),
                                    world.particles_.slots(),
                                    world.gravity_,
                                    world.damping_,
                                    dt);
}

}  // namespace xpbd
