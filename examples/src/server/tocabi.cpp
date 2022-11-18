// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();

  std::vector<raisim::ArticulatedSystem*> tocabi;

  const size_t N = 5;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      tocabi.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\tocabi\\dyros_tocabi.urdf"));
      tocabi.back()->setGeneralizedCoordinate(
          {double(2 * i), double(j), 1.0, 1.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      tocabi.back()->setGeneralizedForce(Eigen::VectorXd::Zero(tocabi.back()->getDOF()));
      tocabi.back()->setName("tocabi" + std::to_string(j + i * N));
    }
  }

  raisim::RaisimServer server(&world);

  // auto tocabi = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\tocabi\\dyros_tocabi.urdf");

  // raisim::RaisimServer server(&world);

  // tocabi->setBasePos({0,0,1.5});

  server.launchServer();
    while (1) {
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    server.integrateWorldThreadSafe();
  }
  server.killServer();
}