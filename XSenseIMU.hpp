#pragma once
#include "engine/alice/alice_codelet.hpp"
#include "messages/imu.capnp.h"

#include "XSenseCallbackHandler.hpp"

class XSenseIMU : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Params
  ISAAC_PARAM(std::string, message, "Hello World!");

  // Communication
  ISAAC_PROTO_TX(ImuProto, imu);
 private:
  XsPortInfo mtPort;
  XsDevice* device;
  XsControl* control;
};
ISAAC_ALICE_REGISTER_CODELET(XSenseIMU);