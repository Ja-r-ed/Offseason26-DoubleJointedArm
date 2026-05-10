// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "subsystems/SubArm.h"
#include "commands/ArmCommands.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().OnTrue(cmd::SetArmsTargetsForPosition({0.5_m, 0.5_m}));
  _driverController.B().OnTrue(cmd::SetArmsTargetsForPosition({0.5_m, -0.5_m}));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
