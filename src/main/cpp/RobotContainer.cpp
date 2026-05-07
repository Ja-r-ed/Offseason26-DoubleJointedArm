// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "subsystems/SubArm.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().OnTrue(SubArm::GetInstance().SetShoulderPositionTarget(0_deg));
  _driverController.B().OnTrue(SubArm::GetInstance().SetShoulderPositionTarget(90_deg));
  _driverController.X().OnTrue(SubArm::GetInstance().SetElbowPositionTarget(0_deg));
  _driverController.Y().OnTrue(SubArm::GetInstance().SetElbowPositionTarget(90_deg));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
