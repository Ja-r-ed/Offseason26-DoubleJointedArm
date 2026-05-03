// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShoulder.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubShoulder::SubShoulder() {
  _shoulderMotorConfig.SmartCurrentLimit(60);
  _shoulderMotorConfig.softLimit.ForwardSoftLimit(SHOULDER_MAX_ANGLE.value());
  _shoulderMotorConfig.softLimit.ReverseSoftLimit(SHOULDER_MIN_ANGLE.value());
  _shoulderMotorConfig.encoder.PositionConversionFactor(1.0 / SHOULDER_GEARING);
  _shoulderMotorConfig.encoder.VelocityConversionFactor(1.0 / SHOULDER_GEARING);
  _shoulderMotorConfig.closedLoop.Pid(SHOULDER_P, SHOULDER_I, SHOULDER_D);
  _shoulderMotorConfig.closedLoop.MaxOutput(1);
  _shoulderMotorConfig.closedLoop.MinOutput(-1);
  _shoulderMotor.OverwriteConfig(_shoulderMotorConfig);

  Logger::Log("Shoulder/ShoulderMotor", &_shoulderMotor);

  _shoulderMechanismRoot = _shoulderMechanism.GetRoot("shoulder_root", 1.5, 1.5);

  _shoulderLigament = _shoulderMechanismRoot->Append<frc::MechanismLigament2d>(
    "shoulder",
    SHOULDER_ARM_LENGTH.value(),
    SHOULDER_STARTING_ANGLE,
    6.0,
    frc::Color8Bit(0, 0, 255)
  );

  Logger::Log("Shoulder Mech", &_shoulderMechanism);
}

// This method will be called once per scheduler run
void SubShoulder::Periodic() {
}

void SubShoulder::SimulationPeriodic() {
    _shoulderSim.SetInputVoltage(_shoulderMotor.CalcSimVoltage());
    _shoulderSim.Update(20_ms);
    _shoulderMotor.IterateSim(_shoulderSim.GetVelocity(), _shoulderSim.GetAngle());
    _shoulderLigament->SetAngle(
        units::degree_t(_shoulderSim.GetAngle())
    );
}

frc2::CommandPtr SubShoulder::SetPositionTarget(units::degree_t target) {
    return RunOnce([this, target] {_shoulderMotor.SetPositionTarget(target);});
}

units::degree_t SubShoulder::GetPositionTarget() {
    return _shoulderMotor.GetPositionTarget();
}

units::degree_t SubShoulder::GetPosition() {
    return _shoulderMotor.GetPosition();
}

units::degrees_per_second_t SubShoulder::GetVelocity() {
    return _shoulderMotor.GetVelocity();
}

bool SubShoulder::ShoulderIsAtTarget() {
    return units::math::abs(GetPositionTarget() - GetPosition()) < SHOULDER_TOLERANCE;
}