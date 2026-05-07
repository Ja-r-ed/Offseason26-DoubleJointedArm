// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubArm.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubArm::SubArm() {
  _shoulderMotorConfig.SmartCurrentLimit(60);
  _shoulderMotorConfig.softLimit.ForwardSoftLimit(SHOULDER_MAX_ANGLE.value());
  _shoulderMotorConfig.softLimit.ReverseSoftLimit(SHOULDER_MIN_ANGLE.value());
  _shoulderMotorConfig.encoder.PositionConversionFactor(1.0 / SHOULDER_GEARING);
  _shoulderMotorConfig.encoder.VelocityConversionFactor(1.0 / SHOULDER_GEARING);
  _shoulderMotorConfig.closedLoop.Pid(SHOULDER_P, SHOULDER_I, SHOULDER_D);
  _shoulderMotorConfig.closedLoop.MaxOutput(1);
  _shoulderMotorConfig.closedLoop.MinOutput(-1);
  _shoulderMotor.OverwriteConfig(_shoulderMotorConfig);

  _elbowMotorConfig.SmartCurrentLimit(60);
  _elbowMotorConfig.softLimit.ForwardSoftLimit(ELBOW_MAX_ANGLE.value());
  _elbowMotorConfig.softLimit.ReverseSoftLimit(ELBOW_MIN_ANGLE.value());
  _elbowMotorConfig.encoder.PositionConversionFactor(1.0 / ELBOW_GEARING);
  _elbowMotorConfig.encoder.VelocityConversionFactor(1.0 / ELBOW_GEARING);
  _elbowMotorConfig.closedLoop.Pid(ELBOW_P, ELBOW_I, ELBOW_D);
  _elbowMotorConfig.closedLoop.MaxOutput(1);
  _elbowMotorConfig.closedLoop.MinOutput(-1);
  _elbowMotor.OverwriteConfig(_elbowMotorConfig);

  Logger::Log("Shoulder/ShoulderMotor", &_shoulderMotor);
  Logger::Log("Elbow/ElbowMotor", &_elbowMotor);

  _shoulderMechanismRoot = _shoulderMechanism.GetRoot("shoulder_root", 1.5, 1.5);

  _shoulderLigament = _shoulderMechanismRoot->Append<frc::MechanismLigament2d>(
    "shoulder",
    SHOULDER_ARM_LENGTH.value(),
    SHOULDER_STARTING_ANGLE,
    6.0,
    frc::Color8Bit(0, 0, 255)
  );

  _elbowLigament = _shoulderLigament->Append<frc::MechanismLigament2d>("elbow",
    ELBOW_ARM_LENGTH.value(),
    ELBOW_START_ANGLE,
    4.0,
    frc::Color8Bit(255, 0, 0)
  );

  Logger::Log("Shoulder Mech", &_shoulderMechanism);
}

// This method will be called once per scheduler run
void SubArm::Periodic() {
}

void SubArm::SimulationPeriodic() {
    _shoulderSim.SetInputVoltage(_shoulderMotor.CalcSimVoltage());
    _shoulderSim.Update(20_ms);
    _shoulderMotor.IterateSim(_shoulderSim.GetVelocity(), _shoulderSim.GetAngle());
    _shoulderLigament->SetAngle(
        units::degree_t(_shoulderSim.GetAngle())
    );

    _elbowSim.SetInputVoltage(_elbowMotor.CalcSimVoltage());
    _elbowSim.Update(20_ms);
    _elbowMotor.IterateSim(_elbowSim.GetVelocity(), _elbowSim.GetAngle());
    _elbowLigament->SetAngle(
        units::degree_t(_elbowSim.GetAngle())
    );
}

frc2::CommandPtr SubArm::SetShoulderPositionTarget(units::degree_t target) {
    return RunOnce([this, target] {_shoulderMotor.SetPositionTarget(target);});
}

units::degree_t SubArm::GetShoulderPositionTarget() {
    return _shoulderMotor.GetPositionTarget();
}

units::degree_t SubArm::GetShoulderPosition() {
    return _shoulderMotor.GetPosition();
}

units::degrees_per_second_t SubArm::GetShoulderVelocity() {
    return _shoulderMotor.GetVelocity();
}

bool SubArm::ShoulderIsAtTarget() {
    return units::math::abs(GetShoulderPositionTarget() - GetShoulderPosition()) < SHOULDER_TOLERANCE;
}

frc2::CommandPtr SubArm::SetElbowPositionTarget(units::degree_t target) {
    return RunOnce([this, target] {_elbowMotor.SetPositionTarget(target);});
}

units::degree_t SubArm::GetElbowPositionTarget() {
    return _elbowMotor.GetPositionTarget();
}

units::degree_t SubArm::GetElbowPosition() {
    return _elbowMotor.GetPosition();
}

units::degrees_per_second_t SubArm::GetElbowVelocity() {
    return _elbowMotor.GetVelocity();
}

bool SubArm::ElbowIsAtTarget() {
    return units::math::abs(GetElbowPositionTarget() - GetElbowPosition()) < ELBOW_TOLERANCE;
}

units::degree_t SubArm::GetRelativeElbowPosition() {
    return GetElbowPosition() + GetShoulderPosition();
}