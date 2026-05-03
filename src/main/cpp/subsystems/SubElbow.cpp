// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElbow.h"

#include <units/current.h>
#include <utilities/Logger.h>

#include "subsystems/SubShoulder.h"

SubElbow::SubElbow() {
  _elbowMotorConfig.SmartCurrentLimit(60);
  _elbowMotorConfig.softLimit.ForwardSoftLimit(ELBOW_MAX_ANGLE.value());
  _elbowMotorConfig.softLimit.ReverseSoftLimit(ELBOW_MIN_ANGLE.value());
  _elbowMotorConfig.encoder.PositionConversionFactor(1.0 / ELBOW_GEARING);
  _elbowMotorConfig.encoder.VelocityConversionFactor(1.0 / ELBOW_GEARING);
  _elbowMotorConfig.closedLoop.Pid(ELBOW_P, ELBOW_I, ELBOW_D);
  _elbowMotorConfig.closedLoop.MaxOutput(1);
  _elbowMotorConfig.closedLoop.MinOutput(-1);
  _elbowMotor.OverwriteConfig(_elbowMotorConfig);

  Logger::Log("Elbow/ElbowMotor", &_elbowMotor);

  frc::MechanismLigament2d* shoulderLigament = SubShoulder::GetInstance().GetShoulderLigament();

  _elbowLigament = shoulderLigament->Append<frc::MechanismLigament2d>("elbow",
    ELBOW_ARM_LENGTH.value(),
    ELBOW_START_ANGLE,
    4.0,
    frc::Color8Bit(255, 0, 0)
  );
}

// This method will be called once per scheduler run
void SubElbow::Periodic() {
}

void SubElbow::SimulationPeriodic() {
    _elbowSim.SetInputVoltage(_elbowMotor.CalcSimVoltage());
    _elbowSim.Update(20_ms);
    _elbowMotor.IterateSim(_elbowSim.GetVelocity(), _elbowSim.GetAngle());
    _elbowLigament->SetAngle(
        units::degree_t(_elbowSim.GetAngle())
    );
}

frc2::CommandPtr SubElbow::SetPositionTarget(units::degree_t target) {
    return RunOnce([this, target] {_elbowMotor.SetPositionTarget(target);});
}

units::degree_t SubElbow::GetPositionTarget() {
    return _elbowMotor.GetPositionTarget();
}

units::degree_t SubElbow::GetPosition() {
    return _elbowMotor.GetPosition();
}

units::degrees_per_second_t SubElbow::GetVelocity() {
    return _elbowMotor.GetVelocity();
}

bool SubElbow::ElbowIsAtTarget() {
    return units::math::abs(GetPositionTarget() - GetPosition()) < ELBOW_TOLERANCE;
}