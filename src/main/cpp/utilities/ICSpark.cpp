#include "utilities/ICSpark.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>
#include <rev/ClosedLoopTypes.h>

ICSpark::ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder,
                 rev::spark::SparkBaseConfigAccessor& configAccessor)
    : _spark(spark),
      _configAccessor(configAccessor),
      _encoder(inbuiltEncoder),
      _simSpark(spark, &_vortexModel) {}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
  // clang-format off
  //----------------------- Label --------------------------- Getter ---------------------------------------------------------------------------------------- Setter -------------------------------------------------
  builder.AddDoubleProperty("Position (tr)",                  [&] { return GetPosition().value(); },                                                          nullptr);
  builder.AddDoubleProperty("Velocity (rpm)",                 [&] { return GetVelocity().value(); },                                                          nullptr);
  builder.AddDoubleProperty("Duty Cycle",                     [&] { return GetDutyCycle(); },                                                                 nullptr);
  builder.AddDoubleProperty("Voltage (V)",                    [&] { return GetMotorVoltage().value(); },                                                      nullptr);
  builder.AddDoubleProperty("Current (A)",                    [&] { return GetStatorCurrent().value(); },                                                     nullptr);
  builder.AddDoubleProperty("Temperature (C)",                [&] { return _spark->GetMotorTemperature(); },                                                  nullptr);
  builder.AddDoubleProperty("Position Target (tr)",           [&] { return _positionTarget.value(); },                                                        [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target (rpm)",          [&] { return _velocityTarget.value(); },                                                        [&](double targ) { SetVelocityTarget(targ*1_tps); });
  builder.AddDoubleProperty("Profile Position Target (tr)",   [&] { return _latestMotionTarget.position.value(); },                                           [&](double targ) { SetMotionProfileTarget(targ*1_tr); });
  builder.AddDoubleProperty("Profile Velocity Target (rpm)",  [&] { return _latestMotionTarget.velocity.convert<units::revolutions_per_minute>().value(); },  nullptr);
  builder.AddDoubleProperty("Gains/Active Slot",              [&] { return _sparkPidController.GetSelectedSlot(); },                                          nullptr);
  builder.AddDoubleProperty("Gains/FB P",                     [&] { return _configCache.feedbackP; },                                                         [&](double P) { TuneFeedbackProportional(P); });
  builder.AddDoubleProperty("Gains/FB I",                     [&] { return _configCache.feedbackI; },                                                         [&](double I) { TuneFeedbackIntegral(I); });
  builder.AddDoubleProperty("Gains/FB D",                     [&] { return _configCache.feedbackD; },                                                         [&](double D) { TuneFeedbackDerivative(D); });
  builder.AddDoubleProperty("Gains/FF S (V)",                 [&] { return _configCache.feedforwardStaticFriction.value(); },                                 [&](double S) { TuneFeedforwardStaticFriction(S*1_V); });
  builder.AddDoubleProperty("Gains/FF Linear G (V)",          [&] { return _configCache.feedforwardLinearGravity.value(); },                                  [&](double lG) { TuneFeedforwardLinearGravity(lG*1_V); });
  builder.AddDoubleProperty("Gains/FF Rotational G (V)",      [&] { return _configCache.feedforwardRotationalGravity.value(); },                              [&](double rG) { TuneFeedforwardRotationalGravity(rG*1_V); });
  builder.AddDoubleProperty("Gains/FF V (V_per_rpm)",         [&] { return _configCache.feedforwardVelocity.value(); },                                       [&](double V) { TuneFeedforwardVelocity(VoltsPerRpm{V}); });
  builder.AddDoubleProperty("Gains/FF A (V_per_rpm_per_s)",   [&] { return _configCache.feedforwardAcceleration.value(); },                                   [&](double A) { TuneFeedforwardAcceleration(VoltsPerRpmPerS{A}); });
  builder.AddDoubleProperty("Motion/Max vel (rpm)",           [&] { return _configCache.motionMaxVelocity.value(); },                                         [&](double vel) { TuneMotionMaxVel(vel*1_rpm); });
  builder.AddDoubleProperty("Motion/Max accel (rpm_per_s)",   [&] { return _configCache.motionMaxAcceleration.value(); },                                     [&](double accel) { TuneMotionMaxAccel(accel*1_rev_per_m_per_s); });
  // clang-format on
}

rev::REVLibError ICSpark::Configure(rev::spark::SparkBaseConfig &config,
                                    rev::ResetMode resetMode,
                                    rev::PersistMode persistMode, bool async)
{
  rev::REVLibError error;
  if (async) {
    error = _spark->ConfigureAsync(config, resetMode, persistMode);
  } else {
    error = _spark->Configure(config, resetMode, persistMode);
    if (error != rev::REVLibError::kOk) {
      _configErrorAlert.SetText("CONFIGURATION ERROR in Spark controller ID" +
                                std::to_string(_spark->GetDeviceId()) + ": RevLibError ID " +
                                std::to_string(int(error)));
      _configErrorAlert.Set(true);
    }
  }
  // RefreshConfigCache(); // TODO: ConfigAccessor seems to be causing segfault, fix.
  return error;
}

rev::REVLibError ICSpark::AdjustConfig(rev::spark::SparkBaseConfig& config) {
  return Configure(config, rev::ResetMode::kNoResetSafeParameters,
                   rev::PersistMode::kPersistParameters);
};

rev::REVLibError ICSpark::AdjustConfigNoPersist(rev::spark::SparkBaseConfig& config) {
  return Configure(config, rev::ResetMode::kNoResetSafeParameters,
                   rev::PersistMode::kNoPersistParameters);
};

rev::REVLibError ICSpark::OverwriteConfig(rev::spark::SparkBaseConfig& config) {
  return Configure(config, rev::ResetMode::kResetSafeParameters,
                   rev::PersistMode::kPersistParameters);
};

void ICSpark::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
}

void ICSpark::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward,
                                rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kPosition;

  _sparkPidController.SetSetpoint(
      target.value(), rev::spark::SparkLowLevel::ControlType::kPosition, slot,
      _arbFeedForward.value() + CalculateFeedforward(_positionTarget, 0_tps).value());
}

void ICSpark::SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward,
                                 rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _arbFeedForward = arbFeedForward;
  _controlType = ControlType::kMaxMotion;
  _latestMotionTarget = {
      units::turn_t{_sparkPidController.GetMAXMotionSetpointPosition()},
      units::revolutions_per_minute_t{_sparkPidController.GetMAXMotionSetpointVelocity()}};

  _sparkPidController.SetSetpoint(target.value(),
                                  rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                  slot, _arbFeedForward.value());
}

void ICSpark::SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward,
                                     rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMotionProfile;

  UpdateMotionProfile();
}

void ICSpark::SetVelocityTarget(units::revolutions_per_minute_t target,
                                units::volt_t arbFeedForward, rev::spark::ClosedLoopSlot slot) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kVelocity;

  _sparkPidController.SetSetpoint(
      target.value(), rev::spark::SparkLowLevel::ControlType::kVelocity, slot,
      _arbFeedForward.value() + CalculateFeedforward(0_tr, _velocityTarget).value());
}

void ICSpark::SetDutyCycle(double speed) {
  _velocityTarget = units::revolutions_per_minute_t{0};
  _positionTarget = units::turn_t{0};
  _arbFeedForward = 0_V;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kDutyCycle;

  _sparkPidController.SetSetpoint(speed, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
}

void ICSpark::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _arbFeedForward = 0_V;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kVoltage;

  _sparkPidController.SetSetpoint(output.value(), rev::spark::SparkLowLevel::ControlType::kVoltage);
}

void ICSpark::UpdateMotionProfile(units::second_t loopTime) {
  if (GetControlType() == ControlType::kMotionProfile) {
    // In motion profile mode, we use the prev target state as the "current state"
    // and the sparkPIDController uses the next target state as its goal.
    auto prevVelTarget = _latestMotionTarget.velocity;
    _latestMotionTarget =
        _motionProfile.Calculate(loopTime, _latestMotionTarget, {_positionTarget, 0_rpm});
    auto accelTarget = (_latestMotionTarget.velocity - prevVelTarget) / loopTime;
    auto modelFeedForward = CalculateFeedforward(_latestMotionTarget.position,
                                                 _latestMotionTarget.velocity, accelTarget);
    _sparkPidController.SetSetpoint(_latestMotionTarget.position.value(), GetREVControlType(),
                                    _sparkPidController.GetSelectedSlot(),
                                    modelFeedForward.value() + _arbFeedForward.value());
  } else if (GetControlType() == ControlType::kMaxMotion) {
    // The built-in spark logic takes care of max motion.
    // just set the motion target to whatever max moton says for logging.
    _latestMotionTarget = {
        units::turn_t{_sparkPidController.GetMAXMotionSetpointPosition()},
        units::revolutions_per_minute_t{_sparkPidController.GetMAXMotionSetpointVelocity()}};
  }
}

units::volt_t ICSpark::CalculateFeedforward(units::turn_t pos, units::revolutions_per_minute_t vel,
                                            units::revolutions_per_minute_per_second_t accel) {
  auto kS = _configCache.feedforwardStaticFriction;
  auto kLG = _configCache.feedforwardLinearGravity;
  auto kRG = _configCache.feedforwardRotationalGravity;
  auto kV = _configCache.feedforwardVelocity;
  auto kA = _configCache.feedforwardAcceleration;

  return kS * wpi::sgn(vel) + kLG + kRG * units::math::cos(pos) + kV * vel + kA * accel;
}

rev::spark::SparkLowLevel::ControlType ICSpark::GetREVControlType() {
  auto controlType = GetControlType();
  if (controlType == ControlType::kMotionProfile) {
    return rev::spark::SparkLowLevel::ControlType::kPosition;
  } else {
    return (rev::spark::SparkLowLevel::ControlType)controlType;
  }
}

void ICSpark::TuneMotionMaxVel(units::revolutions_per_minute_t maxVelocity) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.maxMotion.CruiseVelocity(maxVelocity.value(),
                                             _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneMotionMaxAccel(units::revolutions_per_minute_per_second_t maxAcceleration) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.maxMotion.MaxAcceleration(maxAcceleration.value(),
                                              _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedbackProportional(double P) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.P(P, _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedbackIntegral(double I) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.I(I, _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedbackDerivative(double D) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.D(D, _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardStaticFriction(units::volt_t S) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kS(S.value(), _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardLinearGravity(units::volt_t linearG) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kG(linearG.value(), _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardRotationalGravity(units::volt_t rotationalG) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kCos(rotationalG.value(), _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardVelocity(VoltsPerRpm V) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kV(V.value(), _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardAcceleration(VoltsPerRpmPerS A) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kA(A.value(), _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardCosineRatio(double ratio) {
  rev::spark::SparkBaseConfig config;
  config.closedLoop.feedForward.kCosRatio(ratio, _sparkPidController.GetSelectedSlot());
  AdjustConfigNoPersist(config);
}

void ICSpark::RefreshConfigCache() {
  _configCache.feedbackP = _configAccessor.closedLoop.GetP(_sparkPidController.GetSelectedSlot());
  _configCache.feedbackI = _configAccessor.closedLoop.GetI(_sparkPidController.GetSelectedSlot());
  _configCache.feedbackD = _configAccessor.closedLoop.GetD(_sparkPidController.GetSelectedSlot());
  _configCache.feedforwardStaticFriction = units::volt_t{
      _configAccessor.closedLoop.feedForward.getkS(_sparkPidController.GetSelectedSlot())};
  _configCache.feedforwardLinearGravity = units::volt_t{
      _configAccessor.closedLoop.feedForward.getkG(_sparkPidController.GetSelectedSlot())};
  _configCache.feedforwardRotationalGravity = units::volt_t{
      _configAccessor.closedLoop.feedForward.getkCos(_sparkPidController.GetSelectedSlot())};
  _configCache.feedforwardVelocity = VoltsPerRpm{
      _configAccessor.closedLoop.feedForward.getkV(_sparkPidController.GetSelectedSlot())};
  _configCache.feedforwardAcceleration = VoltsPerRpmPerS{
      _configAccessor.closedLoop.feedForward.getkA(_sparkPidController.GetSelectedSlot())};
  _configCache.feedforwardCosineRatio =
      _configAccessor.closedLoop.feedForward.getkCosRatio(_sparkPidController.GetSelectedSlot());
  _configCache.motionMaxVelocity =
      units::revolutions_per_minute_t{_configAccessor.closedLoop.maxMotion.GetCruiseVelocity(
          _sparkPidController.GetSelectedSlot())};
  _configCache.motionMaxAcceleration = units::revolutions_per_minute_per_second_t{
      _configAccessor.closedLoop.maxMotion.GetMaxAcceleration(
          _sparkPidController.GetSelectedSlot())};

  // Update motion profile constraints
  _motionProfile = frc::TrapezoidProfile<units::turns>(
      {_configCache.motionMaxVelocity, _configCache.motionMaxAcceleration});
}

units::revolutions_per_minute_t ICSpark::GetVelocity() {
  return units::revolutions_per_minute_t{_encoder.GetVelocity()};
}

units::turn_t ICSpark::GetPosition() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return units::turn_t{_simSpark.GetPosition()};
  }
  return units::turn_t{_encoder.GetPosition()};
}

double ICSpark::GetDutyCycle() const {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return _simSpark.GetAppliedOutput();
  } else {
    return _spark->GetAppliedOutput();
  }
}

units::volt_t ICSpark::GetMotorVoltage() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return CalcSimVoltage();
  } else {
    return _spark->GetAppliedOutput() * _spark->GetBusVoltage() * 1_V;
  }
}

units::ampere_t ICSpark::GetStatorCurrent() {
  return _spark->GetOutputCurrent() * 1_A;
}

units::celsius_t ICSpark::GetTemperature() {
  return _spark->GetMotorTemperature() * 1_degC;
}

units::volt_t ICSpark::CalcSimVoltage() {
  return _simSpark.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage();
}

void ICSpark::IterateSim(units::revolutions_per_minute_t velocity,
                         std::optional<units::turn_t> position) {
  const units::second_t dt = 20_ms;
  const units::volt_t batteryVoltage = frc::sim::RoboRioSim::GetVInVoltage();
  _simSpark.iterate(velocity.value(), batteryVoltage.value(), dt.value());

  // REV's spark sim can work without explicitly telling it the positon of the mechanism,
  // but we find that it falls out of sync with the physics model if we don't set it.
  if (position.has_value()) {
    _simSpark.SetPosition(position->value());
  }
}

bool ICSpark::InMotionMode() {
  return GetControlType() == ControlType::kMotionProfile ||
         GetControlType() == ControlType::kMaxMotion;
}

ICSpark::MPState ICSpark::CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                                               units::second_t lookahead) {
  return _motionProfile.Calculate(lookahead, current, {goalPosition, 0_rpm});
}

void ICSpark::CheckAlerts() {
  // Temperature alert logic
  if (GetTemperature() > 70_degC) {
    if (!_temperatureAlert.Get()) {
      _temperatureAlert.SetText("TEMPERATURE TOO HIGH in Spark controller ID" +
                                std::to_string(_spark->GetDeviceId()));
      _temperatureAlert.Set(true);
    }
  } else if (_temperatureAlert.Get()) {
    _temperatureAlert.Set(false);
  }

  // Current alert timing logic
  if (GetStatorCurrent() > 80_A) {
    _currentAlertTimer.Start();
  } else if (_currentAlertTimer.IsRunning()) {
    _currentAlertTimer.Stop();
    _currentAlertTimer.Reset();
  }

  // Current alert display logic
  if (_currentAlertTimer.Get() > 2_s) {
    if (!_currentAlert.Get()) {
      _currentAlert.SetText("CURRENT TOO HIGH in Spark controller ID" +
                            std::to_string(_spark->GetDeviceId()));
      _currentAlert.Set(true);
    }
  } else if (_currentAlert.Get()) {
    _currentAlert.Set(false);
  }
}