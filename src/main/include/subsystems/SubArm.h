// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/Commands.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/util/Color.h>
#include <frc/filter/LinearFilter.h>
#include <units/angular_acceleration.h>

#include "Constants.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkFlexConfigAccessor.h"

class SubArm : public frc2::SubsystemBase
{
public:
  static SubArm &GetInstance()
  {
    static SubArm instance;
    return instance;
  }
  SubArm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  // Apply per-joint position targets with an arbitrary feedforward. These re-issue the
  // closed-loop reference every time they are called, so a command may call them each
  // scheduler cycle with a freshly computed (live) feedforward.
  void SetShoulderPositionTarget(units::degree_t shoulderTarget, units::volt_t shoulderFF);
  void SetElbowPositionTarget(units::degree_t elbowTarget, units::volt_t elbowFF);

  // Getters
  units::degree_t GetShoulderPositionTarget();
  units::degree_t GetShoulderPosition();
  units::degrees_per_second_t GetShoulderVelocity();
  units::degree_t GetElbowPositionTarget();
  units::degree_t GetElbowPosition();
  units::degrees_per_second_t GetElbowVelocity();
  units::degree_t GetRelativeElbowPosition();
  // Measured angular acceleration (low-pass filtered derivative of the encoder velocity,
  // updated every Periodic()). Same units and sign convention as the velocity getters.
  units::degrees_per_second_squared_t GetShoulderAcceleration();
  units::degrees_per_second_squared_t GetElbowAcceleration();

  // Bool
  bool ShoulderIsAtTarget();
  bool ElbowIsAtTarget();

  static constexpr units::meter_t SHOULDER_ARM_LENGTH = 0.5_m;
  static constexpr units::meter_t ELBOW_ARM_LENGTH = 0.5_m;
  static constexpr units::kilogram_t SHOULDER_LINK_MASS = 0.5_kg;
  static constexpr units::kilogram_t ELBOW_LINK_MASS = 0.5_kg;
  static constexpr units::kilogram_t ELBOW_MOTOR_MASS = 0.0_kg;

  // Gearbox ratios (used by the two-joint feedforward and the sim models).
  static constexpr double SHOULDER_GEARING = 55.8;
  static constexpr double ELBOW_GEARING = 55.8;

private:
  ICSparkFlex _shoulderMotor{canid::SHOULDER};
  rev::spark::SparkFlexConfig _shoulderMotorConfig;

  ICSparkFlex _elbowMotor{canid::ELBOW};
  rev::spark::SparkFlexConfig _elbowMotorConfig;

  static constexpr double SHOULDER_P = 0.1;
  static constexpr double SHOULDER_I = 0.0;
  static constexpr double SHOULDER_D = 0.0;
  static constexpr units::degree_t SHOULDER_MAX_ANGLE = 180_deg;
  static constexpr units::degree_t SHOULDER_MIN_ANGLE = -40_deg;
  static constexpr units::degree_t SHOULDER_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t SHOULDER_TOLERANCE = 1_deg;

  static constexpr double ELBOW_P = 0.1;
  static constexpr double ELBOW_I = 0.0;
  static constexpr double ELBOW_D = 0.0;
  static constexpr units::degree_t ELBOW_MAX_ANGLE = 180_deg;
  static constexpr units::degree_t ELBOW_MIN_ANGLE = -180_deg;
  static constexpr units::degree_t ELBOW_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t ELBOW_TOLERANCE = 1_deg;

  // Measured joint accelerations (deg/s^2): single-pole low-pass filtered derivative of the
  // encoder velocity, computed in Periodic(). The time constant rejects encoder noise while
  // still capturing real arm dynamics for the model feedforward.
  static constexpr units::second_t ACCEL_FILTER_TIME_CONSTANT = 0.10_s;
  frc::LinearFilter<double> _shoulderAccelFilter =
      frc::LinearFilter<double>::SinglePoleIIR(ACCEL_FILTER_TIME_CONSTANT.value(), 20_ms);
  frc::LinearFilter<double> _elbowAccelFilter =
      frc::LinearFilter<double>::SinglePoleIIR(ACCEL_FILTER_TIME_CONSTANT.value(), 20_ms);
  units::degrees_per_second_t _lastShoulderVelocity{0};
  units::degrees_per_second_t _lastElbowVelocity{0};
  bool _shoulderAccelInitialized = false;
  bool _elbowAccelInitialized = false;
  units::degrees_per_second_squared_t _shoulderAccel{0};
  units::degrees_per_second_squared_t _elbowAccel{0};

  // Simulation components
  static constexpr units::degree_t SHOULDER_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t SHOULDER_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor SHOULDER_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _shoulderArmSystem =
      frc::LinearSystemId::SingleJointedArmSystem(SHOULDER_MOTOR_MODEL, SHOULDER_MOI, SHOULDER_GEARING);
  frc::sim::SingleJointedArmSim _shoulderSim{_shoulderArmSystem, SHOULDER_MOTOR_MODEL,
                                             SHOULDER_GEARING, SHOULDER_ARM_LENGTH, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE, true,
                                             SHOULDER_STARTING_ANGLE};

  static constexpr units::degree_t ELBOW_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t ELBOW_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor ELBOW_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _elbowArmSystem =
      frc::LinearSystemId::SingleJointedArmSystem(ELBOW_MOTOR_MODEL, ELBOW_MOI, ELBOW_GEARING);
  frc::sim::SingleJointedArmSim _elbowSim{_elbowArmSystem, ELBOW_MOTOR_MODEL,
                                          ELBOW_GEARING, ELBOW_ARM_LENGTH, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, true,
                                          ELBOW_STARTING_ANGLE};

  // Mech visualisation
  frc::Mechanism2d _shoulderMechanism{3.0, 3.0};
  frc::MechanismRoot2d *_shoulderMechanismRoot = nullptr;
  frc::MechanismLigament2d *_shoulderLigament = nullptr;
  frc::MechanismLigament2d *_elbowLigament = nullptr;
};
