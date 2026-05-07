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

#include "Constants.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkFlexConfigAccessor.h"

class SubArm : public frc2::SubsystemBase {
 public:
  static SubArm& GetInstance() {
    static SubArm instance;
    return instance;
  }
  SubArm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  //CommandPtrs
  frc2::CommandPtr SetShoulderPositionTarget(units::degree_t target);
  frc2::CommandPtr SetElbowPositionTarget(units::degree_t target);

  //Getters
  units::degree_t GetShoulderPositionTarget();
  units::degree_t GetShoulderPosition();
  units::degrees_per_second_t GetShoulderVelocity();
  units::degree_t GetElbowPositionTarget();
  units::degree_t GetElbowPosition();
  units::degrees_per_second_t GetElbowVelocity();
  units::degree_t GetRelativeElbowPosition();

  //Bool
  bool ShoulderIsAtTarget();
  bool ElbowIsAtTarget();

 private:
  ICSparkFlex _shoulderMotor{canid::SHOULDER};
  rev::spark::SparkFlexConfig _shoulderMotorConfig;

  ICSparkFlex _elbowMotor{canid::ELBOW};
  rev::spark::SparkFlexConfig _elbowMotorConfig;

  static constexpr double SHOULDER_P = 0.0;
  static constexpr double SHOULDER_I = 0.0;
  static constexpr double SHOULDER_D = 0.0;
  static constexpr double SHOULDER_GEARING = 55.8;
  static constexpr units::degree_t SHOULDER_MAX_ANGLE = 120_deg;
  static constexpr units::degree_t SHOULDER_MIN_ANGLE = -20_deg;
  static constexpr units::degree_t SHOULDER_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t SHOULDER_TOLERANCE = 1_deg;

  static constexpr double ELBOW_P = 0.0;
  static constexpr double ELBOW_I = 0.0;
  static constexpr double ELBOW_D = 0.0;
  static constexpr double ELBOW_GEARING = 55.8;
  static constexpr units::degree_t ELBOW_MAX_ANGLE = 120_deg;
  static constexpr units::degree_t ELBOW_MIN_ANGLE = -60_deg;
  static constexpr units::degree_t ELBOW_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t ELBOW_TOLERANCE = 1_deg;

  // Simulation components
  static constexpr units::degree_t SHOULDER_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t SHOULDER_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor SHOULDER_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr units::meter_t SHOULDER_ARM_LENGTH = 0.5_m;
  frc::LinearSystem<2, 1, 2> _shoulderArmSystem =
    frc::LinearSystemId::SingleJointedArmSystem(SHOULDER_MOTOR_MODEL, SHOULDER_MOI, SHOULDER_GEARING);
  frc::sim::SingleJointedArmSim _shoulderSim{_shoulderArmSystem, SHOULDER_MOTOR_MODEL,
    SHOULDER_GEARING, SHOULDER_ARM_LENGTH, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE, true,
    SHOULDER_STARTING_ANGLE};

  static constexpr units::degree_t ELBOW_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t ELBOW_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor ELBOW_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr units::meter_t ELBOW_ARM_LENGTH = 0.5_m;
  frc::LinearSystem<2, 1, 2> _elbowArmSystem =
    frc::LinearSystemId::SingleJointedArmSystem(ELBOW_MOTOR_MODEL, ELBOW_MOI, ELBOW_GEARING);
  frc::sim::SingleJointedArmSim _elbowSim{_elbowArmSystem, ELBOW_MOTOR_MODEL,
    ELBOW_GEARING, ELBOW_ARM_LENGTH, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, true,
    ELBOW_STARTING_ANGLE};

  // Mech visualisation
  frc::Mechanism2d _shoulderMechanism{3.0, 3.0};
  frc::MechanismRoot2d* _shoulderMechanismRoot = nullptr;
  frc::MechanismLigament2d* _shoulderLigament = nullptr;
  frc::MechanismLigament2d* _elbowLigament = nullptr;
};
