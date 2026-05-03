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

class SubShoulder : public frc2::SubsystemBase {
 public:
  static SubShoulder& GetInstance() {
    static SubShoulder instance;
    return instance;
  }
  SubShoulder();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  //CommandPtrs
  frc2::CommandPtr SetPositionTarget(units::degree_t target);

  //Getters
  units::degree_t GetPositionTarget();
  units::degree_t GetPosition();
  units::degrees_per_second_t GetVelocity();

  //Bool
  bool ShoulderIsAtTarget();

 private:
  ICSparkFlex _shoulderMotor{canid::SHOULDER};
  rev::spark::SparkFlexConfig _shoulderMotorConfig;

  static constexpr double SHOULDER_P = 1.0;
  static constexpr double SHOULDER_I = 0.0;
  static constexpr double SHOULDER_D = 0.0;
  static constexpr double SHOULDER_GEARING = 55.8;
  static constexpr units::degree_t SHOULDER_MAX_ANGLE = 120_deg;
  static constexpr units::degree_t SHOULDER_MIN_ANGLE = -20_deg;
  static constexpr units::degree_t SHOULDER_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t SHOULDER_TOLERANCE = 1_deg;

  // Simulation components
  static constexpr units::degree_t SHOULDER_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t SHOULDER_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor SHOULDER_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr units::meter_t SHOULDER_ARM_LENGTH = 0.5_m;
  frc::LinearSystem<2, 1, 2> _shoulderArmSystem =
    frc::LinearSystemId::SingleJointedArmSystem(SHOULDER_MOTOR_MODEL, SHOULDER_MOI, SHOULDER_GEARING);
  frc::sim::SingleJointedArmSim _shoulderSim{_shoulderArmSystem, SHOULDER_MOTOR_MODEL,
    SHOULDER_GEARING, SHOULDER_ARM_LENGTH, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE, false,
    SHOULDER_STARTING_ANGLE};

  // Mech visualisation
  frc::Mechanism2d _shoulderMechanism{3.0, 3.0};
  frc::MechanismRoot2d* _shoulderMechanismRoot = nullptr;
  frc::MechanismLigament2d* _shoulderLigament = nullptr;
};
