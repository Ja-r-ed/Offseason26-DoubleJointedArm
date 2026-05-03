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

class SubElbow : public frc2::SubsystemBase {
 public:
  static SubElbow& GetInstance() {
    static SubElbow instance;
    return instance;
  }
  SubElbow();

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
  bool ElbowIsAtTarget();

 private:
  ICSparkFlex _elbowMotor{canid::ELBOW};
  rev::spark::SparkFlexConfig _elbowMotorConfig;

  static constexpr double ELBOW_P = 1.0;
  static constexpr double ELBOW_I = 0.0;
  static constexpr double ELBOW_D = 0.0;
  static constexpr double ELBOW_GEARING = 55.8;
  static constexpr units::degree_t ELBOW_MAX_ANGLE = 120_deg;
  static constexpr units::degree_t ELBOW_MIN_ANGLE = -20_deg;
  static constexpr units::degree_t ELBOW_STARTING_ANGLE = 0_deg;
  static constexpr units::degree_t ELBOW_TOLERANCE = 1_deg;

  // Simulation components
  static constexpr units::degree_t ELBOW_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t ELBOW_MOI = 0.5_kg_sq_m;
  static constexpr frc::DCMotor ELBOW_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr units::meter_t ELBOW_ARM_LENGTH = 0.5_m;
  frc::LinearSystem<2, 1, 2> _elbowArmSystem =
    frc::LinearSystemId::SingleJointedArmSystem(ELBOW_MOTOR_MODEL, ELBOW_MOI, ELBOW_GEARING);
  frc::sim::SingleJointedArmSim _elbowSim{_elbowArmSystem, ELBOW_MOTOR_MODEL,
    ELBOW_GEARING, ELBOW_ARM_LENGTH, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, false,
    ELBOW_STARTING_ANGLE};

  // Mech visualisation
  frc::MechanismLigament2d* _elbowLigament = nullptr;
};
