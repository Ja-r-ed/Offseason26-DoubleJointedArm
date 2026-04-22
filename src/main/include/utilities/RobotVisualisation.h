#pragma once

#include "utilities/Logger.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

class RobotVisualisation {
 public:
  RobotVisualisation() { Logger::Log("RobotVisualisation", &_display); }
  static RobotVisualisation& GetInstance() {
    static RobotVisualisation intstance;
    return intstance;
  }

  frc::Mechanism2d _display{0.75, 0.75};

  // Deploy/Intake Visualisation
  frc::MechanismRoot2d* _deployRoot = _display.GetRoot("Deploy Root", 0.375, 0.075);
  frc::MechanismLigament2d* _deployLigament =
    _deployRoot->Append<frc::MechanismLigament2d>("Deploy", 0.1, 90_deg);
  MechanismCircle2d _intakeWheel{_deployLigament, "IntakeWheel", 0.01, 90_deg};

  // Indexer Visualisation
  frc::MechanismRoot2d* _indexerMechRoot = _display.GetRoot("indexerRoot", 0.250, 0.125);
  MechanismCircle2d _indexerMechCircle{_indexerMechRoot, "indexerCircle", 0.10, 0_deg};

  // Outdexer Visualisation
  frc::MechanismRoot2d* _outdexerMechRoot = _display.GetRoot("outdexerRoot", 0.130, 0.200);
  MechanismCircle2d _outdexerMechCircle{_outdexerMechRoot, "outdexerCircle", 0.02, 0_deg};

  // Feeder Visualisation
  frc::MechanismRoot2d* _feederMechRoot = _display.GetRoot("feederRoot", 0.055, 0.170);
  frc::MechanismLigament2d* _feederMechUpperConnector =
    _feederMechRoot->Append<frc::MechanismLigament2d>("feederUpperConnector", 0.1, 90_deg, 0);
  MechanismCircle2d _feederMechTopWheel{
    _feederMechUpperConnector, "shooterTopWheel", 0.025, 0_deg};
  frc::MechanismLigament2d* _feederMechLowerConnector =
    _feederMechRoot->Append<frc::MechanismLigament2d>("feederLowerConnector", 0.1, -90_deg, 0);
  MechanismCircle2d _feederMechBottomWheel{
    _feederMechLowerConnector, "feederBottomWheel", 0.025, 0_deg};

  // Turret Visualisation
  frc::MechanismRoot2d* _turretMechRoot = _display.GetRoot("turretRoot", 0.055, 0.335);
  MechanismCircle2d _turretMechCircle{_turretMechRoot, "turretCircle", 0.05, 0_deg};

  // Hood Visualisation
   frc::MechanismRoot2d* _hoodMechRoot = _display.GetRoot("hoodRoot", 0.055, 0.430);
   MechanismCircle2d _hoodMechCircle{_hoodMechRoot, "hoodCircle", 0.05, 0_deg};

  // Shooter Visualisation
  frc::MechanismRoot2d* _shooterMechRoot = _display.GetRoot("shooterRoot", 0.055, 0.550);
   frc::MechanismLigament2d* _shooterMechUpperConnector =
  _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterUpperConnector", 0.05, 90_deg, 0);
   MechanismCircle2d _shooterMechTopRoller{
   _shooterMechUpperConnector, "shooterTopRoller", 0.025, 0_deg};
   frc::MechanismLigament2d* _shooterMechLowerConnector =
  _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterLowerConnector", 0.05, -90_deg, 0);
   MechanismCircle2d _shooterMechBottomRoller{
  _shooterMechLowerConnector, "shooterBottomRoller", 0.025, 0_deg};

  // Climber Visualsation
  frc::MechanismRoot2d* _climberMechRoot = _display.GetRoot("climberRoot", 0.544, 0.125);
  frc::MechanismLigament2d* _climberMechElevator = 
    _climberMechRoot->Append<frc::MechanismLigament2d>("climberMechElevator", 0, 90_deg);
};