#include "commands/ArmCommands.h"
#include "subsystems/SubArm.h"

#include <frc/system/plant/DCMotor.h>

#include <cmath>
#include <utility>

#include "gtest/gtest.h"

// Reference implementation of rafi's "Two Jointed Arm Dynamics" (Team 449 whitepaper),
// computed independently from the code under test. Uses the same physical constants that
// the feedforward reads from SubArm, and the same ideal-motor law (V = (R/Kt)*tau + (1/Kv)*w)
// with the WPILib NEO Vortex model. Any discrepancy with CalculateTwoJointedArmFeedforward
// is a regression in the arm dynamics math.
namespace {

double Torque1(double th1, double th2, double w1, double w2, double a1, double a2) {
  const double m1 = SubArm::SHOULDER_LINK_MASS.value();
  const double m2 = SubArm::ELBOW_LINK_MASS.value();
  const double m3 = SubArm::ELBOW_MOTOR_MASS.value();
  const double l1 = SubArm::SHOULDER_ARM_LENGTH.value();
  const double l2 = SubArm::ELBOW_ARM_LENGTH.value();
  const double r1 = l1 / 2.0;
  const double r2 = l2 / 2.0;
  const double I1 = m1 * l1 * l1 / 12.0;
  const double I2 = m2 * l2 * l2 / 12.0;
  const double g = 9.81;

  const double c1 = std::cos(th1);
  const double c2 = std::cos(th2);
  const double s2 = std::sin(th2);
  const double c12 = std::cos(th1 + th2);

  // Inertia matrix (paper section 3)
  const double M11 = m1 * r1 * r1 + I1 + m2 * (l1 * l1 + r2 * r2) + I2
                     + 2.0 * m2 * l1 * r2 * c2 + m3 * l1 * l1;
  const double M12 = I2 + m2 * (r2 * r2 + l1 * r2 * c2);

  // Coriolis / centrifugal (paper section 4)
  const double h = m2 * l1 * r2;
  const double C1 = -h * s2 * (2.0 * w1 * w2 + w2 * w2);

  // Gravity (paper section 5), + elbow motor mass as a point mass at the joint
  const double TauG1 = (m1 * r1 + m2 * l1 + m3 * l1) * g * c1 + m2 * r2 * g * c12;

  return M11 * a1 + M12 * a2 + C1 + TauG1;
}

double Torque2(double th1, double th2, double w1, double w2, double a1, double a2) {
  const double m2 = SubArm::ELBOW_LINK_MASS.value();
  const double l1 = SubArm::SHOULDER_ARM_LENGTH.value();
  const double r2 = SubArm::ELBOW_ARM_LENGTH.value() / 2.0;
  const double I2 = m2 * SubArm::ELBOW_ARM_LENGTH.value() * SubArm::ELBOW_ARM_LENGTH.value() / 12.0;
  const double g = 9.81;

  const double c2 = std::cos(th2);
  const double s2 = std::sin(th2);
  const double c12 = std::cos(th1 + th2);

  const double M12 = I2 + m2 * (r2 * r2 + l1 * r2 * c2);
  const double M22 = I2 + m2 * r2 * r2;
  const double h = m2 * l1 * r2;
  const double C2 = h * s2 * w1 * w1;
  const double TauG2 = m2 * r2 * g * c12;

  return M12 * a1 + M22 * a2 + C2 + TauG2;
}

// Ideal-motor law: V = R/Kt * torque + 1/Kv * speed, evaluated at the MOTOR shaft.
double MotorVoltage(double torqueAtOutput, double outputRadPerSec, double gearing) {
  const frc::DCMotor motor = frc::DCMotor::NeoVortex();
  const double coeffTorque = (motor.R.value() / motor.Kt.value()) / gearing;
  const double coeffBackEmf = gearing / motor.Kv.value();
  return coeffTorque * torqueAtOutput + coeffBackEmf * outputRadPerSec;
}

}  // namespace

namespace cmd {

TEST(ArmFeedforwardTest, GravityOnlyHold) {
  // Static hold: arm horizontal-out, forearm slightly flexed back (elbow-up).
  // No motion -> only the gravity-compensation terms should matter.
  const double th1 = 0.0;
  const double th2 = -0.5;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2},
      0_rad_per_s, 0_rad_per_s, 0_rad_per_s_sq, 0_rad_per_s_sq);

  const double w1 = 0.0, w2 = 0.0, a1 = 0.0, a2 = 0.0;
  const double expectedV1 =
      MotorVoltage(Torque1(th1, th2, w1, w2, a1, a2), 0.0, SubArm::SHOULDER_GEARING);
  const double expectedV2 =
      MotorVoltage(Torque2(th1, th2, w1, w2, a1, a2), 0.0, SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6) << "shoulder gravity FF";
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6) << "elbow gravity FF";

  // Golden regression values (kg: 0.5/0.5, m: 0.5/0.5, NEO Vortex, 55.8:1).
  EXPECT_NEAR(ff.first.value(), 0.284043, 1e-6);
  EXPECT_NEAR(ff.second.value(), 0.0642853, 1e-6);
}

TEST(ArmFeedforwardTest, PureInertiaCoupling) {
  // Arm straight up (th1 = pi/2) with the forearm inline (th2 = 0). Gravity and
  // Coriolis terms vanish (c1 = 0, c12 = 0, s2 = 0), leaving pure inertia response
  // to a shoulder acceleration. The elbow must also feel the coupling torque M12*a1.
  const double th1 = std::acos(-1.0) / 2.0;
  const double th2 = 0.0;
  const double a1 = 1.0;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2},
      0_rad_per_s, 0_rad_per_s, units::radians_per_second_squared_t{a1}, 0_rad_per_s_sq);

  const double expectedV1 =
      MotorVoltage(Torque1(th1, th2, 0.0, 0.0, a1, 0.0), 0.0, SubArm::SHOULDER_GEARING);
  const double expectedV2 =
      MotorVoltage(Torque2(th1, th2, 0.0, 0.0, a1, 0.0), 0.0, SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6) << "shoulder inertia FF";
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6) << "elbow coupling inertia FF";

  // Golden regression values.
  EXPECT_NEAR(ff.first.value(), 0.0199124, 1e-6);
  EXPECT_NEAR(ff.second.value(), 0.00622262, 1e-6);
}

TEST(ArmFeedforwardTest, MotionCoriolisAndBackEmf) {
  // Both joints moving; no acceleration. Exercises the Coriolis/centrifugal terms
  // and the back-EMF voltage while the gravitational share stays in.
  const double th1 = 0.3;
  const double th2 = -0.6;
  const double w1 = 0.7, w2 = 0.3;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2},
      units::radians_per_second_t{w1}, units::radians_per_second_t{w2},
      0_rad_per_s_sq, 0_rad_per_s_sq);

  const double expectedV1 = MotorVoltage(Torque1(th1, th2, w1, w2, 0.0, 0.0), w1,
                                         SubArm::SHOULDER_GEARING);
  const double expectedV2 = MotorVoltage(Torque2(th1, th2, w1, w2, 0.0, 0.0), w2,
                                         SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6) << "shoulder motion FF";
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6) << "elbow motion FF";

  // Golden regression values.
  EXPECT_NEAR(ff.first.value(), 0.929475, 1e-6);
  EXPECT_NEAR(ff.second.value(), 0.346866, 1e-6);
}

TEST(ArmFeedforwardTest, CoriolisCouplingSign) {
  // Regression for the Coriolis sign bug that existed in the old h = -r1*r2*s2*...
  // implementation. At this pose, spinning the shoulder up while the arm is parked must
  // produce a NEGATIVE elbow feedforward: with th2 = -0.6 (s2 < 0), the net centrifugal
  // torque on the elbow C2 = h*s2*w1^2 is negative. Comparing the moving vs the parked
  // command removes gravity entirely.
  const double th1 = 0.3;
  const double th2 = -0.6;
  const double w1 = 0.7;

  const auto ffMoving = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2}, units::radians_per_second_t{w1},
      0_rad_per_s, 0_rad_per_s_sq, 0_rad_per_s_sq);
  const auto ffStill = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2}, 0_rad_per_s, 0_rad_per_s,
      0_rad_per_s_sq, 0_rad_per_s_sq);

  // Expected Coriolis-only elbow contribution: C2 * (R/Kt)/G with C2 = h*s2*w1^2 < 0.
  const double m2 = SubArm::ELBOW_LINK_MASS.value();
  const double l1 = SubArm::SHOULDER_ARM_LENGTH.value();
  const double r2 = SubArm::ELBOW_ARM_LENGTH.value() / 2.0;
  const double h = m2 * l1 * r2;
  const double s2 = std::sin(th2);
  const double C2 = h * s2 * w1 * w1;
  const frc::DCMotor motor = frc::DCMotor::NeoVortex();
  const double coeffTorque = (motor.R.value() / motor.Kt.value()) / SubArm::ELBOW_GEARING;

  EXPECT_NEAR(ffMoving.second.value() - ffStill.second.value(), coeffTorque * C2, 1e-6);
  EXPECT_LT(ffMoving.second.value() - ffStill.second.value(), 0.0);
}

TEST(ArmFeedforwardTest, BackEmfOnly) {
  // Arm straight up, forearm inline: no gravity, no Coriolis (s2 = 0). Only the
  // back-EMF term should appear, and with the correct sign: shoulder FF is positive
  // while the elbow travels backwards and hence goes negative.
  const double th1 = std::acos(-1.0) / 2.0;
  const double th2 = 0.0;
  const double w1 = 2.0, w2 = -1.5;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2}, units::radians_per_second_t{w1},
      units::radians_per_second_t{w2}, 0_rad_per_s_sq, 0_rad_per_s_sq);

  const double expectedV1 = MotorVoltage(0.0, w1, SubArm::SHOULDER_GEARING);
  const double expectedV2 = MotorVoltage(0.0, w2, SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6);
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6);

  // Golden regression values.
  EXPECT_NEAR(ff.first.value(), 1.85279, 1e-4);
  EXPECT_NEAR(ff.second.value(), -1.38959, 1e-4);
}

TEST(ArmFeedforwardTest, ElbowAccelCouplesIntoShoulder) {
  // Straight-up pose (gravity = Coriolis = 0): a pure ELBOW acceleration must demand torque
  // at BOTH joints through the off-diagonal inertia terms (shoulder gets M12*accel2,
  // elbow gets M22*accel2). Confirms acceleration is wired into every dynamics term.
  const double th1 = std::acos(-1.0) / 2.0;
  const double th2 = 0.0;
  const double a2 = 1.0;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2},
      0_rad_per_s, 0_rad_per_s, 0_rad_per_s_sq, units::radians_per_second_squared_t{a2});

  const double expectedV1 =
      MotorVoltage(Torque1(th1, th2, 0.0, 0.0, 0.0, a2), 0.0, SubArm::SHOULDER_GEARING);
  const double expectedV2 =
      MotorVoltage(Torque2(th1, th2, 0.0, 0.0, 0.0, a2), 0.0, SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6) << "shoulder coupling from elbow accel";
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6) << "elbow accel FF";

  // M12 at this pose = I2 + m2*(r2^2 + l1*r2) = 1/96 + 0.5*(1/16 + 1/8) = 0.1041667 (N*m)
  EXPECT_GT(ff.first.value(), 0.0);
  EXPECT_NEAR(ff.first.value(), 0.059737157 * 0.104166667, 1e-6);
}

TEST(ArmFeedforwardTest, CombinedAccelerationAndMotion) {
  // Both joints moving AND accelerating in a general pose: the feedforward must match the
  // full model torque = M*accel + Coriolis + gravity, plus back-EMF from both velocities.
  const double th1 = 0.3;
  const double th2 = -0.6;
  const double w1 = 0.7, w2 = 0.3, a1 = 0.5, a2 = -1.2;
  const auto ff = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2}, units::radians_per_second_t{w1},
      units::radians_per_second_t{w2}, units::radians_per_second_squared_t{a1},
      units::radians_per_second_squared_t{a2});

  const double expectedV1 = MotorVoltage(Torque1(th1, th2, w1, w2, a1, a2), w1,
                                         SubArm::SHOULDER_GEARING);
  const double expectedV2 = MotorVoltage(Torque2(th1, th2, w1, w2, a1, a2), w2,
                                         SubArm::ELBOW_GEARING);

  EXPECT_NEAR(ff.first.value(), expectedV1, 1e-6) << "shoulder combined FF";
  EXPECT_NEAR(ff.second.value(), expectedV2, 1e-6) << "elbow combined FF";

  // The acceleration terms must measurably change the output vs. the acceleration-free case.
  const auto ffNoAccel = CalculateTwoJointedArmFeedforward(
      units::radian_t{th1}, units::radian_t{th2}, units::radians_per_second_t{w1},
      units::radians_per_second_t{w2}, 0_rad_per_s_sq, 0_rad_per_s_sq);
  EXPECT_NE(ff.first.value(), ffNoAccel.first.value());
  EXPECT_NE(ff.second.value(), ffNoAccel.second.value());
}

}  // namespace cmd