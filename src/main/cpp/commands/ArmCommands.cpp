#include "commands/ArmCommands.h"
#include "subsystems/SubArm.h"
#include "utilities/Logger.h"

#include <frc/system/plant/DCMotor.h>
#include <cmath>

namespace cmd {

    std::pair<units::degree_t, units::degree_t> GetArmAnglesForPosition(std::pair<units::meter_t, units::meter_t> position) {
        // Inverse kinematics for a planar 2-link (shoulder -> elbow) arm.
        // Returns {shoulderAngle, elbowAngle} in degrees.
        // Uses units types internally and logs intermediate values.

        // Link lengths as units::meter_t
        const units::meter_t L1 = SubArm::SHOULDER_ARM_LENGTH;
        const units::meter_t L2 = SubArm::ELBOW_ARM_LENGTH;
        Logger::Log("GetArmAnglesForPosition/L1", L1.value());
        Logger::Log("GetArmAnglesForPosition/L2", L2.value());

        // Input position as units
        const units::meter_t x_m = position.first;
        const units::meter_t y_m = position.second;
        Logger::Log("GetArmAnglesForPosition/input x", x_m.value());
        Logger::Log("GetArmAnglesForPosition/input y", y_m.value());

        // Work in raw doubles for trig / math, but keep units around for clarity.
        double x = x_m.value();
        double y = y_m.value();

        double r2 = x * x + y * y;
        double r = std::sqrt(r2);
        Logger::Log("GetArmAnglesForPosition/r", r);

        // Reachability clamp: if the point is outside reachable workspace,
        // snap to the closest reachable point along the same direction.
        const double maxReach = (L1 + L2).value();
        const double minReach = std::fabs((L1 - L2).value());
        Logger::Log("GetArmAnglesForPosition/maxReach", maxReach);
        Logger::Log("GetArmAnglesForPosition/minReach", minReach);

        if (r > maxReach) {
            Logger::Log("GetArmAnglesForPosition.clamp", 1.0); // indicate clamped high
            x *= (maxReach / r);
            y *= (maxReach / r);
            r = maxReach;
            r2 = x * x + y * y;
            Logger::Log("GetArmAnglesForPosition/clamped x", x);
            Logger::Log("GetArmAnglesForPosition/clamped y", y);
            Logger::Log("GetArmAnglesForPosition/clamped r", r);
        } else if (r < minReach && r > 1e-9) {
            Logger::Log("GetArmAnglesForPosition.clamp", -1.0); // indicate clamped low
            x *= (minReach / r);
            y *= (minReach / r);
            r = minReach;
            r2 = x * x + y * y;
            Logger::Log("GetArmAnglesForPosition/clamped x", x);
            Logger::Log("GetArmAnglesForPosition/clamped y", y);
            Logger::Log("GetArmAnglesForPosition/clamped r", r);
        }

        // Law of cosines for elbow angle (angle between the two links)
        double cosTheta2 = (r2 - (L1.value() * L1.value()) - (L2.value() * L2.value())) / (2.0 * L1.value() * L2.value());
        if (cosTheta2 > 1.0) cosTheta2 = 1.0;
        if (cosTheta2 < -1.0) cosTheta2 = -1.0;
        Logger::Log("GetArmAnglesForPosition/cosTheta2", cosTheta2);

        double theta2 = std::acos(cosTheta2); // elbow flexion in radians
        Logger::Log("GetArmAnglesForPosition/theta2_rad", theta2);

        // Two possible configurations (elbow-up / elbow-down).
        // To pick the "elbow-up" solution use negative sin(theta2).
        double sinTheta2 = -std::sqrt(std::max(0.0, 1.0 - cosTheta2 * cosTheta2));
        Logger::Log("GetArmAnglesForPosition.sinTheta2", sinTheta2);

        // Compute shoulder angle using geometry
        double k1 = L1.value() + L2.value() * cosTheta2;
        double k2 = L2.value() * sinTheta2;
        Logger::Log("GetArmAnglesForPosition/k1", k1);
        Logger::Log("GetArmAnglesForPosition/k2", k2);

        double theta1 = std::atan2(y, x) - std::atan2(k2, k1); // radians
        Logger::Log("GetArmAnglesForPosition/theta1_rad", theta1);

        // Convert to units and degrees for return
        const units::radian_t theta1_rad{theta1};
        const units::radian_t theta2_rad{theta2};
        const units::degree_t theta1_deg = units::degree_t{theta1_rad};
        const units::degree_t theta2_deg = units::degree_t{theta2_rad};

        Logger::Log("GetArmAnglesForPosition/theta1", theta1_deg.value());
        Logger::Log("GetArmAnglesForPosition/theta2", theta2_deg.value());

        return std::make_pair(theta1_deg, theta2_deg);
    }

    bool IsPositionReachable(std::pair<units::meter_t, units::meter_t> position) {
        // Same link lengths as used in GetArmAnglesForPosition.
        const units::meter_t L1 = SubArm::SHOULDER_ARM_LENGTH;
        const units::meter_t L2 = SubArm::ELBOW_ARM_LENGTH;
        Logger::Log("IsPositionReachable/L1", L1.value());
        Logger::Log("IsPositionReachable/L2", L2.value());

        const units::meter_t x_m = position.first;
        const units::meter_t y_m = position.second;
        Logger::Log("IsPositionReachable/input x", x_m.value());
        Logger::Log("IsPositionReachable/input y", y_m.value());

        double x = x_m.value();
        double y = y_m.value();
        double r = std::sqrt(x * x + y * y);
        Logger::Log("IsPositionReachable/r", r);

        const double maxReach = (L1 + L2).value();
        const double minReach = std::fabs((L1 - L2).value());
        Logger::Log("IsPositionReachable/maxReach", maxReach);
        Logger::Log("IsPositionReachable/minReach", minReach);

        // Allow a tiny numerical tolerance.
        const double eps = 1e-9;
        if (r > maxReach + eps) {
            Logger::Log("IsPositionReachable.result", false);
            return false;
        }
        if (r + eps < minReach) {
            Logger::Log("IsPositionReachable.result", false);
            return false;
        }
        Logger::Log("IsPositionReachable.result", true);
        return true;
    }

    frc2::CommandPtr SetArmsTargetsForPosition(std::pair<units::meter_t, units::meter_t> position) {
        // Point-to-point arm command. The IK target is fixed, but the model-based feedforward
        // is recomputed EVERY scheduler cycle from the arm's live measured velocity AND
        // acceleration (both joints), and the targets are re-applied with that fresh feedforward.
        // This lets the M*a inertia (including the off-diagonal M12 coupling between the two
        // joints), the Coriolis term, gravity, and back-EMF all track the arm as it actually moves.
        const std::pair<units::degree_t, units::degree_t> angles = GetArmAnglesForPosition(position);
        const units::degree_t shoulderTarget = angles.first;   // model theta1 (motor coords match)
        const units::degree_t elbowTarget = -angles.second;    // model theta2 (elbow-up is negative, motor coords match)

        return SubArm::GetInstance().Run([shoulderTarget, elbowTarget] {
            const std::pair<units::volt_t, units::volt_t> ff = CalculateTwoJointedArmFeedforward(
                units::radian_t{shoulderTarget},
                units::radian_t{elbowTarget},
                units::radians_per_second_t{SubArm::GetInstance().GetShoulderVelocity()},
                units::radians_per_second_t{SubArm::GetInstance().GetElbowVelocity()},
                units::radians_per_second_squared_t{SubArm::GetInstance().GetShoulderAcceleration()},
                units::radians_per_second_squared_t{SubArm::GetInstance().GetElbowAcceleration()});

            // Motor positive direction == model positive angle for BOTH joints: the elbow target
            // is negated only to convert the IK's positive flexion angle into the model's negative
            // elbow-up angle. The returned model voltages are applied as-is (no extra sign flip).
            SubArm::GetInstance().SetShoulderPositionTarget(shoulderTarget, ff.first);
            SubArm::GetInstance().SetElbowPositionTarget(elbowTarget, ff.second);
        });
    }

    std::pair<units::volt_t, units::volt_t> CalculateTwoJointedArmFeedforward(
        units::radian_t shoulderAngle,
        units::radian_t elbowAngle,
        units::radians_per_second_t shoulderVelocity,
        units::radians_per_second_t elbowVelocity,
        units::radians_per_second_squared_t shoulderAccel,
        units::radians_per_second_squared_t elbowAccel)
    {
        // --- Physical constants (per rafi's "Two Jointed Arm Dynamics" whitepaper) ---
        constexpr double m1 = SubArm::SHOULDER_LINK_MASS.value();  // kg - shoulder link
        constexpr double m2 = SubArm::ELBOW_LINK_MASS.value();    // kg - elbow link
        constexpr double m3 = SubArm::ELBOW_MOTOR_MASS.value();   // kg - elbow motor, modelled as
                                                                  //      a point mass AT the elbow
                                                                  //      joint (on link 1)

        constexpr double l1 = SubArm::SHOULDER_ARM_LENGTH.value();  // m
        constexpr double l2 = SubArm::ELBOW_ARM_LENGTH.value();     // m

        // Distance from each joint to the center of mass of its link (uniform rod: L/2).
        // NOTE: only r1/r2 are COM distances. Every term that levers a mass OUTSIDE the
        // shoulder joint (link 2, the elbow motor) must use the FULL length l1, not r1.
        constexpr double r1 = l1 / 2.0;
        constexpr double r2 = l2 / 2.0;

        constexpr double g = 9.81;  // m/s^2

        // Moments of inertia about each link's center of mass (uniform rod: I = mL^2/12)
        constexpr double I1 = m1 * l1 * l1 / 12.0;
        constexpr double I2 = m2 * l2 * l2 / 12.0;

        // Each joint is driven by one NEO Vortex through its own gearbox (same motor model
        // the sim uses). The ideal-motor voltage law V = (R/Kt)*torque + (1/Kv)*speed
        // (equivalent to the paper's B and Kb matrices) is provided by frc::DCMotor.
        static constexpr frc::DCMotor kMotor = frc::DCMotor::NeoVortex();
        const double kGearShoulder = SubArm::SHOULDER_GEARING;
        const double kGearElbow = SubArm::ELBOW_GEARING;

        // --- Extract angles and velocities ---
        const double th1 = shoulderAngle.value();      // rad - link 1 angle from horizontal (CCW+)
        const double th2 = elbowAngle.value();          // rad - RELATIVE angle, link 2 CCW+ from link 1
        const double w1  = shoulderVelocity.value();    // rad/s
        const double w2  = elbowVelocity.value();       // rad/s
        const double a1  = shoulderAccel.value();       // rad/s^2
        const double a2  = elbowAccel.value();          // rad/s^2

        // Trig shorthand
        const double c1  = std::cos(th1);
        const double c2  = std::cos(th2);
        const double s2  = std::sin(th2);
        const double c12 = std::cos(th1 + th2);

        // === Inertia matrix M (paper, section 3) ===
        // M11: all masses contribute; link 1 also needs its parallel-axis term m1*r1^2,
        //      and link 2 / elbow motor lever about the FULL shoulder length l1.
        const double M11 = m1*r1*r1 + I1                       // shoulder link about base pivot
                         + m2*(l1*l1 + r2*r2) + I2             // elbow link COM through base pivot
                         + 2.0*m2*l1*r2*c2                     // distance between the two COMs
                         + m3 * (l1*l1);                       // elbow motor, point mass at the joint
        // M12 = M21 (coupling inertia)
        const double M12 = I2 + m2*(r2*r2 + l1*r2*c2);
        // M22 (elbow inertia)
        const double M22 = I2 + m2*r2*r2;

        // === Coriolis / centrifugal terms h = m2*l1*r2 (paper, section 4) ===
        const double h = m2 * l1 * r2;
        const double C1 = -h * s2 * (w2 * (2.0*w1 + w2));   // joint 1 (Coriolis + centrifugal)
        const double C2 =  h * s2 * (w1 * w1);              // joint 2 (centrifugal)

        // === Gravity vector (paper, section 5) + elbow motor mass ===
        const double tauGravity1 = (m1*r1 + m2*l1 + m3*l1) * g * c1
                                 + m2*r2 * g * c12;
        const double tauGravity2 = m2*r2 * g * c12;

        // === Joint torques required (N*m at the output shafts) ===
        const double tau1 = M11*a1 + M12*a2 + C1 + tauGravity1;
        const double tau2 = M12*a1 + M22*a2 + C2 + tauGravity2;

        // === Convert to motor voltages (paper, section 6) ===
        // Torque at the motor shaft is tau/G; motor shaft speed is G*w.
        const units::volt_t v1 = kMotor.Voltage(units::newton_meter_t{tau1 / kGearShoulder},
                                                units::radians_per_second_t{kGearShoulder * w1});
        const units::volt_t v2 = kMotor.Voltage(units::newton_meter_t{tau2 / kGearElbow},
                                                units::radians_per_second_t{kGearElbow * w2});

        return {v1, v2};
    }

}
