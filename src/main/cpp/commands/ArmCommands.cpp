#include "commands/ArmCommands.h"
#include "subsystems/SubArm.h"
#include "utilities/Logger.h"

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
        // Single command that requires SubArm. Runs once at runtime: computes the IK angles
        // and feedforward from the *current* arm state and sets both motor targets together.
        // Keep all calculation inside the RunOnce so it happens at runtime, and use one
        // CommandPtr so there is no parallel/subsystem conflict.
        std::pair<units::degree_t, units::degree_t> angles = [position]{
            return GetArmAnglesForPosition(position);
        }();
        std::pair<units::volt_t, units::volt_t> ff = [position, &angles]{
            return CalculateTwoJointedArmFeedforward(
                units::radian_t{angles.first},
                units::radian_t{-angles.second},  // relative elbow angle, negative theta2 (matches motor sign)
                units::angular_velocity::radians_per_second_t{SubArm::GetInstance().GetShoulderVelocity()},
                units::angular_velocity::radians_per_second_t{SubArm::GetInstance().GetElbowVelocity()},
                0_rad_per_s_sq,
                0_rad_per_s_sq
            );
        }();
        return SubArm::GetInstance().SetShoulderAndElbowPositionTargets(
        angles.first, -angles.second, ff.first, ff.second);
        // return SubArm::GetInstance().SetShoulderAndElbowPositionTargets(
        // angles.first, -angles.second, 0_V, 0_V);
    }

    std::pair<units::volt_t, units::volt_t> CalculateTwoJointedArmFeedforward(
        units::radian_t shoulderAngle,
        units::radian_t elbowAngle,
        units::radians_per_second_t shoulderVelocity,
        units::radians_per_second_t elbowVelocity,
        units::radians_per_second_squared_t shoulderAccel,
        units::radians_per_second_squared_t elbowAccel)
    {
        // --- Physical constants ---
        constexpr double m1 = SubArm::SHOULDER_LINK_MASS.value();    // kg - shoulder link
        constexpr double m2 = SubArm::ELBOW_LINK_MASS.value();      // kg - elbow link
        constexpr double m3 = SubArm::ELBOW_MOTOR_MASS.value();     // kg - elbow motor (mounted at joint)

        // Arm link lengths
        constexpr double L1 = SubArm::SHOULDER_ARM_LENGTH.value();   // m
        constexpr double L2 = SubArm::ELBOW_ARM_LENGTH.value();     // m

        // Distance from each joint to the center of mass of its link (assume uniform: L/2)
        constexpr double r1 = L1 / 2.0;
        constexpr double r2 = L2 / 2.0;

        constexpr double g = 9.81;  // m/s^2

        // Moments of inertia about each link's center of mass (uniform rod: I = mL^2/12)
        constexpr double I1 = m1 * L1 * L1 / 12.0;
        constexpr double I2 = m2 * L2 * L2 / 12.0;

        // NEO Vortex motor constants
        constexpr double kV_actual = 12.0 / (6784.0 * (2.0 * M_PI / 60.0));  // V/(rad/s) on motor shaft
        constexpr double kt = 1.0 / kV_actual;  // N*m/A (torque constant)
        constexpr double G = 55.8;        // Gear ratio (shoulder and elbow)
        constexpr double efficiency = 0.85;  // Gearbox efficiency

        // --- Extract angles and velocities ---
        const double th1 = shoulderAngle.value();      // rad
        const double th2 = elbowAngle.value();          // rad (relative to shoulder)
        const double w1  = shoulderVelocity.value();    // rad/s
        const double w2  = elbowVelocity.value();       // rad/s
        const double a1  = shoulderAccel.value();       // rad/s^2
        const double a2  = elbowAccel.value();          // rad/s^2

        // Trig shorthand
        const double c1 = std::cos(th1);
        const double c2 = std::cos(th2);
        const double s2 = std::sin(th2);

        // === Mass (inertia) matrix M ===
        // M[0][0]: effective inertia at shoulder (all masses contribute)
        const double M11 = I1 + I2 + m2 * (r1*r1 + r2*r2 + 2.0*r1*r2*c2)
                         + m3 * (L1*L1 + r2*r2 + 2.0*L1*r2*c2);
        // M[0][1] = M[1][0]: coupling inertia
        const double M12 = I2 + m2 * r2 * (r1 * c2 + r2)
                         + m3 * r2 * (L1 * c2 + r2);
        // M[1][1]: effective inertia at elbow
        const double M22 = I2 + m2 * r2 * r2 + m3 * r2 * r2;

        // === Coriolis / centrifugal matrix C ===
        const double h = -r1 * r2 * s2 * (m2 + m3);
        const double C1 = h * (w2 * (2.0 * w1 + w2));
        const double C2 = h * w1 * w1;

        // === Gravity vector ===
        // Positive torque = counterclockwise (opposes downward gravity)
        const double G1 = (m1*r1 + m2*L1 + m3*L1) * g * c1
                        + (m2*r2 + m3*r2) * g * std::cos(th1 + th2);
        const double G2 = (m2*r2 + m3*r2) * g * std::cos(th1 + th2);

        // === Joint torques (N*m at the output shaft) ===
        const double tau1 = M11 * a1 + M12 * a2 + C1 + G1;
        const double tau2 = M12 * a1 + M22 * a2 + C2 + G2;

        // === Convert to motor voltages ===
        // Motor voltage: V = (tau * G) / (kt * efficiency) + kV * G * omega
        const double v1 = (tau1 * G) / (kt * efficiency) + kV_actual * G * w1;
        const double v2 = (tau2 * G) / (kt * efficiency) + kV_actual * G * w2;

        return {units::volt_t{v1}, units::volt_t{v2}};
    }

}
