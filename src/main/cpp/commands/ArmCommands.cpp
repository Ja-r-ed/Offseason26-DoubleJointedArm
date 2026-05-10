#include "commands/ArmCommands.h"
#include "subsystems/SubArm.h"
#include "subsystems/SubArm.h"
#include "utilities/Logger.h"

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

    frc2::CommandPtr SetArmsTarget(std::pair<units::degree_t, units::degree_t> angles) {
        return SubArm::GetInstance().SetShoulderPositionTarget(angles.first)
        .AlongWith(SubArm::GetInstance().SetElbowPositionTarget(angles.second));
    }

    frc2::CommandPtr SetArmsTargetsForPosition(std::pair<units::meter_t, units::meter_t> position) {
        auto angles = GetArmAnglesForPosition(position);
        return SetArmsTarget(angles);
    }

}