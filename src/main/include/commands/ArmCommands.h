#include <frc2/command/Commands.h>
#include <utility>
#include <units/angle.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

namespace cmd {
    std::pair<units::degree_t, units::degree_t> GetArmAnglesForPosition(std::pair<units::meter_t, units::meter_t> position);
    bool IsPositionReachable(std::pair<units::meter_t, units::meter_t> position);
    frc2::CommandPtr SetArmsTargetsForPosition(std::pair<units::meter_t, units::meter_t> position);

    /**
     * Calculate the feedforward voltages for a two-jointed arm using the full dynamics model
     * from rafi's "Two Jointed Arm Dynamics" whitepaper (Team 449).
     *
     * This accounts for:
     * - Gravity compensation for both links (including the elbow motor mass on the shoulder)
     * - Cross-coupling inertial torques (accelerating one joint affects the other)
     * - Coriolis and centrifugal forces
     * - Motor back-EMF and winding resistance
     *
     * Angle convention: Both angles are measured from horizontal, positive counterclockwise.
     *   shoulderAngle = absolute angle of the first link from horizontal.
     *   elbowAngle    = RELATIVE angle between the two links (not absolute).
     *     If your IK returns an absolute elbow angle, convert with: relative = absolute - shoulder.
     *
     * @param shoulderAngle      Current shoulder angle (radians, from horizontal)
     * @param elbowAngle         Elbow angle relative to shoulder link (radians)
     * @param shoulderVelocity   Current shoulder angular velocity (rad/s)
     * @param elbowVelocity      Current elbow angular velocity (rad/s)
     * @param shoulderAccel      Shoulder angular acceleration (rad/s^2), default 0
     * @param elbowAccel         Elbow angular acceleration (rad/s^2), default 0
     * @return Pair of {shoulderVoltage, elbowVoltage} to apply as arb feedforward
     */
    std::pair<units::volt_t, units::volt_t> CalculateTwoJointedArmFeedforward(
        units::radian_t shoulderAngle,
        units::radian_t elbowAngle,
        units::radians_per_second_t shoulderVelocity,
        units::radians_per_second_t elbowVelocity,
        units::radians_per_second_squared_t shoulderAccel = 0_rad_per_s_sq,
        units::radians_per_second_squared_t elbowAccel = 0_rad_per_s_sq);
}