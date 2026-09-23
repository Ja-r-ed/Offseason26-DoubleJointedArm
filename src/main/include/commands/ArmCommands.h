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
     * from rafi's "Two Jointed Arm Dynamics" whitepaper (Team 449), section 6, eq. 26:
     *
     *   u = B^-1 [ M(theta) theta_ddot + C(theta_dot,theta) theta_dot + tau_g(theta) + Kb theta_dot ]
     *
     * This accounts for:
     * - Gravity compensation for both links (plus the elbow motor mass, modelled as a
     *   point mass at the elbow joint: it adds m3*l1^2 to M11 and m3*g*l1*cos(th1) to tau_g1)
     * - The full inertia matrix (including m1*r1^2 parallel-axis and cross-coupling terms)
     * - Coriolis and centrifugal torques
     * - Motor back-EMF and winding resistance (ideal motor law V = (R/Kt)*tau + (1/Kv)*w,
     *   evaluated with frc::DCMotor::NeoVortex())
     *
     * Angle convention: Both angles are measured from horizontal, positive counterclockwise.
     *   shoulderAngle = absolute angle of the first link from horizontal.
     *   elbowAngle    = RELATIVE angle between the two links (not absolute), CCW+.
     *     If your IK returns an interior flexion angle, use relative = -(flexion) for the
     *     elbow-up configuration.
     *
     * SIGN CONVENTION OF THE RETURN VALUE: the returned voltages produce torques in the
     * model's coordinate system (positive = CCW on theta). Apply them to the motors as-is,
     * in exactly the same coordinates you use for the target angles: if the elbow is commanded
     * with relative = -(flexion) (elbow-up), the motor CAN reads that same negative angle, so a
     * positive returned voltage is a positive (CCW) torque on the forearm -- no extra negation.
     * Only negate a returned voltage if that joint's motor positive direction does NOT match
     * the model angle you passed in (for example a motor physically mounted reversed).
     *
     * @param shoulderAngle      Current shoulder angle (radians, from horizontal)
     * @param elbowAngle         Elbow angle relative to shoulder link (radians, CCW+)
     * @param shoulderVelocity   Current shoulder angular velocity (rad/s)
     * @param elbowVelocity      Current elbow angular velocity (rad/s)
     * @param shoulderAccel      Shoulder angular acceleration (rad/s^2), default 0
     * @param elbowAccel         Elbow angular acceleration (rad/s^2), default 0
     * @return Pair of {shoulderVoltage, elbowVoltage} to apply as arb feedforward (model coordinates)
     */
    std::pair<units::volt_t, units::volt_t> CalculateTwoJointedArmFeedforward(
        units::radian_t shoulderAngle,
        units::radian_t elbowAngle,
        units::radians_per_second_t shoulderVelocity,
        units::radians_per_second_t elbowVelocity,
        units::radians_per_second_squared_t shoulderAccel = 0_rad_per_s_sq,
        units::radians_per_second_squared_t elbowAccel = 0_rad_per_s_sq);
}