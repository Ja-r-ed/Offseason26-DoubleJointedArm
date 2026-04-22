#include "utilities/ICgeometry.h"
#include "utilities/Logger.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

#include <cmath>
#include <units/angle.h>
#include <units/length.h>

namespace ICgeometry {
frc::Pose2d xPoseFlip(frc::Pose2d pose) {
  units::meter_t xdiff = units::math::abs(FIELD_LENGTH / 2 - pose.X());
  units::meter_t x =
    (pose.X() < FIELD_LENGTH / 2) ? FIELD_LENGTH / 2 + xdiff : FIELD_LENGTH / 2 - xdiff;
  return frc::Pose2d(x, pose.Y(), frc::Rotation2d{180_deg}-pose.Rotation());
}

frc::Translation3d xTranslationFlip(frc::Translation3d translation3d) {
  units::meter_t xdiff = units::math::abs(FIELD_LENGTH / 2 - translation3d.X());
  units::meter_t x =
    (translation3d.X() < FIELD_LENGTH / 2) ? FIELD_LENGTH / 2 + xdiff : FIELD_LENGTH / 2 - xdiff;
  return frc::Translation3d(x, translation3d.Y(), translation3d.Z());
}

frc::Pose2d yPoseFlip(frc::Pose2d pose) {
  units::meter_t ydiff = units::math::abs(FIELD_WIDTH / 2 - pose.Y());
  units::meter_t y =
    (pose.Y() < FIELD_WIDTH / 2) ? FIELD_WIDTH / 2 + ydiff : FIELD_WIDTH / 2 - ydiff;
  return frc::Pose2d(pose.X(), y, -pose.Rotation());
}

frc::Pose2d xyPoseFlip(frc::Pose2d pose) {
  return yPoseFlip(xPoseFlip(pose));
}

frc::Rotation2d PoseDirection(frc::Pose2d origin, frc::Pose2d destination) {
  double x = (origin.X() - destination.X()).value();
  double y = (origin.Y() - destination.Y()).value();
  return frc::Rotation2d(static_cast<units::angle::radian_t>(std::tan(y / x)));
}

frc::Pose2d GetFieldRelativePose(frc::Pose2d allianceRelativePose) {
  if(frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed) {
    return ICgeometry::xyPoseFlip(allianceRelativePose);
  } else {
    return allianceRelativePose;
  }
};
}  // namespace ICgeometry