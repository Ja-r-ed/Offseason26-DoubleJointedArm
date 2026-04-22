#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation3d.h>

namespace ICgeometry {
const units::meter_t FIELD_WIDTH = 8.036_m;
const units::meter_t FIELD_LENGTH = 16.541_m;

frc::Pose2d xPoseFlip(frc::Pose2d pose);
frc::Translation3d xTranslationFlip(frc::Translation3d translation3d);
frc::Pose2d yPoseFlip(frc::Pose2d pose);
frc::Pose2d xyPoseFlip(frc::Pose2d pose);

frc::Rotation2d PoseDirection(frc::Pose2d origin, frc::Pose2d destination);
frc::Pose2d GetFieldRelativePose(frc::Pose2d allianceRelativePose);
}  // namespace ICgeometry