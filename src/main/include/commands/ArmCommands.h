#include <frc2/command/Commands.h>
#include <utility>
#include <units/angle.h>
#include <units/length.h>

namespace cmd {
    std::pair<units::degree_t, units::degree_t> GetArmAnglesForPosition(std::pair<units::meter_t, units::meter_t> position);
    bool IsPositionReachable(std::pair<units::meter_t, units::meter_t> position);
    frc2::CommandPtr SetArmsTarget(std::pair<units::degree_t, units::degree_t> angles);
    frc2::CommandPtr SetArmsTargetsForPosition(std::pair<units::meter_t, units::meter_t> position);
}