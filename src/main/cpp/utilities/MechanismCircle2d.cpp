#include "utilities/MechanismCircle2d.h"

void MechanismCircle2d::SetAngle(units::degree_t angle) {
    size_t spokes = _spokeLigaments.size();
    for (unsigned int i = 0; i < spokes; i++) {
        _spokeLigaments[i]->SetAngle(angle + (360_deg/spokes) * i);
    }
}

void MechanismCircle2d::SetIndicatorColor(const frc::Color8Bit& color) {
    _spokeLigaments[0]->SetColor(color);
}

void MechanismCircle2d::SetCircleColor(const frc::Color8Bit& color) {
    for (frc::MechanismLigament2d* spoke : _spokeLigaments) {
        spoke->SetColor(color);
    }
}