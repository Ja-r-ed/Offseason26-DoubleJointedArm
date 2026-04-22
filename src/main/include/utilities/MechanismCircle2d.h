#pragma once

#include <string_view>
#include <units/angle.h>
#include <frc/util/Color8Bit.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

class MechanismCircle2d {
    public:
        template<typename T>
        requires std::same_as<T*, frc::MechanismRoot2d*> || std::convertible_to<T*, frc::MechanismObject2d*>
        MechanismCircle2d(T* location, std::string name, double radius,
            units::degree_t angle,
            int spokes=3,
            double spokeWidth=6.0,
            const frc::Color8Bit& indicatorColor={51, 149, 234}, /* #3395ea */
            const frc::Color8Bit& spokeColor={255, 255, 255} /* #ffffff */
        ) {
            //Create circle background spokes
            for (int i = 0; i < spokes; i++) {
                /* If it's the first spoke, set its color to indicatorColor */
                const frc::Color8Bit& color = i == 0 ? indicatorColor : spokeColor;
                frc::MechanismLigament2d* spoke = location->template 
                    Append<frc::MechanismLigament2d>(
                        name+"_spoke"+std::to_string(i),
                        radius,
                        angle+(360_deg/spokes)*i,
                        spokeWidth,
                        color); //append spoke ligament to chosen location
                _spokeLigaments.push_back(spoke);
            }
        }
        
        void SetAngle(units::degree_t angle);
        void SetCircleColor(const frc::Color8Bit& color);
        void SetIndicatorColor(const frc::Color8Bit& color);

    private:
        std::vector<frc::MechanismLigament2d*> _spokeLigaments;
};