/** 
 *     This file is part of dqbot.
 *  
 *     dqbot is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dqbot is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dqbot. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file examples/example1.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include <array>
#include "SerialManipulator.hpp"

int main() {
    std::array<std::array<float, 6>, 5> dh {
        std::array<float, 6>{0,       M_PI,      -M_PI_2,       0,      0,   0},
        std::array<float, 6>{0.1735,  0,         0,       -0.1182,  0,      -0.264},
        std::array<float, 6>{0.027,   0.1629,   0.055,    0,      0,      0},
        std::array<float, 6>{-M_PI_2,     M_PI,      -M_PI_2,       M_PI_2,    -M_PI_2,     M_PI},
        std::array<float, 6>{0,       0,      0,       0,      0,   0}
    };

    std::array<std::array<float, 6>, 4> limits {
        std::array<float, 6>{-175,     0,        0,       -175,    -120,    -175},
        std::array<float, 6>{175,     175,      150,      175,     120,     175},
        std::array<float, 6>{-20,     -20,      -40,      -40,     -40,     -60},
        std::array<float, 6>{20,      20,       40,       40,      40,      60}
    };
    for (int i=0; i<6; ++i) {{
        limits[0][i] = limits[0][i] / 180. * M_PI;
        limits[1][i] = limits[1][i] / 180. * M_PI;
        limits[2][i] = limits[2][i] / 180. * M_PI;
        limits[3][i] = limits[3][i] / 180. * M_PI;
    }}

    std::array<float, 6> joint_positions {0,0,0,0,0,0};

    dqkinematics::SerialManipulator<float, 6> robot(dh, limits, joint_positions);

    dqpose::Posef x_init = robot.end_pose();
    dqpose::Tranf t_init = x_init.translation();
    dqpose::Rotf r_init = x_init.rotation();
    float radius = 0.01;
    float rad_speed = 0.001;
    size_t i = 0;

    while (true)
    {   
        dqpose::Tranf offset = t_init + dqpose::Tranf(-0.02, radius * cos(rad_speed*i), radius * sin(rad_speed*i) + 0.05);
        ++i;
        dqpose::Posef xd = dqpose::Posef::build_from(r_init, offset);

        robot.update(xd);

        // std::cout << robot.end_pose().rotation() << "\n" << xd.rotation() << "\n";
    }

}             