#pragma once

#include <cstdint>
#include <sys/types.h>
namespace rmcs_msgs {

struct GameRobotHp {
    uint16_t red_1;
    uint16_t red_2;
    uint16_t red_3;
    uint16_t red_4;
    uint16_t red_5;
    uint16_t red_7;
    uint16_t red_outpost;
    uint16_t red_base;
    uint16_t blue_1;
    uint16_t blue_2;
    uint16_t blue_3;
    uint16_t blue_4;
    uint16_t blue_5;
    uint16_t blue_7;
    uint16_t blue_outpost;
    uint16_t blue_base;
};
} // namespace rmcs_msgs