#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/HAL/CAN/can_hal.hpp"
#include "ControlTask.hpp"

extern ALG::PID::PID yaw_angle_pid;
extern ALG::PID::PID yaw_velocity_pid;

#endif // !MOTORTASK_HPP