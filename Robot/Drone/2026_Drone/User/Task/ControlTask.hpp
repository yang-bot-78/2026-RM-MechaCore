#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/SerivalTask.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../User/core/Alg/PID/pid.hpp"

extern Output_gimbal gimbal_output;

extern BSP::Motor::DM::J4310<1> MotorJ4310;
extern BSP::Motor::Dji::GM6020<1> Motor6020;

extern BSP::REMOTE_CONTROL::RemoteController DR16;

#endif // !1