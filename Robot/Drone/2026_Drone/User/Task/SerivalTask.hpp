#ifndef SERIVALTASK_HPP
#define SERIVALTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/BSP/RemoteControl/DT7.hpp"
#include "../core/HAL/UART/uart_hal.hpp"
#include "../core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../core/BSP/Motor/DM/DmMotor.hpp"

extern BSP::REMOTE_CONTROL::RemoteController DR16;
extern uint8_t dr16_rx_buffer[18];

#endif // DEBUG