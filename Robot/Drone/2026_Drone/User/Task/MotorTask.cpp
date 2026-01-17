#include "MotorTask.hpp"

BSP::Motor::Dji::GM6020<1> Motor6020(0x204,{2},0x1FF);
BSP::Motor::DM::J4310<1> MotorJ4310(0x04, {1},{0x11});

void MotorTaskInit()
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can1.register_rx_callback([](HAL::CAN::Frame frame)
    {
        Motor6020.Parse(frame);
    });
    // can2.register_rx_callback([](HAL::CAN::Frame frame)
    // {
    //     MotorJ4310.Parse(frame);
    // });
}

static void motor_control_logic()
{
    Motor6020.setCAN(static_cast<int16_t>(gimbal_output.out_yaw), 2);
    Motor6020.sendCAN();
}

extern "C" void Motor(void const * argument)
{
    MotorTaskInit();
    for(;;)
    {
        motor_control_logic();
        osDelay(1);
    }    
}