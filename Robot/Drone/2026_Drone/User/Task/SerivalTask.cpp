#include "SerivalTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DR16;
uint8_t dr16_rx_buffer[18];

void serivalInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    HAL::UART::Data dr16_rx_data{dr16_rx_buffer, sizeof(dr16_rx_buffer)};
    uart1.receive_dma_idle(dr16_rx_data);
    uart1.register_rx_callback([](HAL::UART::Data data)
    {
        if(data.size >= 18 && data.buffer!= nullptr)
        {
        DR16.parseData(data.buffer);
        }
    });
}

extern "C" void Serial(void const * argument)
{
    serivalInit();
    for(;;)
    {
        //vofa_send(gimbal_target.target_yaw,gimbal_target.normalized_feedback,0.0f,0.0f,0.0f,0.0f);
        osDelay(1);
    }
}