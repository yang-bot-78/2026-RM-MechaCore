#include "Callback.hpp"
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) 
    {
        HAL::UART::Data rx_data{dr16_rx_buffer, sizeof(dr16_rx_buffer)};
            // 获取UART实例
        auto& uart1= HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        
        if(huart == uart1.get_handle())
        {
            uart1.receive_dma_idle(rx_data);
            uart1.trigger_rx_callbacks(rx_data);
        }
    }   

}