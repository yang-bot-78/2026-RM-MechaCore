#include "MotorTask.hpp"

extern "C" void Motor(void const * argument)
{
    for(;;)
    {
        osDelay(1);
    }    
}

