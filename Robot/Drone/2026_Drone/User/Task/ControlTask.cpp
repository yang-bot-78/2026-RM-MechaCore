#include "ControlTask.hpp"

extern "C" void Control(void const * argument)
{
    for(;;)
    {
        osDelay(1);
    }    
}