#include <stdbool.h>
#include "delay.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

void delay_ms(uint32_t ms) {
  while (ms--) {
    SysCtlDelay(SysCtlClockGet() / (1000 / 3));
  }
}  

void delay_us(uint32_t us) {
  while (us--) {
    SysCtlDelay(SysCtlClockGet() / (1000000 / 3));
  }
}
