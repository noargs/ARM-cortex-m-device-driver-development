
#include "stm32f407xx.h"

int main(void) {

	return 0;
}

// this is ISR implementation and will override
// .weak EXT10_IRQHanlder() declared in startup file
// @ "../Startup/startup_stm32f407vgtx.s"
void EXT10_IRQHandler(void) {
	// handle the interrupt
	GPIO_IRQHandling(0);
}
