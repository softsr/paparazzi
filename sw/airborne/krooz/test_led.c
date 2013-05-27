/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Damjan Marion <damjan.marion@gmail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mcu.h>
//#include <libopencm3/stm32/f4/gpio.h>
#include "led.h"

	/* Enable GPIOD clock. */
	//rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);

/*
void gpio_setup(void)
{
	// Set GPIO12-15 (in GPIO port D) to 'output push-pull'.
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}
*/
int main(void)
{
	int i;

	mcu_init();

	// Set two LEDs for wigwag effect when toggling.
	//gpio_toggle(GPIOD, GPIO12 | GPIO14);
	LED_TOGGLE(1); LED_TOGGLE(3);

	// Blink the LEDs (PD12, PD13, PD14 and PD15) on the board.
	while (1) {
		// Toggle LEDs.
		//gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
		LED_TOGGLE(2); LED_TOGGLE(3); 
		for (i = 0; i < 12000000; i++) // Wait a bit.
			__asm__("nop");
	}

	return 0;
}
