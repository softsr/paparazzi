/*
 * $Id: $
 *
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef CAM_IR_RC_H
#define CAM_IR_RC_H

#include "generated/airframe.h"
#include "std.h"
#include "led_hw.h"

#ifndef IR_PORT
#define IR_PORT   B
#endif
#ifndef IR_PIN
#define IR_PIN    6
#endif

#if ARCH==lpc21

#endif

#if ARCH==stm32
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>

#define _IR_GPIO_RCC(_port)     RCC_AHB1ENR_IOP ## _port ## EN
#define IR_GPIO_RCC(_port)      _IR_GPIO_RCC(_port)
#define _IR_GPIO_PORT(_port)     GPIO ## _port
#define IR_GPIO_PORT(_port)     _IR_GPIO_PORT(_port)
#define _IR_GPIO_PIN(_pin)       GPIO ## _pin
#define IR_GPIO_PIN(_pin)       _IR_GPIO_PIN(_pin)

#define IR_PORT_INIT() {                                                                                      \
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, IR_GPIO_RCC(IR_PORT));                                            \
  gpio_mode_setup(IR_GPIO_PORT(IR_PORT), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, IR_GPIO_PIN(IR_PIN));              \
	gpio_set_output_options(IR_GPIO_PORT(IR_PORT), GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, IR_GPIO_PIN(IR_PIN));       \
  }

#define IR_LED_ON() gpio_set(IR_GPIO_PORT(IR_PORT), IR_GPIO_PIN(IR_PIN))
#define IR_LED_OFF() gpio_clear(IR_GPIO_PORT(IR_PORT), IR_GPIO_PIN(IR_PIN))
#define IR_LED_TOGGLE() gpio_toggle(IR_GPIO_PORT(IR_PORT), IR_GPIO_PIN(IR_PIN))
#define IR_LED_GET() gpio_get(IR_GPIO_PORT(IR_PORT), IR_GPIO_PIN(IR_PIN))

#endif

#define NEX_ADDR       0x1E3A
#define NEX_SHOT       0x2D
#define NEX_SHOT2S     0x37
#define NEX_VIDEO      0x48

extern void cam_ir_rc_init(void);
//extern void cam_ir_rc_periodic(void);
extern void cam_ir_rc_event(void);

extern uint8_t ir_shot;

#endif /* CAM_IR_RC_H */

