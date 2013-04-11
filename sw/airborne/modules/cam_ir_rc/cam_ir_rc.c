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

#include "cam_ir_rc/cam_ir_rc.h"
/*
#include "state.h"
#include "subsystems/commands.h"
#include "subsystems/ins.h"
#include "generated/flight_plan.h"
#include "subsystems/radio_control.h"
#include "subsystems/navigation/common_nav.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"
*/
#include "mcu_periph/sys_time.h"
#include "led.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>

#define IR_LED_GPIO_PORT   GPIOB
#define IR_LED_GPIO_PIN    GPIO6

#define IR_LED_ON() gpio_set(IR_LED_GPIO_PORT, IR_LED_GPIO_PIN)
#define IR_LED_OFF() gpio_clear(IR_LED_GPIO_PORT, IR_LED_GPIO_PIN)
#define IR_LED_TOGGLE() gpio_toggle(IR_LED_GPIO_PORT, IR_LED_GPIO_PIN)
#define IR_LED_GET() gpio_get(IR_LED_GPIO_PORT, IR_LED_GPIO_PIN)

                            /* IR commands for Sony NEX-5N:                                 */
#define NEX_DELAY 11
#define IRCMD_ADDRESS    0x1E3A       /* IR command address                                 */
#define IRCMD_SHUTTER    0x2D         /* IR command for immediate shutter activation        */
#define IRCMD_SHUTTER2S  0x37         /* IR command for shutter activation with 2 sec delay */
#define IRCMD_VIDON      0x48         /* IR command for video (de)activation                */
#define IRCMD_VIDOFF     0x48         /* IR command for video (de)activation                */

uint8_t  shot;
uint8_t  bit_cnt;
uint16_t shot_pause_cnt;
uint16_t packet_cnt, cmd_cnt, pulse_cnt;

uint32_t delay_time;
uint16_t mask;                                             /* mask for only one bit to send */
uint16_t address = IRCMD_ADDRESS;                          /* camera command                */

void cam_ir_rc_init() {
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
  gpio_mode_setup(IR_LED_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, IR_LED_GPIO_PIN);
	gpio_set_output_options(IR_LED_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, IR_LED_GPIO_PIN);
  IR_LED_OFF();
  
  SysTimeTimerStart(delay_time);
}

/**/
void cam_ir_rc_periodic() {
  //IR_LED_TOGGLE();
  if(shot_pause_cnt++ >= 4096) {
    shot = IRCMD_SHUTTER;//IRCMD_VIDON;
    shot_pause_cnt = 0;
  }
}

#define PULSE_40k(_cnt) if(SysTimeTimer(delay_time) >= NEX_DELAY) { \
                          if(!IR_LED_GET()) IR_LED_ON(); \
                          else {IR_LED_OFF(); _cnt++;} \
                          SysTimeTimerStart(delay_time); \
                        }

#define PAUSE_US(_pause) (SysTimeTimer(delay_time) >= _pause)

void cam_ir_rc_event(void) {

  if(shot) {
    if(packet_cnt < 5) {
      switch(cmd_cnt) {
        case 0:
          if(pulse_cnt < 96) {
            PULSE_40k(pulse_cnt);
          }
          else 
            if(PAUSE_US(640 + NEX_DELAY)) {
              cmd_cnt++;
              pulse_cnt = 0;
              bit_cnt = 0;
            }
        break;
        case 1:
          if(bit_cnt < 7) {
            if(pulse_cnt < ((shot & (1 << bit_cnt)) ? 48 : 24)) {
              PULSE_40k(pulse_cnt);
            }
            else 
              if(PAUSE_US(640 + NEX_DELAY)) {
                pulse_cnt = 0;
                bit_cnt++;
              }
          }
          else {
            cmd_cnt++;
            pulse_cnt = 0;
            bit_cnt = 0;
          }
        break;
        case 2:
          if(bit_cnt < 13) {
            if(pulse_cnt < ((address & (1 << bit_cnt)) ? 48 : 24)) {
              PULSE_40k(pulse_cnt);
            }
            else 
              if(PAUSE_US(640 + NEX_DELAY)) {
                pulse_cnt = 0;
                bit_cnt++;
              }
          }
          else 
            if(PAUSE_US(11000)) {
              packet_cnt++;
              cmd_cnt = 0;
              pulse_cnt = 0;
              bit_cnt = 0;
            }
        break;
        default:
        break;
      }
    }
    else {
      shot = 0;
      packet_cnt = 0;
      cmd_cnt = 0;
      pulse_cnt = 0;
      bit_cnt = 0;
      LED_TOGGLE(3)
    }
  }

}

