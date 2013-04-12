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
 * 
 * The infrared LED camera remote control module.
 * The control infrared LED should be connected on the IR_LED_GPIO over 470 Ohm resistance
 *
 * Currently the NEX5 cameras for STM32 architecture version only
 * 
 */

#include "cam_ir_rc/cam_ir_rc.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#define NEX_START_CMD     0
#define NEX_COMMAND_CMD   1
#define NEX_ADDR_CMD      2
#define NEX_DELAY         11

uint8_t  ir_shot;
uint8_t  bit_cnt;
uint16_t packet_cnt, cmd_cnt, pulse_cnt;
uint32_t delay_time;
uint16_t address = NEX_ADDR;

void cam_ir_rc_init() {
  IR_PORT_INIT();
  IR_LED_OFF();
  SysTimeTimerStart(delay_time);
}

/*
void cam_ir_rc_periodic() {

  static uint16_t shot_pause_cnt;
  if(shot_pause_cnt++ >= 4096) {
    ir_shot = NEX_SHOT;//NEX_VIDEO;
    shot_pause_cnt = 0;
  }
}
*/

#define PAUSE_US(_pause) (SysTimeTimer(delay_time) >= _pause)

#define PULSE_NEX(_cnt) if(PAUSE_US(NEX_DELAY)) {         \
                          if(!IR_LED_GET()) IR_LED_ON();  \
                          else {IR_LED_OFF(); _cnt++;}    \
                          SysTimeTimerStart(delay_time);  \
                        }

void cam_ir_rc_event(void) {

  if(ir_shot) {
    if(packet_cnt < 5) {
      switch(cmd_cnt) {
        case NEX_START_CMD:
          if(pulse_cnt < 96) {
            PULSE_NEX(pulse_cnt);
          }
          else 
            if(PAUSE_US(640 + NEX_DELAY)) {
              cmd_cnt++;
              pulse_cnt = 0;
              bit_cnt = 0;
            }
        break;
        case NEX_COMMAND_CMD:
          if(bit_cnt < 7) {
            if(pulse_cnt < ((ir_shot & (1 << bit_cnt)) ? 48 : 24)) {
              PULSE_NEX(pulse_cnt);
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
        case NEX_ADDR_CMD:
          if(bit_cnt < 13) {
            if(pulse_cnt < ((address & (1 << bit_cnt)) ? 48 : 24)) {
              PULSE_NEX(pulse_cnt);
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
      ir_shot = 0;
      packet_cnt = 0;
      cmd_cnt = 0;
      pulse_cnt = 0;
      bit_cnt = 0;
    }
  }

}

