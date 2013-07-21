/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main.c
 *
 * Rotorcraft main loop.
 */

#define MODULES_C

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "firmwares/rtos/main.h"

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

tid_t main_periodic_tid; ///< id for main_periodic() timer

int main( void ) {
  main_init();

  while(1) {
    handle_periodic_tasks();
    main_event();
  }
  return 0;
}

STATIC_INLINE void main_init( void ) {

  mcu_init();

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
}

STATIC_INLINE void handle_periodic_tasks( void ) {
  if (sys_time_check_and_ack_timer(main_periodic_tid))
    main_periodic();
}

STATIC_INLINE void main_periodic( void ) {

  RunOnceEvery(10, LED_PERIODIC());
}

STATIC_INLINE void main_event( void ) {

}
