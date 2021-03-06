/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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

/**
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot.
 *
 */

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"
#include "subsystems/gps.h"
#include "subsystems/commands.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/ins.h"
#include "led.h"

uint8_t  autopilot_mode;
uint8_t  autopilot_mode_auto2;

bool_t   autopilot_in_flight;
uint32_t autopilot_in_flight_counter;
uint16_t autopilot_flight_time;

bool_t   autopilot_motors_on;
bool_t   kill_throttle;

bool_t   autopilot_rc;
bool_t   autopilot_power_switch;

bool_t   autopilot_detect_ground;
bool_t   autopilot_detect_ground_once;

#define AUTOPILOT_IN_FLIGHT_TIME    40

#ifndef AUTOPILOT_DISABLE_AHRS_KILL
#include "subsystems/ahrs.h"
static inline int ahrs_is_aligned(void) {
  return (ahrs.status == AHRS_RUNNING);
}
#else
static inline int ahrs_is_aligned(void) {
  return TRUE;
}
#endif

#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
#include "autopilot_arming_switch.h"
#elif USE_THROTTLE_FOR_MOTOR_ARMING
#include "autopilot_arming_throttle.h"
#else
#include "autopilot_arming_yaw.h"
#endif

#ifdef USE_RC_FP_BLOCK_SWITCHING
#include "autopilot_fp_block_switch.h"
#endif

void autopilot_init(void) {
  autopilot_mode = AP_MODE_KILL;
  autopilot_motors_on = FALSE;
  kill_throttle = ! autopilot_motors_on;
  autopilot_in_flight = FALSE;
  autopilot_in_flight_counter = 0;
  autopilot_mode_auto2 = MODE_AUTO2;
  autopilot_detect_ground = FALSE;
  autopilot_detect_ground_once = FALSE;
  autopilot_flight_time = 0;
  autopilot_rc = TRUE;
  autopilot_power_switch = FALSE;
#ifdef POWER_SWITCH_LED
  LED_ON(POWER_SWITCH_LED); // POWER OFF
#endif
  autopilot_arming_init();
}


void autopilot_periodic(void) {

  RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
#ifdef FAILSAFE_GROUND_DETECT
  if ((autopilot_mode == AP_MODE_FAILSAFE
      || autopilot_mode == AP_MODE_HOVER_Z_HOLD
      || autopilot_mode == AP_MODE_NAV) 
      && autopilot_detect_ground) {
    autopilot_set_mode(AP_MODE_KILL);
    autopilot_detect_ground = FALSE;
  }
#endif

  /* set failsafe commands, if in FAILSAFE or KILL mode */
#ifndef FAILSAFE_GROUND_DETECT
  if (autopilot_mode == AP_MODE_KILL ||
      autopilot_mode == AP_MODE_FAILSAFE) {
#else
  if (autopilot_mode == AP_MODE_KILL) {
#endif
    SetCommands(commands_failsafe);
  }
  else {
    guidance_v_run( autopilot_in_flight );
    guidance_h_run( autopilot_in_flight );
    SetRotorcraftCommands(stabilization_cmd, autopilot_in_flight, autopilot_motors_on);
  }

}


void autopilot_set_mode(uint8_t new_autopilot_mode) {

  /* force kill mode as long as AHRS is not aligned */
  if (!ahrs_is_aligned())
    new_autopilot_mode = AP_MODE_KILL;

  //if (new_autopilot_mode != autopilot_mode) 
  if (new_autopilot_mode == autopilot_mode) {
    if(!autopilot_motors_on)
      rc_command = 0;
    return;
  }
  rc_command = RC_MODE_CHANGE;
  {
    /* horizontal mode */
    switch (new_autopilot_mode) {
    case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
      stab_att_sp_euler.phi = 0;
      stab_att_sp_euler.theta = 0;
      guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
      break;
#endif
    case AP_MODE_KILL:
      autopilot_set_motors_on(FALSE);
      autopilot_in_flight = FALSE;
      autopilot_in_flight_counter = 0;
      guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
      break;
    case AP_MODE_RC_DIRECT:
      guidance_h_mode_changed(GUIDANCE_H_MODE_RC_DIRECT);
      break;
    case AP_MODE_RATE_DIRECT:
    case AP_MODE_RATE_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
      break;
    case AP_MODE_ATTITUDE_DIRECT:
    case AP_MODE_ATTITUDE_CLIMB:
    case AP_MODE_ATTITUDE_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
      break;
    case AP_MODE_HOVER_DIRECT:
    case AP_MODE_HOVER_CLIMB:
    case AP_MODE_HOVER_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
      break;
    case AP_MODE_NAV:
      guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
      break;
    default:
      break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
    case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
      guidance_v_zd_sp = SPEED_BFP_OF_REAL(0.5);
      guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
      break;
#endif
    case AP_MODE_KILL:
      guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
      break;
    case AP_MODE_RC_DIRECT:
    case AP_MODE_RATE_DIRECT:
    case AP_MODE_ATTITUDE_DIRECT:
    case AP_MODE_HOVER_DIRECT:
      guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
      break;
    case AP_MODE_RATE_RC_CLIMB:
    case AP_MODE_ATTITUDE_RC_CLIMB:
      guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
      break;
    case AP_MODE_ATTITUDE_CLIMB:
    case AP_MODE_HOVER_CLIMB:
      guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
      break;
    case AP_MODE_RATE_Z_HOLD:
    case AP_MODE_ATTITUDE_Z_HOLD:
    case AP_MODE_HOVER_Z_HOLD:
      guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
      break;
    case AP_MODE_NAV:
      guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
      break;
    default:
      break;
    }
    autopilot_mode = new_autopilot_mode;
  }

}

#ifndef AP_IN_FLIGHT_MIN_SPEED
#define AP_IN_FLIGHT_MIN_SPEED SPEED_BFP_OF_REAL(0.2)
#endif
#ifndef AP_IN_FLIGHT_MIN_ACCEL
#define AP_IN_FLIGHT_MIN_ACCEL ACCEL_BFP_OF_REAL(2.0)
#endif

static inline void autopilot_check_in_flight( bool_t motors_on ) {
  static int32_t accel_filter;
  if (autopilot_in_flight) {
    if(!motors_on) {
      autopilot_in_flight = FALSE;
      autopilot_in_flight_counter = 0;
      return;
    }
    if (autopilot_in_flight_counter > 0) {
      if (THROTTLE_STICK_DOWN() && ((autopilot_mode == AP_MODE_HOVER_Z_HOLD
                                     && (abs(ins_ltp_speed.z) < AP_IN_FLIGHT_MIN_SPEED)
                                     && (((accel_filter*9 + abs(ins_ltp_accel.z))/10) < AP_IN_FLIGHT_MIN_ACCEL))
                                   || (autopilot_mode != AP_MODE_NAV && autopilot_mode != AP_MODE_HOVER_Z_HOLD))) {
        autopilot_in_flight_counter--;
        if (autopilot_in_flight_counter == 0) {
          autopilot_in_flight = FALSE;
        }
      }
      else {  /* !THROTTLE_STICK_DOWN */
        autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
      }
    }
  }
  else { /* not in flight */
    if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
        motors_on) {
      if ((autopilot_mode != AP_MODE_HOVER_Z_HOLD && autopilot_mode != AP_MODE_NAV && !THROTTLE_STICK_DOWN()) ||
          (autopilot_mode == AP_MODE_HOVER_Z_HOLD && THROTTLE_STICK_CENTER_UP()) || 
          (autopilot_mode == AP_MODE_NAV && THROTTLE_STICK_UP())) {
        autopilot_in_flight_counter++;
        if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
          autopilot_in_flight = TRUE;
      }
      else { /*  THROTTLE_STICK_DOWN */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

static inline void autopilot_check_ground_detect_on( bool_t motors_on ) {
  if(autopilot_mode == AP_MODE_HOVER_Z_HOLD && THROTTLE_STICK_DOWN() && !autopilot_detect_ground_once && stateGetPositionEnu_f()->z < 3.0 && motors_on)
    autopilot_detect_ground_once = TRUE;
  else
    autopilot_detect_ground_once = FALSE;
}

void autopilot_set_motors_on(bool_t motors_on) {
  if (ahrs_is_aligned() && motors_on)
    autopilot_motors_on = TRUE;
  else
    autopilot_motors_on = FALSE;
  kill_throttle = ! autopilot_motors_on;
  autopilot_arming_set(autopilot_motors_on);
}


void autopilot_on_rc_frame(void) {

  if (kill_switch_is_on())
    autopilot_set_mode(AP_MODE_KILL);
  else {
    uint8_t new_autopilot_mode = 0;
    AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode);
    /* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
    if (!(new_autopilot_mode == AP_MODE_NAV && GpsIsLost()))
      autopilot_set_mode(new_autopilot_mode);
  }

  /* if not in FAILSAFE mode check motor and in_flight status, read RC */
  if (autopilot_mode > AP_MODE_FAILSAFE) {

    /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
    SetAutoCommandsFromRC(commands, radio_control.values);
#endif

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    if (autopilot_mode != AP_MODE_NAV) {
      SetCommandsFromRC(commands, radio_control.values);
    }
#endif

    /* an arming sequence is used to start/stop motors */
    autopilot_arming_check_motors_on();
    kill_throttle = ! autopilot_motors_on;

    autopilot_check_in_flight(autopilot_motors_on);
#ifdef USE_RC_FP_BLOCK_SWITCHING
    autopilot_block_switch_check(autopilot_in_flight);
#endif

    autopilot_check_ground_detect_on(autopilot_in_flight);
    
    guidance_v_read_rc();
    guidance_h_read_rc(autopilot_in_flight);
  }

}
