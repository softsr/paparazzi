/*
 * $Id: $
 *
 * Copyright (C) 2012 Sergey Krukowski <softsr@yahoo.de>
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

#include "cam_control/krooz_cam.h"
#include "cam_control/cam_power.h"
#include "state.h"
#include "subsystems/commands.h"
//#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "generated/flight_plan.h"
#include "subsystems/radio_control.h"
#include "subsystems/navigation/common_nav.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"

#ifndef TILT_COEFF
#define TILT_COEFF 500
#endif
#ifndef PAN_COEFF
#define PAN_COEFF 500
#endif
#ifndef TILT_RATE
#define TILT_RATE 50
#endif
#ifndef PAN_RATE
#define PAN_RATE 50
#endif
#ifndef PAN_CENTER
#define PAN_CENTER 1500
#endif
#ifndef TILT_CENTER
#define TILT_CENTER 1500
#endif
#ifndef PAN_AREA
#define PAN_AREA 300
#endif
#ifndef PAN_DOWN
#define PAN_DOWN 1800
#endif

bool_t cam_flag;
bool_t servo_flag;
#ifdef CAM_SETUP
int16_t tilt_center = TILT_CENTER;
int16_t tilt_coeff = TILT_COEFF;
int16_t pan_coeff = PAN_COEFF;
int16_t tilt_rate = TILT_RATE;
int16_t pan_rate = PAN_RATE;
int16_t pan_down = PAN_DOWN;
int16_t pan_area = PAN_AREA;
#endif
int16_t cam_tilt = 0;
int16_t cam_pan = 0;
int32_t radio_cam_value = 0;

void krooz_cam_init() {
  ActuatorsPwmInit();
}

void krooz_cam_periodic() {
  cam_flag = TRUE;
}

void krooz_servo_periodic() {
  servo_flag = TRUE;
}

void krooz_cam_event(void) {
  if(servo_flag) {
#ifdef CAM_SETUP
    int32_t angle_buf = stateGetNedToBodyEulers_i()->phi * tilt_coeff / INT32_ANGLE_PI;
    int32_t rate_buf = stateGetBodyRates_i()->p * tilt_rate/INT32_ANGLE_PI;
    cam_tilt = (int16_t)angle_buf + (int16_t)rate_buf + tilt_center;
    angle_buf = stateGetNedToBodyEulers_i()->theta * pan_coeff/INT32_ANGLE_PI;
    rate_buf = stateGetBodyRates_i()->q * pan_rate / INT32_ANGLE_PI;
#else
    int32_t angle_buf = stateGetNedToBodyEulers_i()->phi * TILT_COEFF / INT32_ANGLE_PI;
    int32_t rate_buf = stateGetBodyRates_i()->p * TILT_RATE / INT32_ANGLE_PI;
    cam_tilt = (int16_t)angle_buf + (int16_t)rate_buf + TILT_CENTER;
    angle_buf = stateGetNedToBodyEulers_i()->theta * PAN_COEFF/INT32_ANGLE_PI;
    rate_buf = stateGetBodyRates_i()->q * PAN_RATE / INT32_ANGLE_PI;
#endif
    if(radio_control.status == RC_OK)
      radio_cam_value = (radio_cam_value*49 + (int32_t)radio_control.values[RADIO_CAM]) / 50;
    if(autopilot_mode != AP_MODE_NAV)
#if CAM_SETUP == 1
      cam_pan = (int16_t)angle_buf + (radio_cam_value - MAX_PPRZ)*pan_area/MAX_PPRZ + (int16_t)rate_buf + pan_down;
#else
      cam_pan = (int16_t)angle_buf + (radio_cam_value - MAX_PPRZ)*PAN_AREA/MAX_PPRZ + (int16_t)rate_buf + PAN_DOWN;
#endif
    else
#if CAM_SETUP == 1
      cam_pan = (int16_t)angle_buf + pan_down + (int16_t)rate_buf;
#else
      cam_pan = (int16_t)angle_buf + PAN_DOWN + (int16_t)rate_buf;
#endif
#ifdef CAM_TEST
    cam_pan = (int16_t)angle_buf + radio_cam_value*600/MAX_PPRZ + (int16_t)rate_buf + PAN_CENTER;
#endif
/*
#ifdef COMMAND_CAM_PAN
    commands[COMMAND_CAM_PAN] = cam_pan;
#endif
#ifdef COMMAND_CAM_TILT
    /commands[COMMAND_CAM_TILT] = cam_tilt;
#endif
*/  
    Bound(cam_tilt, 900, 2100);
    Bound(cam_pan, 900, 2100);
    ActuatorPwmSet(8, cam_pan);
    ActuatorPwmSet(9, cam_tilt);
    ActuatorsPwmCommit();
    servo_flag = FALSE;
  }
  
  if(cam_flag) {
    periodic_task_kamera();
    cam_flag = FALSE;
  }
}

