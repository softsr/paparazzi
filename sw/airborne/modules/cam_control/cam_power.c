
#include "cam_power.h"
#include "subsystems/gps.h"
#include "subsystems/radio_control/ppm.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"
#include "messages.h"
#include "state.h"
#include "subsystems/ahrs.h"
#include "subsystems/datalink/datalink.h"

#include "subsystems/radio_control.h"

#include "mcu_periph/adc.h"
#include "std.h"

#include "subsystems/datalink/downlink.h"
#ifdef SITL
#include "printf.h"
#endif

#ifndef SHOT_PERIOD
#define SHOT_PERIOD 4
#endif

#ifndef STAGE_PHOTO_NUM
#define STAGE_PHOTO_NUM 3
#endif

#define CAM_OFF_DELAY      10

uint16_t dc_photo_nr = 1;
uint16_t last_known_shot = 0;

bool_t use_uav_agri = 0;
bool_t   cam_switch;
uint16_t cam_counter = 8;
#ifdef VIDEO_1
int16_t  video_counter = -1;
#endif

bool_t on = FALSE;
bool_t    cam_on;
bool_t     cam_alt_on;
int16_t     cam_climb_on = 0;
bool_t     do_shots;
bool_t    do_nav_shots;
bool_t     shot_video;
bool_t     video_on;

uint8_t    stage_photo_cnt = 0;
uint8_t    stage_photo_num;
uint8_t    shot_period;
uint8_t   cam_mode;


//Mindesthoehe in cm
#ifndef CAM_MIN_ALT
#define CAM_MIN_ALT 300
#endif

int32_t cam_min_alt = CAM_MIN_ALT;

#ifdef ADC_CHANNEL_CAM1
struct adc_buf cam1_state;
int32_t cam1_state_val;
#endif
#ifdef CAMERA_2
#ifdef ADC_CHANNEL_CAM2
struct adc_buf cam2_state;
#endif
#endif

#ifdef SITL
int32_t IO0CLR, IO0DIR, IO0SET, IO0PIN;
#endif

#define CLEAR_TIME   2
#define CAM_LAG      4

typedef struct
{
  uint16_t    photo_nr;
  int32_t     gps_lat;
  int32_t     gps_lon;
  float       gps_z;
  uint8_t     gps_utm_zone;
  int16_t     phi;
  int16_t     theta;
  int16_t     gps_course;
  uint16_t    gps_gspeed;
  uint32_t    gps_itow;
} T_DC_SHOT;

#define SHOT_BUF_LEN 50
T_DC_SHOT buffer[SHOT_BUF_LEN];
int buffer_pos = 0;

void CacheDel(uint16_t photo_nr)
{
  while(buffer_pos > 0 && photo_nr >= buffer[0].photo_nr)
  {
    buffer_pos--;
    for(uint16_t i = 0; i < buffer_pos; i++)
      buffer[i] = buffer[i+1];
  }
}


static void CacheAdd(uint16_t photo_nr)
{
  if(buffer_pos < SHOT_BUF_LEN)
  {
    buffer[buffer_pos].photo_nr   = photo_nr;
    buffer[buffer_pos].gps_lat    = DegOfRad(gps.lla_pos.lat);
    buffer[buffer_pos].gps_lon    = DegOfRad(gps.lla_pos.lon);
    buffer[buffer_pos].gps_z      = ((float)(stateGetPositionEnu_i()->z * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM)) / 100.0f;
    buffer[buffer_pos].gps_utm_zone = gps.utm_pos.zone;
    buffer[buffer_pos].phi        = 0; // Degree*10
    buffer[buffer_pos].theta      = 0; // Degree*10
    buffer[buffer_pos].gps_course = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi*10));
    buffer[buffer_pos].gps_gspeed = gps.gspeed;
    buffer[buffer_pos].gps_itow   = gps.tow;
    buffer_pos++;
  }
  else  
  { //Puffer voll, sofort senden
    int32_t lat = DegOfRad(gps.lla_pos.lat);
    int32_t lon = DegOfRad(gps.lla_pos.lon);
    int16_t phi = 0; // Degree*10
    int16_t theta = 0; // Degree*10
    int16_t course = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi * 10));
    float gps_z = ((float)(stateGetPositionEnu_i()->z * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM)) / 100.0f;
    DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice, &photo_nr, &lat, &lon, &gps_z, &gps.utm_pos.zone, &phi, &theta,  &course, &gps.gspeed, &gps.tow);
  }
}

#define DATALINK_TIME 1
#define DataLink_Valid (datalink_time <= DATALINK_TIME)

static void shot(void)
{
  CamClr();
  if(!use_uav_agri) //Sofort senden
  {
    int32_t lat = DegOfRad(gps.lla_pos.lat);
    int32_t lon = DegOfRad(gps.lla_pos.lon);
    int16_t phi = 0; // Degree*10
    int16_t theta = 0; // Degree*10
    float gps_z = ((float)(stateGetPositionEnu_i()->z * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM)) / 100.0f;
    int16_t course = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi * 10));
    DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice, &dc_photo_nr, &lat, &lon, &gps_z, &gps.utm_pos.zone, &phi, &theta,  &course, &gps.gspeed, &gps.tow);
  }
  else
    CacheAdd(dc_photo_nr);

  dc_photo_nr++;
  stage_photo_cnt++;
}

#ifdef SITL
#define GPS_VALID TRUE
#else
#define GPS_VALID GpsFixValid()
#endif

void kamera_check_gps(void)
{
  int32_t pos_z = stateGetPositionEnu_i()->z * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM;
  
  if(GPS_VALID)
  {
    if(cam_min_alt >= 0)
    {
      if((pos_z > (cam_min_alt + 200)) && shot_period > 0) //Einschalten = minimale Höhe + 2m
        cam_alt_on = TRUE;
      else if(pos_z < cam_min_alt) //Ausschalten = minimale Höhe
        cam_alt_on = FALSE;
    }
    if(stateGetSpeedEnu_i()->z * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM < -500) //starker Sinkflug - Kamera 5sec aus
      cam_climb_on = 10;
//    if(pprz_mode != PPRZ_MODE_AUTO1 && pprz_mode != PPRZ_MODE_AUTO2)
//      cam_alt_on = FALSE;
  }
}

static bool_t cam_check_on(bool_t cam_state) {
  bool_t res;
  res = ((cam_state != 0) == (Cam1Check()));
#ifdef CAMERA_2
  res &= ((cam_state != 0) == (Cam2Check()));
#endif
  return res;
}

#define CAM_MODE_TURN_ON         1
#define CAM_MODE_SHOT             2
#define CAM_MODE_TURN_OFF         3
#define CAM_MODE_SHOT_PAUSE       4
#define CAM_MODE_IDLE             5

static bool_t Cam_mode_change(int16_t new_mode) {
  if(new_mode != cam_mode) {
    if(cam_mode == CAM_MODE_IDLE || cam_mode == CAM_MODE_SHOT_PAUSE) {
      cam_mode = new_mode;
      CamSet();
      cam_counter = 0;
      return TRUE;
    }
  }
  return FALSE;
}

void kamera_init( void )
{
  
#ifdef CAM_TEST
  cam_alt_on = TRUE;
  cam_climb_on = 0;
  cam_on = TRUE;
  cam_mode = CAM_MODE_IDLE;
#else
  cam_alt_on = FALSE;
  cam_climb_on = 0;
  //cam_on = FALSE;
  cam_on = TRUE;
  cam_mode = CAM_MODE_IDLE;
  do_shots = TRUE;
  do_nav_shots = FALSE;
#endif
  video_on = FALSE;

#ifdef LPC21
#ifdef ADC_CHANNEL_CAM1
  CAM1_SHOT_IODIR &= ~(1 << CAM1_SHOT_PIN);
  Cam1SwitchClr();
  Cam1VideoClr();
#else
  IO0DIR &= ~CAM_PINS;  //alle 5 Pins als Eingang
  //IO0DIR |= CAM_SHOT; //Auslöser als Ausgang
#endif
#endif

#if defined(STM32F4)
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, CAM_SW_GPIO_CLK);
  gpio_mode_setup(CAM_SW_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CAM_SW_GPIO_PIN);
#endif

  CamSet();
  
  stage_photo_num = STAGE_PHOTO_NUM;
  shot_period = SHOT_PERIOD*4;
#ifdef VIDEO_1
  VideoSet(); //P1.22 auf High
#endif
  cam_counter = 0;
  
#ifdef ADC_CHANNEL_CAM1
  cam1_state.av_nb_sample = DEFAULT_AV_NB_SAMPLE;
  adc_buf_channel(ADC_CHANNEL_CAM1, &cam1_state, DEFAULT_AV_NB_SAMPLE);
#endif
  
}

#define CAM_TURN_ON_TIME       38//16
#define CAM_TURN_OFF_TIME       28
#define CAM_SHOT_TIME           6//4
#define CAM_SHOT_PULSE_TIME    4
#define CAM_PULSE_TIME        6

void periodic_task_kamera(void) //4Hz
{

  cam1_state_val = cam1_state.sum / cam1_state.av_nb_sample;

#ifdef VIDEO_1
  if(video_counter > -5)
    video_counter--;
  if(video_counter == 0)
  {
    VideoClr();
  }
  else if(video_counter == -2)
    VideoSet();
#endif
  
  //cam_counter++;
  
  if(cam_counter > 0)
    cam_counter--;
    
#ifndef CAM_TEST    
  kamera_check_gps();
  
  if(autopilot_mode != AP_MODE_NAV /*|| use_uav_agri || !stage_complete */) {
    //LED_OFF(3)
    cam_on = TRUE;
    //Cam_mode_change(CAM_MODE_SHOT);
  }
  else {
    //cam_on = FALSE;
    //LED_ON(3)
  }
#endif
  
  on = cam_alt_on && (cam_climb_on <= 0) && cam_on;// 
  
  if(cam_mode != CAM_MODE_SHOT) {
    if (!cam_check_on(on)) {
      if(on)
        Cam_mode_change(CAM_MODE_TURN_ON);
      else
        Cam_mode_change(CAM_MODE_TURN_OFF);
      stage_photo_cnt = 0;
      //LED_TOGGLE(3)
    }
    else {
      if(on && cam_mode == CAM_MODE_IDLE && do_shots)
        Cam_mode_change(CAM_MODE_SHOT);
    }
  }
  
  #ifdef VIDEO  
    if(SHOT_VIDEO && autopilot_mode != AP_MODE_NAV)
      shot_video = TRUE;
    else
      shot_video = FALSE;
  #endif
    
  if((DO_SHOTS && autopilot_mode != AP_MODE_NAV) || (do_nav_shots && autopilot_mode == AP_MODE_NAV)) {
    if(!do_shots) {
      //if(Cam_mode_change(CAM_MODE_SHOT))
        if(on) {
          do_shots = TRUE;
          //Cam_mode_change(CAM_MODE_SHOT);
        }
        else
          do_shots = FALSE;
    }
  #ifdef VIDEO
    if(do_shots && shot_video && !video_on) {
      if(on)
        Cam_mode_change(CAM_MODE_SHOT);
    }
  #endif
  }
  else {
    if(do_shots) {
      do_shots = FALSE;
    #ifdef VIDEO
      if(shot_video && video_on)
        Cam_mode_change(CAM_MODE_SHOT);
      else
    #endif  
        Cam_mode_change(CAM_MODE_IDLE);
    }
  #ifdef VIDEO
    if(!do_shots && shot_video && video_on) {
      Cam_mode_change(CAM_MODE_SHOT);
    }
  #endif
  }
  
  switch(cam_mode) {
  
    case CAM_MODE_TURN_ON:
      //LED_TOGGLE(3)
      if(cam_counter <= 0) {
        //LED_ON(3)
        cam_counter = CAM_TURN_ON_TIME;
        //CAM_SWITCH1 als Ausgang
        Cam1SwitchSet()
      #ifdef CAMERA_2
        Cam2SwitchSet()
      #endif
      #ifdef VIDEO
        video_on = FALSE;
      #endif
      }
      else {
        if(cam_counter == CAM_TURN_ON_TIME - CAM_PULSE_TIME)
          Cam1SwitchClr()
        if(cam_counter == 1) {
          if(Cam1Check()) {
            if(do_shots)
              cam_mode = CAM_MODE_SHOT;
            else
              cam_mode = CAM_MODE_IDLE;
          }
        }
      }
      break;
    
    case CAM_MODE_SHOT:
      if(cam_counter <= 0) {
        //LED_OFF(3)
        cam_counter = CAM_SHOT_TIME;
      #ifdef VIDEO
        if(shot_video && autopilot_mode != AP_MODE_NAV) {
          Cam1VideoSet()
          video_on = ~video_on;
        }
        else
      #endif
          shot();
      }
      else {
        if(cam_counter == CAM_SHOT_TIME - CAM_SHOT_PULSE_TIME) {
          //LED_ON(3)
        #ifdef VIDEO
          if(shot_video && autopilot_mode != AP_MODE_NAV)
            Cam1VideoClr()
          else
        #endif
          CamSet();
        }
        if(cam_counter == 1) {
          if((stage_photo_num > stage_photo_cnt) || autopilot_mode == AP_MODE_NAV/* || use_uav_agri*/) {
          //if(stage_photo_num > stage_photo_cnt)
            cam_mode = CAM_MODE_SHOT_PAUSE;
            //if(!use_uav_agri) 
            //  stage_complete = TRUE;
            stage_photo_cnt = 0;
          }
          else
            cam_mode = CAM_MODE_TURN_OFF;//CAM_MODE_IDLE;//
        }
      }
      break;
    
    case CAM_MODE_TURN_OFF:
      //LED_TOGGLE(3)
      if(cam_counter <= 0) {
        cam_counter = CAM_TURN_OFF_TIME;
      #ifdef VIDEO
        video_on = FALSE;
      #endif
      }
      else {
        if(cam_counter == CAM_TURN_OFF_TIME - 12) {
          Cam1SwitchSet()
        #ifdef CAMERA_2
          Cam2SwitchSet();
        #endif
        }
        if(cam_counter == CAM_TURN_OFF_TIME - 12 - CAM_PULSE_TIME) {
          Cam1SwitchClr()
        #ifdef CAMERA_2
          Cam2SwitchClr();
        #endif
        }
        else {
          if(cam_check_on(FALSE)) {
            //stage_complete = TRUE;
            cam_counter = 0;
            cam_mode = CAM_MODE_IDLE;
          #ifdef CAM_TEST
            cam_alt_on = FALSE;
          #endif
          }
        }
      }
      
      break;
    
    case CAM_MODE_SHOT_PAUSE:
      if(cam_counter <= 0) {
        //LED_TOGGLE(3)
        cam_counter = shot_period - CAM_SHOT_TIME;//
        CamSet();
      }
      if(cam_counter == 1) {
        if(do_shots) {
        #ifdef VIDEO
          if(!shot_video || (shot_video && !video_on))
        #endif
          if((autopilot_mode == AP_MODE_NAV/* && stage_complete == FALSE*/) || autopilot_mode != AP_MODE_NAV/* || use_uav_agri*/) 
            cam_mode = CAM_MODE_SHOT;
        }
      } 
      break;
    
    case CAM_MODE_IDLE:
      stage_photo_cnt = 0;
    #ifdef CAM_TEST
      if(cam_counter <= 0) {
        //LED_TOGGLE(3)
        cam_counter = 40;//
      }
      if(cam_counter == 1) {
        if(!Cam1Check()) {
          cam_mode = CAM_MODE_TURN_ON;
          cam_alt_on = TRUE;
        }
      }
    #endif
      break;
    default:
      break;
  }
  
  if(cam_climb_on > 0)
    cam_climb_on--;

  if(use_uav_agri) {
    //Puffer leeren
    if(buffer_pos > 0) {
      DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice, &buffer[0].photo_nr,
                                            &buffer[0].gps_lat,
                                            &buffer[0].gps_lon,
                                            &buffer[0].gps_z,
                                            &buffer[0].gps_utm_zone,
                                            &buffer[0].phi,
                                            &buffer[0].theta,
                                            &buffer[0].gps_course,
                                            &buffer[0].gps_gspeed,
                                            &buffer[0].gps_itow);
    }
  }
}

