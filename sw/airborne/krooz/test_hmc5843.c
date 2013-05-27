/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
/*
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
*/
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/nvic.h>

/*
 *           lisa/L   lisa/M    krooz
 * mag drdy  PB5      PB5				PA8
 *
 */

#include BOARD_CONFIG
#include "std.h"
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "peripherals/hmc5843.h"
#include "peripherals/mpu60X0.h"
#include "math/pprz_algebra_int.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);
static void send_config(void);

static struct i2c_transaction i2c_trans;
struct i2c_transaction t1;
struct i2c_transaction t2;

#define INITIALIZED 9
static uint8_t mag_state = 0;
static volatile uint8_t mag_ready_for_read = FALSE;
static uint8_t reading_mag = FALSE;
//extern void exti9_5_isr(void);

struct i2c_transaction mpu_trans;
uint8_t mpu_state = 0;
uint8_t mpu_ready_for_read = FALSE;
uint8_t mpu_reading = FALSE;


int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  main_init_hw();
	mpu_trans.status=I2CTransSuccess;
}

//#define MPU60X0_I2C_ADDR MPU60X0_ADDR_ALT
#define MPU60X0_I2C_ADDR MPU60X0_ADDR

static inline void main_periodic_task( void ) {
 
  RunOnceEvery(100,
  {
		//LED_TOGGLE(3);
		DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    LED_PERIODIC();
  });
  RunOnceEvery(256,
    {
      uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
      uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
      uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
      uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
      uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
      uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
      uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
      uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
      uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
      const uint8_t _bus2 = 2;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c2_ack_fail_cnt,
                               &i2c2_miss_start_stop_cnt,
                               &i2c2_arb_lost_cnt,
                               &i2c2_over_under_cnt,
                               &i2c2_pec_recep_cnt,
                               &i2c2_timeout_tlow_cnt,
                               &i2c2_smbus_alert_cnt,
                               &i2c2_unexpected_event_cnt,
                               &i2c2_last_unexpected_event,
                               &_bus2);
    });
  if (mag_state == 2) send_config();
	
 if(mpu_trans.status==I2CTransSuccess && mag_state >= INITIALIZED) 
 {
	switch (mpu_state) {
		case 0:
			// MPU60X0_REG_PWR_MGMT_1
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_PWR_MGMT_1;
			mpu_trans.buf[1] = 0x01;
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				LED_ON(3);
			}
		break;
		case 1:
			// MPU60X0_REG_CONFIG
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_CONFIG;
			mpu_trans.buf[1] = (2 << 3) | 			// Fsync / ext sync on gyro X (bit 3->6)
													(0 << 0);					// Low-Pass Filter
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				//LED_ON(3);
			}
		break;
		case 2:
			// MPU60X0_REG_SMPLRT_DIV
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_SMPLRT_DIV;
			mpu_trans.buf[1] = 1;				
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				//LED_TOGGLE(3);
			}
		break;
		case 3:
			// MPU60X0_REG_GYRO_CONFIG
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_GYRO_CONFIG;
			mpu_trans.buf[1] = (3 << 3);				// -2000deg/sec			
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				//LED_TOGGLE(3);
			}
		break;
		case 4:
			// MPU60X0_REG_ACCEL_CONFIG
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_ACCEL_CONFIG;
			mpu_trans.buf[1] = (0 << 0) |			// No HPFL
													(3 << 3);			// Full Scale = 16g		
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				//LED_TOGGLE(3);
			}
		break;
		case 5:
			// MPU60X0_REG_ACCEL_CONFIG
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_INT_PIN_CFG;
			mpu_trans.buf[1] = (1 << 4);			// Any read action clears INT status	
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				//LED_TOGGLE(3);
			}
		break;
		case 6:
			// MPU60X0_REG_ACCEL_CONFIG
			mpu_trans.type = I2CTransTx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_INT_ENABLE;
			mpu_trans.buf[1] = 1;		// INT enable	
			mpu_trans.len_w = 2;
			if(i2c_submit(&i2c2,&mpu_trans)) {
				mpu_state++;
				LED_OFF(3);
			}
		break;
		default:
    break;
/*	
		if(mpu_state != INITIALIZED) {
			mpu_trans.type = I2CTransTxRx;
			mpu_trans.slave_addr = MPU60X0_I2C_ADDR;
			mpu_trans.buf[0] = MPU60X0_REG_WHO_AM_I;
			mpu_trans.len_w = 1;
			mpu_trans.len_r = 2;
			i2c_submit(&i2c2,&mpu_trans);
			mpu_state = INITIALIZED;
		}
		else {
			RunOnceEvery(100,LED_TOGGLE(3));
			if(mpu_trans.status==I2CTransSuccess) {
				RunOnceEvery(100,DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 1, mpu_trans.buf));
			}
		}
*/
	}
	}
	
	if (mag_state < INITIALIZED) 
		mag_state++;
	
}


static inline void main_event_task( void ) {
/**/
  if (mag_state == INITIALIZED && mag_ready_for_read && (i2c_trans.status==I2CTransSuccess || i2c_trans.status == I2CTransFailed)) {
    // read mag
    i2c_trans.type = I2CTransTxRx;
    i2c_trans.slave_addr = HMC5843_ADDR;
		i2c_trans.buf[0] = 0x07;
		i2c_trans.len_w = 1;
    i2c_trans.len_r = 7;
    if(!i2c_submit(&i2c2,&i2c_trans))
			return;
    reading_mag = TRUE;
    mag_ready_for_read = FALSE;
  }

  if (reading_mag && i2c_trans.status==I2CTransSuccess) {
    RunOnceEvery(10,
    {
      int16_t mx   = i2c_trans.buf[0]<<8 | i2c_trans.buf[1];
      int16_t my   = i2c_trans.buf[2]<<8 | i2c_trans.buf[3];
      int16_t mz   = i2c_trans.buf[4]<<8 | i2c_trans.buf[5];
      struct Int32Vect3 m;
      VECT3_ASSIGN(m, mx, my, mz);
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &m.x, &m.y, &m.z);
			//LED_TOGGLE(2);
      //      uint8_t tmp[8];
      //      memcpy(tmp, i2c_trans.buf, 8);
      //      DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 8, tmp);
    }
		 );
    reading_mag = FALSE;
  }

}

static void send_config(void) {

  t1.type = I2CTransTx;
  t1.slave_addr = HMC5843_ADDR;
  t1.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz with HMC5843, 75Hz with HMC5883
  t1.buf[1] = 0x00 | (0x06 << 2);
  t1.len_w = 2;
  i2c_submit(&i2c2,&t1);

  t2.type = I2CTransTx;
  t2.slave_addr = HMC5843_ADDR;
  t2.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
  t2.buf[1] = 0x01<<5;
  t2.len_w = 2;
  i2c_submit(&i2c2,&t2);

  i2c_trans.type = I2CTransTx;
  i2c_trans.slave_addr = HMC5843_ADDR;
  i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
  i2c_trans.buf[1] = 0x00;
  i2c_trans.len_w = 2;
  i2c_submit(&i2c2,&i2c_trans);

}

static inline void main_init_hw( void ) {

#ifdef BOARD_KROOZ
#	if defined(STM32F1) || defined(STM32F2)
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	/* configure external interrupt exti8 on PA8( mag int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* configure external interrupt exti5 on PB5( MPU6050 int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line8 | EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#	elif defined(STM32F4)
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
	//rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);
	
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
	exti_select_source(EXTI5, GPIOB);
	//exti_select_source(EXTI8, GPIOA);
  exti_select_source(EXTI6, GPIOC);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
	//exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
  exti_set_trigger(EXTI6, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI5);
	//exti_enable_request(EXTI8);
  exti_enable_request(EXTI6);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
#endif
#endif
}

void exti9_5_isr(void) {
  /* clear EXTI */
	//if(EXTI_PR & EXTI8) {
  if(EXTI_PR & EXTI6) {
    //exti_reset_request(EXTI8);
    exti_reset_request(EXTI6);
		if (mag_state == INITIALIZED) mag_ready_for_read = TRUE;
		RunOnceEvery(10,LED_TOGGLE(2));
	}
  if(EXTI_PR & EXTI5) {
    exti_reset_request(EXTI5);
		RunOnceEvery(100,LED_TOGGLE(3));
	}
}
