/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_boot.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"
#include "itm.h"

#include "stm32.h"
#include "qtif427.h"

/* stm32_adc*/
#include <errno.h>
#include <nuttx/analog/adc.h>
#include "chip.h"

/* stm32_can*/
#include <nuttx/can/can.h>

/* stm32_capture*/
#include <nuttx/timers/capture.h>
#include "stm32_capture.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_SCHED_CRITMONITOR
  up_perf_init((void *)STM32_SYSCLK_FREQUENCY);
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_STM32_OTGFS
  /* Initialize USB if the
   * 1) OTG FS controller is in the configuration and
   * 2) disabled, and
   * 3) the weak function stm32_usbinitialize() has been brought into the
   * build. Presumably either CONFIG_USBDEV or CONFIG_USBHOST is also
   * selected.
   */

  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif

#ifdef HAVE_NETMONITOR
  /* Configure board resources to support networking. */

  if (stm32_netinitialize)
    {
      stm32_netinitialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef HAVE_CCM_HEAP
  /* Initialize CCM allocator */

  ccm_initialize();
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
  /* Perform board-specific initialization */

  stm32_bringup();
}
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  stm32_bringup();
}
#endif


/******************************stm32_adc***********************************/


#ifdef CONFIG_ADC

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
#ifndef CONFIG_STM32_ADC1
#  warning "Channel information only available for ADC1"
#endif

#define GPIO_ADC_VOLTAGE       GPIO_ADC1_IN10

#define DEV_VOLTAGE   "/dev/adc_power"

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS  1

#define ADC_NCHANNELS  (ADC1_NCHANNELS)

static const uint8_t  g_chanlist1[ADC1_NCHANNELS] =
{
  10,
};


/* Configurations of pins used byte each ADC channels */

static const uint32_t g_pinlist[ADC_NCHANNELS] =
{
	GPIO_ADC_VOLTAGE,
};

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
	static bool initialized = false;
	struct adc_dev_s *adc1;
	int ret1;
	int i;

	/* Check if we have already initialized */
	if (!initialized)
    {
		/* Configure the pins as analog inputs for the selected channels */
		for (i = 0; i < ADC_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist[i]);
        }

		/* Call stm32_adcinitialize() to get an instance of the ADC interface */
		adc1 = stm32_adcinitialize(1, g_chanlist1, ADC1_NCHANNELS);
		if (adc1 == NULL ) {
			syslog(LOG_ERR, "ERROR: Failed to get ADC interface\n");
			return -ENODEV;
		}
		/* Register the ADC driver at "/dev/adcxxx" */
		ret1 = adc_register(DEV_VOLTAGE, adc1);
		if (ret1 < 0) {
			syslog(LOG_ERR, "ERROR: Failed to get ADC1 interface\n");
			return ret1;
		}
		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif /* CONFIG_ADC */

/**********************************stm32_can*******************************
 *
 *
 **/

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

// #if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
// #  warning "Both CAN1 and CAN2 are enabled.  Assuming only CAN1."
// #  undef CONFIG_STM32_CAN2
// #endif

#ifdef CONFIG_STM32_CAN1
#  define CAN_PORT_1 1
#endif

#ifdef CONFIG_STM32_CAN2
#  define CAN_PORT_2 2
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int stm32_can_setup(void)
{
#if defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2)
  struct can_dev_s *can;
  int ret;

  /* Call stm32_caninitialize() to get an instance of the CAN interface */

  can = stm32_caninitialize(CAN_PORT_1);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

  can = stm32_caninitialize(CAN_PORT_2);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }
  /* Register the CAN driver at "/dev/can0" */

  ret = can_register("/dev/can1", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */




/*************************************stm32_capture**************************
 *
 *
 * */


#ifdef CONFIG_CAPTURE
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Capture
 *
 * The stm32f4 use timer1 capture function to capture RC receiver PWM
 */

#ifdef CONFIG_CAPTURE
#define RC_CAPTURE_TIMER 1
#define RC_CAPTURE_TIMER_1 9
#endif

#define HAVE_CAPTURE 1

#ifndef CONFIG_CAPTURE
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM1
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM1_CAP
#  undef HAVE_CAPTURE
#endif

#if !defined(CONFIG_STM32_TIM1_CHANNEL)
#  undef HAVE_CAPTURE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_capture_setup
 *
 * Description:
 *   Initialize and register the pwm capture driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/capture0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_capture_setup(void)
{
#ifdef HAVE_CAPTURE
  struct cap_lowerhalf_s *capture;
  int ret;

  capture = stm32_cap_initialize(RC_CAPTURE_TIMER);

  /* register capture0 */

  ret = cap_register("/dev/capture0", capture);
  if (ret < 0) {
	syslog(LOG_ERR, "ERROR: Error registering /dev/capture0\n");
  }

  /* register capture1 */
  capture = stm32_cap_initialize(RC_CAPTURE_TIMER_1);
  ret = cap_register("/dev/capture1", capture);
  if (ret < 0) {
        syslog(LOG_ERR, "ERROR: Error registering /dev/capture1\n");
  }

  return ret;

#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAPTURE */

