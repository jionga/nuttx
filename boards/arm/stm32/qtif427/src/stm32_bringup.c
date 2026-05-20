/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include "qtif427.h"

/* Conditional logic in omnibusf4.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * omnibusf4.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

  printf("stm32 bringup\n");

#ifdef HAVE_RTC_DRIVER
  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n",
                 ret);
        }
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "failed to mount procfs at %s %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = stm32_gpio_initialize();
  if (ret < 0)
  {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
     return ret;
  }
  syslog(LOG_INFO, " GPIOs have initialized\n");
printf("stm32_gpio_initialize done \n");

#endif

#ifdef CONFIG_SENSORS_ICM42688
  /* Initialize the ICM42688  device. */

  ret = stm32_icm42688_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ICM42688_initialize() failed: %d\n", ret);
    }
#endif


#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAPTURE
  /* Initialize Capture and register the Capture driver. */
  ret = stm32_capture_setup();
  if (ret < 0)
  {
	  syslog(LOG_ERR, "ERROR: stm32_capture_setup failed: %d\n", ret);
	  return ret;
  }
  syslog(LOG_INFO, "Capture have initialized\n");
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */
  ret = stm32_adc_setup();
  if (ret <0)
  {
	  syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
	  return ret;
  }
  syslog(LOG_INFO, "ADC  have initialized\n");
#endif

  setlogmask(LOG_UPTO(LOG_INFO));
  return ret;
}
