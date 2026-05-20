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
#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/nxffs.h>

#include <nuttx/mtd/mtd.h>

#include "stm32.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#include "qtibootf427.h"


#define PROGMEM_MTD_MINOR 0

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

#if defined(CONFIG_MTD_PROGMEM)
  FAR struct mtd_dev_s *mtd;
#if defined(CONFIG_BCH)
  char blockdev[18];
  char chardev[12];
#endif /* defined(CONFIG_BCH) */
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_MTD_PROGMEM)

  mtd = progmem_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
    }
syslog(LOG_ERR, "mtd progmem initialized\n");
  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
             ret);
      return ret;
    }
syslog(LOG_ERR, "ftl initialized\n");
#if defined(CONFIG_BCH)
  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "/dev/mtdblock%d", PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "/dev/mtd%d", PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
             chardev, ret);
      return ret;
    }
#endif /* defined(CONFIG_BCH) */
#endif

  return ret;
}
