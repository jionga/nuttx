/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/omnibusf4.h
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


#ifndef __BOARDS_ARM_STM32_QTIF427_SRC_QTIF427_H
#define __BOARDS_ARM_STM32_QTIF427_SRC_QTIF427_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_RTC_DRIVER 1

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* GPIOs */

#define BOARD_NGPIOIN     0
#define BOARD_NGPIOOUT    2
#define BOARD_NGPIOINT    0 

#define GPIO_485_HIGH         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                      GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN7)

#define GPIO_CAN_STBY	      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                      GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN13)

#define GPIO_IMU_CS	     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
			GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int stm32_icm42688_initialize(void);

int stm32_bringup(void);
int stm32_gpio_initialize(void);
int stm32_can_setup(void);
int stm32_adc_setup(void);

/* TIMER1 capture for RC */
#ifdef CONFIG_CAPTURE
int stm32_capture_setup(void);
#endif

void weak_function stm32_spidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_QCOMF427_SRC_QTIF427_H */
