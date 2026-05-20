/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include "qtif427.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32_SPI3
//  stm32_configgpio(GPIO_CS_ICM42688);
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

  /* Note: NSS is active-low. */
  /* Note: CS is active-low. */
  stm32_gpiowrite(GPIO_CS_ICM42688, !selected);
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  /* Note: SD_DET is pulled high when there's no SD card present. */

  return stm32_gpioread(GPIO_CS_ICM42688) ? 0 : 1;
}
#endif

#endif

