
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <nuttx/sensors/icm42688.h>
#include <arch/board/board.h>
#include <nuttx/ioexpander/gpio.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_i2c.h"
#include "qtif427.h"
#include "chip.h"
#include "stm32.h"


#define DEVNODE_ICM42688   "/dev/icm"
#define IMU_SLAVE_ADDR      0x68
#define SPIPORT_ICM42688   3
#define SPIMINOR_ICM42688  0

//static struct stm32gpio_dev_s gpio_imu_cs;

/* spi connect imu */
int stm32_icm42688_initialize(void)
{
  int port = SPIPORT_ICM42688;
  int minor = SPIMINOR_ICM42688;

/* init cs gpio  */
/*
   g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
   g_gpout[i].gpio.gp_ops     = &gpout_ops;
   g_gpout[i].id              = i;
   gpio_pin_register(&g_gpout[i].gpio, pincount);
*/
//   stm32_gpiowrite(GPIO_CS_ICM42688, 0);

//   stm32_configgpio(GPIO_CS_ICM42688);

  struct imu_config_s config =
  {
    .spi_devid = minor,
  };

  struct spi_dev_s *spi = stm32_spibus_initialize(port);
  if (spi == NULL)
  {
    syslog(LOG_ERR, "ERROR: ICM42688_initialize() spi init failed\n");
    return -ENODEV;
  }

  config.spi = spi;
  int ret = icm42688_register(DEVNODE_ICM42688, &config);

  return ret;
}
