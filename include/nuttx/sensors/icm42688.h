/****************************************************************************
 * include/nuttx/sensors/icm42688.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_ICM42688_H
#define __INCLUDE_NUTTX_SENSORS_ICM42688_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ICM42688_SPI
struct spi_dev_s;
#else
struct i2c_master_s;
#endif

struct imu_config_s
{
#ifdef CONFIG_ICM42688_SPI
    FAR struct spi_dev_s *spi;
    int spi_devid;
#else
    FAR struct i2c_master_s *i2c;
    int addr;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int icm42688_register(FAR const char *path, FAR struct imu_config_s *config);
#endif
