/****************************************************************************
 * drivers/sensors/icm42688.c
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
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>


#include <nuttx/spi/spi.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/icm42688.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sets bit @n */

#define BIT(n) (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT((m) + 1) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##__WIDTH)) << (m ##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##__SHIFT)) & MASK(m ##__WIDTH))

/* SPI read/write codes */

#define IMU_REG_READ 0x80
#define IMU_REG_WRITE 0
#define ICM42688_ID  0x47

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum imu_regaddr_e
{
    /* bank0 (default）)*/
    ICM42688_DEVICE_CONFIG = 0x11,
    ICM42688_DRIVE_CONFIG = 0x13,
    ICM42688_INT_CONFIG = 0x14,
    ICM42688_FIFO_CONFIG = 0x16,
    ICM42688_TEMP_DATA1 = 0x1D,
    ICM42688_TEMP_DATA0 = 0x1E,
    ICM42688_ACCEL_DATA_X1 =  0x1F,
    ICM42688_ACCEL_DATA_X0 =  0x20,
    ICM42688_ACCEL_DATA_Y1 =  0x21,
    ICM42688_ACCEL_DATA_Y0 =  0x22,
    ICM42688_ACCEL_DATA_Z1 =  0x23,
    ICM42688_ACCEL_DATA_Z0 =  0x24,
    ICM42688_GYRO_DATA_X1 = 0x25,
    ICM42688_GYRO_DATA_X0 = 0x26,
    ICM42688_GYRO_DATA_Y1 = 0x27,
    ICM42688_GYRO_DATA_Y0 = 0x28,
    ICM42688_GYRO_DATA_Z1 = 0x29,
    ICM42688_GYRO_DATA_Z0 = 0x2A,
    ICM42688_TMST_FSYNCH = 0x2B,
    ICM42688_TMST_FSYNCL = 0x2C,
    ICM42688_INT_STATUS = 0x2D,
    ICM42688_FIFO_COUNTH = 0x2E,
    ICM42688_FIFO_COUNTL = 0x2F,
    ICM42688_FIFO_DATA = 0x30,
    ICM42688_APEX_DATA0 = 0x31,
    ICM42688_APEX_DATA1 = 0x32,
    ICM42688_APEX_DATA2 = 0x33,
    ICM42688_APEX_DATA3 = 0x34,
    ICM42688_APEX_DATA4 = 0x35,
    ICM42688_APEX_DATA5 = 0x36,
    ICM42688_INT_STATUS2 = 0x37,
    ICM42688_INT_STATUS3 = 0x38,
    ICM42688_SIGNAL_PATH_RESET =   0x4B,
    ICM42688_INTF_CONFIG0 = 0x4C,
    ICM42688_INTF_CONFIG1 = 0x4D,
    ICM42688_PWR_MGMT0 = 0x4E,
    ICM42688_GYRO_CONFIG0 = 0x4F,
    ICM42688_ACCEL_CONFIG0 =  0x50,
    ICM42688_GYRO_CONFIG1 = 0x51,
    ICM42688_GYRO_ACCEL_CONFIG0 = 0x52,
    ICM42688_ACCEL_CONFIG1 = 0x53,
    ICM42688_TMST_CONFIG = 0x54,
    ICM42688_APEX_CONFIG0 = 0x56,
    ICM42688_SMD_CONFIG = 0x57,
    ICM42688_FIFO_CONFIG1 = 0x5F,
    ICM42688_FIFO_CONFIG2 = 0x60,
    ICM42688_FIFO_CONFIG3 = 0x61,
    ICM42688_FSYNC_CONFIG = 0x62,
    ICM42688_INT_CONFIG0 = 0x63,
    ICM42688_INT_CONFIG1 = 0x64,
    ICM42688_INT_SOURCE0 = 0x65,
    ICM42688_INT_SOURCE1 = 0x66,
    ICM42688_INT_SOURCE3 = 0x68,
    ICM42688_INT_SOURCE4 = 0x69,
    ICM42688_FIFO_LOST_PKT0 = 0x6C,
    ICM42688_FIFO_LOST_PKT1 = 0x6D,
    ICM42688_SELF_TEST_CONFIG = 0x70,
    ICM42688_WHO_AM_I = 0x75,
    ICM42688_REG_BANK_SEL = 0x76,

    /* bank 1 */
    ICM42688_SENSOR_CONFIG0 = 0x03,
    ICM42688_GYRO_CONFIG_STATIC2 = 0x0B,
    ICM42688_GYRO_CONFIG_STATIC3 = 0x0C,
    ICM42688_GYRO_CONFIG_STATIC4 = 0x0D,
    ICM42688_GYRO_CONFIG_STATIC5 = 0x0E,
    ICM42688_GYRO_CONFIG_STATIC6 = 0x0F,
    ICM42688_GYRO_CONFIG_STATIC7 = 0x10,
    ICM42688_GYRO_CONFIG_STATIC8 = 0x11,
    ICM42688_GYRO_CONFIG_STATIC9 = 0x12,
    ICM42688_GYRO_CONFIG_STATIC10 = 0x13,
    ICM42688_XG_ST_DATA = 0x5F,
    ICM42688_YG_ST_DATA = 0x60,
    ICM42688_ZG_ST_DATA = 0x61,
    ICM42688_TMSTVAL0 = 0x62,
    ICM42688_TMSTVAL1 = 0x63,
    ICM42688_TMSTVAL2 = 0x64,
    ICM42688_INTF_CONFIG4 =   0x7A,
    ICM42688_INTF_CONFIG5 =   0x7B,
    ICM42688_INTF_CONFIG6 =   0x7C,

    /* bank2 */
    ICM42688_ACCEL_CONFIG_STATIC2 = 0x03,
    ICM42688_ACCEL_CONFIG_STATIC3 = 0x04,
    ICM42688_ACCEL_CONFIG_STATIC4 = 0x05,
    ICM42688_XA_ST_DATA = 0x3B,
    ICM42688_YA_ST_DATA = 0x3C,
    ICM42688_ZA_ST_DATA = 0x3D,

    /* bank4 */
    ICM42688_GYRO_ON_OFF_CONFIG =  0x0E,
    ICM42688_APEX_CONFIG1 =   0x40,
    ICM42688_APEX_CONFIG2 =   0x41,
    ICM42688_APEX_CONFIG3 =   0x42,
    ICM42688_APEX_CONFIG4 =   0x43,
    ICM42688_APEX_CONFIG5 =   0x44,
    ICM42688_APEX_CONFIG6 =   0x45,
    ICM42688_APEX_CONFIG7 =   0x46,
    ICM42688_APEX_CONFIG8 =   0x47,
    ICM42688_APEX_CONFIG9 =   0x48,
    ICM42688_ACCEL_WOM_X_THR = 0x4A,
    ICM42688_ACCEL_WOM_Y_THR = 0x4B,
    ICM42688_ACCEL_WOM_Z_THR = 0x4C,
    ICM42688_INT_SOURCE6 =    0x4D,
    ICM42688_INT_SOURCE7 =    0x4E,
    ICM42688_INT_SOURCE8 =    0x4F,
    ICM42688_INT_SOURCE9 =    0x50,
    ICM42688_INT_SOURCE10 =   0x51,
    ICM42688_OFFSET_USER0 =   0x77,
    ICM42688_OFFSET_USER1 =   0x78,
    ICM42688_OFFSET_USER2 =   0x79,
    ICM42688_OFFSET_USER3 =   0x7A,
    ICM42688_OFFSET_USER4 =   0x7B,
    ICM42688_OFFSET_USER5 =   0x7C,
    ICM42688_OFFSET_USER6 =   0x7D,
    ICM42688_OFFSET_USER7 =   0x7E,
    ICM42688_OFFSET_USER8 =   0x7F,

    ICM42688_ADDRESS  = 0x69,   /* Address of ICM42688 accel/gyro when ADO = HIGH */

    AFS_2G  = 0x03,
    AFS_4G  = 0x02,
    AFS_8G  = 0x01,
    AFS_16G  = 0x00, /* default */

    GFS_2000DPS = 0x00, /* default */
    GFS_1000DPS = 0x01,
    GFS_500DPS  = 0x02,
    GFS_250DPS  = 0x03,
    GFS_125DPS  = 0x04,
    GFS_62_5DPS = 0x05,
    GFS_31_25DPS = 0x06,
    GFS_15_125DPS = 0x07,

    AODR_8000Hz = 0x03,
    AODR_4000Hz = 0x04,
    AODR_2000Hz = 0x05,
    AODR_1000Hz = 0x06, /* default */
    AODR_200Hz = 0x07,
    AODR_100Hz = 0x08,
    AODR_50Hz = 0x09,
    AODR_25Hz = 0x0A,
    AODR_12_5Hz = 0x0B,
    AODR_6_25Hz = 0x0C,
    AODR_3_125Hz = 0x0D,
    AODR_1_5625Hz = 0x0E,
    AODR_500Hz  = 0x0F,

    GODR_8000Hz = 0x03,
    GODR_4000Hz = 0x04,
    GODR_2000Hz = 0x05,
    GODR_1000Hz = 0x06, /* default */
    GODR_200Hz = 0x07,
    GODR_100Hz = 0x08,
    GODR_50Hz  = 0x09,
    GODR_25Hz  = 0x0A,
    GODR_12_5Hz = 0x0B,
    GODR_500Hz = 0x0F,
};


begin_packed_struct struct sensor_data_s
{
    int16_t temp;
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
} end_packed_struct;

/* Used by the driver to manage the device */

struct imu_dev_s
{
    mutex_t lock;               /* mutex for this structure */
    struct imu_config_s config; /* board-specific information */

    struct sensor_data_s buf;   /* temporary buffer (for read(), etc.) */
    size_t bufpos;              /* cursor into @buf, in bytes (!) */
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

static float g_accsensitivity;   //加速度的最小分辨率 mg/LSB
static float g_gyrosensitivity;    //陀螺仪的最小分辨率

static int imu_open(FAR struct file *filep);
static int imu_close(FAR struct file *filep);
static ssize_t imu_read(FAR struct file *filep, FAR char *buf, size_t len);
static ssize_t imu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len);
static off_t imu_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_imu_fops =
{
    imu_open,        /* open */
    imu_close,       /* close */
    imu_read,        /* read */
    imu_write,       /* write */
    imu_seek,        /* seek */
    NULL,            /* ioctl */
    NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    , NULL           /* unlink */
#endif
};


#ifdef CONFIG_ICM42688_SPI

static int __imu_read_reg_spi(FAR struct imu_dev_s *dev,
                              enum imu_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
    int ret;
    FAR struct spi_dev_s *spi = dev->config.spi;
    int id = dev->config.spi_devid;

    /* We'll probably return the number of bytes asked for. */

    ret = len;

    /* Grab and configure the SPI master device: always mode 0, 20MHz if it's a
     * data register, 1MHz otherwise (per datasheet).
     */

    SPI_LOCK(spi, true);
    SPI_SETMODE(spi, SPIDEV_MODE0);

    /* SET SPI frequency is 24MHZ */
    SPI_SETFREQUENCY(spi, 1000000);

    /* Select the chip. */
    SPI_SELECT(spi, id, true);
    /* Send the read request. */
    SPI_SEND(spi, reg_addr | IMU_REG_READ);

    /* Clock in the data. */
    while (0 != len--)
    {
        *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

    /* Deselect the chip, release the SPI master. */
    SPI_SELECT(spi, id, false);
    SPI_LOCK(spi, false);

    return ret;
}

/* __imu_write_reg(), but for SPI connections. */

static int __imu_write_reg_spi(FAR struct imu_dev_s *dev,
                               enum imu_regaddr_e reg_addr,
                               FAR const uint8_t * buf, uint8_t len)
{
    int ret;
    FAR struct spi_dev_s *spi = dev->config.spi;
    int id = dev->config.spi_devid;

    /* Hopefully, we'll return all the bytes they're asking for. */
    ret = len;

    /* Grab and configure the SPI master device. */
    SPI_LOCK(spi, true);
    SPI_SETMODE(spi, SPIDEV_MODE0);
    SPI_SETFREQUENCY(spi, 1000000);

    /* Select the chip. */
    SPI_SELECT(spi, id, true);

    /* Send the write request. */
    SPI_SEND(spi, reg_addr | IMU_REG_WRITE);

    /* Send the data. */
    while (0 != len--)
    {
        SPI_SEND(spi, *buf++);
    }

    /* Release the chip and SPI master. */
    SPI_SELECT(spi, id, false);
    SPI_LOCK(spi, false);

    return ret;
}

#else

/* __imu_read_reg(), but for i2c-connected devices. */
static int __imu_read_reg_i2c(FAR struct imu_dev_s *dev,
                              enum imu_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
    int ret;
    struct i2c_msg_s msg[2];

    msg[0].frequency = CONFIG_ICM42688_I2C_FREQ;
    msg[0].addr      = dev->config.addr;
    msg[0].flags     = I2C_M_NOSTOP;
    msg[0].buffer    = &reg_addr;
    msg[0].length    = 1;

    msg[1].frequency = CONFIG_ICM42688_I2C_FREQ;
    msg[1].addr      = dev->config.addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = buf;
    msg[1].length    = len;

    ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
    if (ret < 0)
    {
        snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
        return ret;
    }

    return OK;
}

static int __imu_write_reg_i2c(FAR struct imu_dev_s *dev,
                               enum imu_regaddr_e reg_addr,
                               FAR const uint8_t *buf, uint8_t len)
{
    int ret;
    struct i2c_msg_s msg[2];

    msg[0].frequency = CONFIG_ICM42688_I2C_FREQ;
    msg[0].addr      = dev->config.addr;
    msg[0].flags     = I2C_M_NOSTOP;
    msg[0].buffer    = &reg_addr;
    msg[0].length    = 1;
    msg[1].frequency = CONFIG_ICM42688_I2C_FREQ;
    msg[1].addr      = dev->config.addr;
    msg[1].flags     = I2C_M_NOSTART;
    msg[1].buffer    = (FAR uint8_t *)buf;
    msg[1].length    = len;
    ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
    if (ret < 0)
    {
        snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
        return ret;
    }

    return OK;
}
#endif /* CONFIG_ICM42688_SPI */

/* __imu_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int __imu_read_reg(FAR struct imu_dev_s *dev,
                                 enum imu_regaddr_e reg_addr,
                                 FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_ICM42688_SPI
    /* If we're wired to SPI, use that function. */
    if (dev->config.spi != NULL)
    {
        return __imu_read_reg_spi(dev, reg_addr, buf, len);
    }
#else
    /* If we're wired to I2C, use that function. */
    if (dev->config.i2c != NULL)
    {
        return __imu_read_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

    /* If we get this far, it's because we can't "find" our device. */
    return -ENODEV;
}

/* __imu_write_reg()
 *
 * Writes a block of @len byte-wide registers, starting at @reg_addr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

static inline int __imu_write_reg(FAR struct imu_dev_s *dev,
                                  enum imu_regaddr_e reg_addr,
                                  FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_ICM42688_SPI
    /* If we're connected to SPI, use that function. */
    if (dev->config.spi != NULL)
    {
        return __imu_write_reg_spi(dev, reg_addr, buf, len);
    }
#else
    if (dev->config.i2c != NULL)
    {
        return __imu_write_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

    /* If we get this far, it's because we can't "find" our device. */
    return -ENODEV;
}

/****************************************************************************
 * wrapper APIs
 ****************************************************************************/
static inline int __imu_write_imu(FAR struct imu_dev_s *dev,
									  enum imu_regaddr_e reg_addr, uint8_t val)
{
    return __imu_write_reg(dev, reg_addr, &val, sizeof(val));
}

static inline int __imu_read_imu(FAR struct imu_dev_s *dev,
									 enum imu_regaddr_e reg_addr, FAR uint8_t *buf)
{
    return __imu_read_reg(dev, reg_addr, (uint8_t *) buf, sizeof(*buf));
}

static inline uint8_t __imu_read_who_am_i(FAR struct imu_dev_s *dev)
{
    uint8_t val = 0xff;
    __imu_read_reg(dev, ICM42688_WHO_AM_I, &val, sizeof(val));
    return val;
}

/* Locks and unlocks the @dev data structure (mutex).
 *
 * Use these functions any time you call one of the lock-dependent
 * helper functions defined above.
 */

static void inline imu_lock(FAR struct imu_dev_s *dev)
{
    nxmutex_lock(&dev->lock);
}

static void inline imu_unlock(FAR struct imu_dev_s *dev)
{
    nxmutex_unlock(&dev->lock);
}

static float icm42688getares(uint8_t ascale)
{
    float accsensitivity;

    switch(ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
        case AFS_2G:
            accsensitivity = 2000 / 32768.0f;
            break;
        case AFS_4G:
            accsensitivity = 4000 / 32768.0f;
            break;
        case AFS_8G:
            accsensitivity = 8000 / 32768.0f;
            break;
        case AFS_16G:
            accsensitivity = 16000 / 32768.0f;
            break;
    }

    return accsensitivity;
}


float icm42688getgres(uint8_t gscale)
{
    float gyrosensitivity = 0;

    switch(gscale)
    {
        case GFS_15_125DPS:
            gyrosensitivity = 15.125f / 32768.0f;
            break;
        case GFS_31_25DPS:
            gyrosensitivity = 31.25f / 32768.0f;
            break;
        case GFS_62_5DPS:
            gyrosensitivity = 62.5f / 32768.0f;
            break;
        case GFS_125DPS:
            gyrosensitivity = 125.0f / 32768.0f;
            break;
        case GFS_250DPS:
            gyrosensitivity = 250.0f / 32768.0f;
            break;
        case GFS_500DPS:
            gyrosensitivity = 500.0f / 32768.0f;
            break;
        case GFS_1000DPS:
            gyrosensitivity = 1000.0f / 32768.0f;
            break;
        case GFS_2000DPS:
            gyrosensitivity = 2000.0f / 32768.0f;
            break;
    }

    return gyrosensitivity;
}

/* Resets the imu, sets it to a default configuration. */
static int imu_reset(FAR struct imu_dev_s *dev)
{
    uint8_t reg_val;

#ifdef CONFIG_ICM42688_SPI
    if (dev->config.spi == NULL)
    {
        return -EINVAL;
    }
#else
    if (dev->config.i2c == NULL)
    {
        return -EINVAL;
    }
#endif

    imu_lock(dev);

    /* Configure imu chip register*/
    if (ICM42688_ID == __imu_read_who_am_i(dev))
    {
        syslog(LOG_NOTICE, " got right icm42688\n");
    }
    else
    {
        snerr("DEBUG: got icm42688 error \n");
        return ERROR;
    }

	__imu_write_imu(dev, ICM42688_REG_BANK_SEL, 0);
	__imu_write_imu(dev, ICM42688_REG_BANK_SEL, 1);
    nxsig_usleep(100000);

	__imu_write_imu(dev, ICM42688_REG_BANK_SEL, 1);
	__imu_write_imu(dev,ICM42688_INTF_CONFIG4, 0x2);//4-wire SPI mode

	__imu_write_imu(dev, ICM42688_REG_BANK_SEL, 0);
    __imu_write_imu(dev, ICM42688_FIFO_CONFIG, 0x40); //stream-to-FIFO mode

    __imu_read_reg(dev, ICM42688_INT_SOURCE0, &reg_val, sizeof(reg_val));
    __imu_write_imu(dev, ICM42688_INT_SOURCE0, 0x00);
    __imu_write_imu(dev, ICM42688_FIFO_CONFIG2, 0x00);
    __imu_write_imu(dev, ICM42688_FIFO_CONFIG3, 0x02);
    __imu_write_imu(dev, ICM42688_INT_SOURCE0, reg_val);

    __imu_write_imu(dev, ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO
    __imu_write_imu(dev, ICM42688_INT_CONFIG, 0x36);
	
    __imu_read_reg(dev, ICM42688_INT_SOURCE0, &reg_val, sizeof(reg_val));
    reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
    __imu_write_imu(dev, ICM42688_INT_SOURCE0, reg_val);

    g_accsensitivity = icm42688getares(AFS_8G);
    __imu_read_reg(dev, ICM42688_ACCEL_CONFIG0, &reg_val, sizeof(reg_val));
    reg_val |= (AFS_8G << 5);   //量程 ±8g
    reg_val |= (AODR_50Hz);     //输出速率 50HZ
    __imu_write_imu(dev, ICM42688_ACCEL_CONFIG0, reg_val);

    g_gyrosensitivity = icm42688getgres(GFS_1000DPS);
    __imu_read_reg(dev, ICM42688_GYRO_CONFIG0, &reg_val, sizeof(reg_val));
    reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps
    reg_val |= (GODR_50Hz);     //输出速率 50HZ
    __imu_write_imu(dev, ICM42688_GYRO_CONFIG0, reg_val);

    __imu_read_reg(dev, ICM42688_PWR_MGMT0, &reg_val, sizeof(reg_val)); //power on sensor
    reg_val &= ~(1 << 5);//使能温度测量
    reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    __imu_write_imu(dev, ICM42688_PWR_MGMT0, reg_val);
    nxsig_usleep(1000);

    /* Disable i2c if we're on spi. */
    imu_unlock(dev);
    return 0;
}

/****************************************************************************
 * Name: imu_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, don't do that.
 *
 * And no, it's not as simple as just prohibiting concurrent opens or
 * reads with a mutex: there are legit reasons for truy concurrent
 * access, but they must be treated carefully in this interface lest a
 * partial reader end up with a mixture of old and new samples. This
 * will make some users unhappy.
 *
 ****************************************************************************/

static int imu_open(FAR struct file *filep)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct imu_dev_s *dev = inode->i_private;

    /* Reset the register cache */

    imu_lock(dev);
    dev->bufpos = 0;
    imu_unlock(dev);

    return 0;
}

/****************************************************************************
 * Name: imu_close
 ****************************************************************************/

static int imu_close(FAR struct file *filep)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct imu_dev_s *dev = inode->i_private;

    /* Reset (clear) the register cache. */
    imu_lock(dev);
    dev->bufpos = 0;
    imu_unlock(dev);

  return 0;
}

static ssize_t imu_read(FAR struct file *filep, FAR char *buf, size_t len)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct imu_dev_s *dev = inode->i_private;
    size_t send_len = 0;
    FAR uint8_t data[14];

    imu_lock(dev);

	send_len = __imu_read_reg(dev, ICM42688_TEMP_DATA1, data, sizeof(data));

	if ( send_len )
	{
		memcpy(buf, data, send_len);
	}

    # if 0
    /* Populate the register cache if it seems empty. */
    if (!dev->bufpos)
    {
        __imu_read_imu(dev, ICM42688_TEMP_DATA1, &dev->buf);
    }

    /* Send the lesser of: available bytes, or amount requested. */
    send_len = sizeof(dev->buf) - dev->bufpos;
    if (send_len > len)
    {
        send_len = len;
    }

    if (send_len)
    {
        memcpy(buf, ((uint8_t *)&dev->buf) + dev->bufpos, send_len);
    }

    /* Move the cursor, to mark them as sent. */
    dev->bufpos += send_len;

    /* If we've sent the last byte, reset the buffer. */
    if (dev->bufpos >= sizeof(dev->buf))
    {
        dev->bufpos = 0;
    }
    #endif
    imu_unlock(dev);

    return send_len;
}

/****************************************************************************
 * Name: imu_write
 ****************************************************************************/

static ssize_t imu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct imu_dev_s *dev = inode->i_private;
	uint8_t buffer;
	uint8_t reg = 0;
	uint8_t i;

	syslog(LOG_INFO, "imu register: %u %s\n", strlen(buf), buf);
	if (0 == strncmp(buf, "self-test", 9))
	{
		syslog(LOG_INFO, "will invoke self-test \n");
		return 0;
	}

	if (0 == strncmp(buf, "0x", 2))
	{
		if ( strlen(buf) > 5 )
		{
			syslog(LOG_INFO, "Invalid register address too long\n");
			return -EINVAL;
		}

		for ( i=3; i > 1; i --)
		{
			if (buf[i] >= '0' && buf[i] <= '9')
			{
				reg += (3-i) ? 16* (buf[i]-'0') : (buf[i]-'0');
			}
			else if (buf[i]>= 'A' && buf[i] <= 'F')
			{
				reg += (3-i) ? 16 * (buf[i]-'A' + 10) : (buf[i]-'A' +10);
			}
			else
			{
				syslog(LOG_INFO, "Invalid register hex address\n");
				return -EINVAL;
			}
		}
	} else {
		reg = atoi(buf);
	}

	len = __imu_read_reg(dev, reg, &buffer, 1);
	syslog(LOG_INFO, "imu_register: %#X -- %#X\n", reg, buffer);

	UNUSED(inode);
    /*
	UNUSED(dev);
    snerr("ERROR: %p %p %d\n", inode, dev, len);
	*/

    return len;
}

static off_t imu_seek(FAR struct file *filep, off_t offset, int whence)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct imu_dev_s *dev = inode->i_private;

    UNUSED(inode);
    UNUSED(dev);

    snerr("ERROR: %p %p\n", inode, dev);

    return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int icm42688_register(FAR const char *path, FAR struct imu_config_s *config)
{
    FAR struct imu_dev_s *priv;
    int ret;

    /* Without config info, we can't do anything. */
    if (config == NULL)
    {
        return -EINVAL;
    }

    /* Initialize the device structure. */
    priv = (FAR struct imu_dev_s *)kmm_malloc(sizeof(struct imu_dev_s));
    if (priv == NULL)
    {
        snerr("ERROR: Failed to allocate icm42688 device instance\n");
        return -ENOMEM;
    }

    memset(priv, 0, sizeof(*priv));
    nxmutex_init(&priv->lock);

    /* Keep a copy of the config structure, in case the caller discards
     * theirs.
     */
    priv->config = *config;

    /* Reset the chip, to give it an initial configuration. */
    ret = imu_reset(priv);
    if (ret < 0)
    {
        snerr("ERROR: Failed to configure icm42688: %d\n", ret);
        nxmutex_destroy(&priv->lock);

        kmm_free(priv);
        return ret;
    }

    /* Register the device node. */
    ret = register_driver(path, &g_imu_fops, 0666, priv);
    if (ret < 0)
    {
        snerr("ERROR: Failed to register icm42688 interface: %d\n", ret);
        nxmutex_destroy(&priv->lock);
        kmm_free(priv);
        return ret;
    }

    syslog(LOG_INFO, "icm42688 register done.\n");

    return OK;
}
