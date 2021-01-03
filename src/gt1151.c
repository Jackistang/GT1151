/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2020-11-25     tang jia       the first version
 */
#include "gt1151.h"

#include <touch.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "gt1151"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Reference: https://github.com/goodix/gt1x_driver_generic */

#ifndef GT1151_DEBUG
#define GT1151_DEBUG   0
#endif

#define GT1151_ADDRESS         (0x14)

#define GT1151_STATUS          (0x814E)
#define GT1151_PRODUCT_ID      (0x8140)
#define GT1151_CONFIG          (0x8050)
#define GT1151_COMMAND         (0x8040)

#define GT1151_HW_CONFIG_LEN   239

struct gt1151_object {
    rt_base_t irq_pin;
    rt_base_t rst_pin;
    struct rt_i2c_client *i2c_client;
    struct rt_touch_device *touch_device;
    uint8_t *hw_config;     // GT1151 IC register config array, size is GT1151_HW_CONFIG_LEN
    rt_mutex_t mutex;       // guard hw_config
};

#if GT1151_DEBUG
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
/* Reference: https://my.oschina.net/u/4581400/blog/4760201 */
/**
 * dump_hex
 * 
 * @brief dump data in hex format
 * 
 * @param buf: User buffer
 * @param size: Dump data size
 * @param number: The number of outputs per line
 * 
 * @return void
*/
void dump_hex(const uint8_t *buf, uint32_t size, uint32_t number)
{
    int i, j;

    for (i = 0; i < size; i += number)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < number; j++)
        {
            if (j % 8 == 0)
            {
                rt_kprintf(" ");
            }
            if (i + j < size)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        }
        rt_kprintf(" ");

        for (j = 0; j < number; j++)
        {
            if (i + j < size)
            {
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
            }
        }
        rt_kprintf("\n");
    }
}
#endif

static rt_err_t gt1151_write_regs(struct gt1151_object *object, rt_uint16_t reg_start_addr, rt_uint8_t *write_data, rt_uint16_t len)
{
    RT_ASSERT(object);
    RT_ASSERT(write_data);
    RT_ASSERT(len > 0);

    rt_uint8_t buf[len+2];
    buf[0] = (rt_uint8_t)(reg_start_addr >> 8);
    buf[1] = (rt_uint8_t)(reg_start_addr & 0xff);
    rt_memcpy(buf+2, write_data, len);

    struct rt_i2c_client *dev = object->i2c_client;
    struct rt_i2c_msg msgs;

    msgs.addr = dev->client_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = sizeof(buf);

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
        return RT_EOK;
    else
        return -RT_ERROR;
}

static rt_err_t gt1151_read_regs(struct gt1151_object *object, rt_uint16_t reg_start_addr, rt_uint8_t *read_buf, rt_uint16_t len)
{
    RT_ASSERT(object);
    RT_ASSERT(read_buf);
    RT_ASSERT(len > 0);

    rt_uint8_t regs[2];
    regs[0] = (rt_uint8_t)(reg_start_addr >> 8);
    regs[1] = (rt_uint8_t)(reg_start_addr & 0xff);

    struct rt_i2c_client *dev = object->i2c_client;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = dev->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = regs;
    msgs[0].len   = 2;

    msgs[1].addr  = dev->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = read_buf;
    msgs[1].len   = len;

    if (rt_i2c_transfer(dev->bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * This function read the product id
 *
 * @param dev the pointer of rt_i2c_client
 * @param read_buf the buffer for product id
 * @param len the length for read_buf, should be 4.
 *
 * @return the read status, RT_EOK reprensents  read the product id successfully.
 */
static rt_err_t gt1151_get_product_id(struct gt1151_object *object, rt_uint8_t *read_buf)
{
    RT_ASSERT(object);

    rt_uint8_t buf[4];

    if (gt1151_read_regs(object, GT1151_PRODUCT_ID, buf, 4) != RT_EOK)
    {
        rt_kprintf("gt1151 get product id failed\n");
        return -RT_ERROR;
    }

    rt_memcpy(read_buf, buf, 4);
    return RT_EOK;
}

static rt_err_t gt1151_get_hw_config(struct gt1151_object *object)
{
    RT_ASSERT(object);

    if (gt1151_read_regs(object, GT1151_CONFIG, object->hw_config, GT1151_HW_CONFIG_LEN) != RT_EOK)
    {
        LOG_D("gt1151 get hw config ERROR");
        return -RT_ERROR;
    }

#if GT1151_DEBUG
    dump_hex(object->hw_config, GT1151_HW_CONFIG_LEN, 16);
    uint8_t *hw_config = object->hw_config;
    uint16_t cfg_len = GT1151_HW_CONFIG_LEN;
    uint16_t config = 0;
    uint16_t checksum = ((uint16_t)hw_config[cfg_len-3] << 8) + hw_config[cfg_len-2];
    for (int i = 0; i < cfg_len - 3; i += 2) {
        config += ((uint16_t)hw_config[i] << 8) + hw_config[i+1];
    }

    rt_kprintf("config: 0x%2X\tcheck_sum: 0x%2X\r\n", config, checksum);
    rt_kprintf("total: 0x%2X\r\n", (uint16_t)(config + checksum));
#endif

    return RT_EOK;
}

static rt_err_t gt1151_update_hw_config(struct gt1151_object *object)
{
    RT_ASSERT(object);
    
    rt_err_t ret;

    uint8_t *config = object->hw_config;
    uint16_t cfg_len = GT1151_HW_CONFIG_LEN;
    uint16_t checksum = 0;
    for (int i = 0; i < cfg_len - 3; i += 2) {
        checksum += ((uint16_t)config[i] << 8) + config[i+1];
    }
    checksum = 0 - checksum;

    config[cfg_len-3] = (uint8_t)(checksum >> 8);
    config[cfg_len-2] = (uint8_t)(checksum & 0xFF);
    config[cfg_len-1] = 0x1;

    uint8_t retry = 0;
    while (retry++ < 3) {
        ret = gt1151_write_regs(object, GT1151_CONFIG, config, GT1151_HW_CONFIG_LEN);
        if (ret == RT_EOK) {
            /* at least 200ms, wait for storing config into flash. */
            rt_thread_mdelay(200);
            LOG_D("Send config successfully!");
            return 0;
        }
    }
    LOG_D("Send config failed!");

    return ret;
}

static void gt1151_clear_status(struct gt1151_object *object)
{
    uint8_t status = 0;
    if (gt1151_write_regs(object, GT1151_STATUS, &status, 1) != RT_EOK)
        LOG_D("clear status ERROR");
}

static void gt1151_set_xy_range(struct gt1151_object *object, rt_bool_t isx, void *arg)
{
    RT_ASSERT(object);
    RT_ASSERT(arg);

    uint8_t *config = object->hw_config;
    rt_uint16_t range = *(rt_uint16_t *)arg;

    if (isx) {
        config[1] = range & 0xFF;
        config[2] = range >> 8;
    } else {
        config[3] = range & 0xFF;
        config[4] = range >> 8;
    }
}

/**
 * soft reset, but don't find in dataset, just write here.
 */
static rt_err_t gt1151_soft_reset(struct gt1151_object *object)
{
    rt_uint8_t buf = 0x02;

    if (gt1151_write_regs(object, GT1151_COMMAND, &buf, 1) != RT_EOK)
    {
        LOG_E("soft reset failed");
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_size_t touch_gt1151_readpoint(struct rt_touch_device *touch, void *data_buf, rt_size_t touch_num)
{
    RT_ASSERT(touch_num == 1);  // only support 1 touch point

    struct gt1151_object *object = (struct gt1151_object *)touch->config.user_data;

    uint8_t status;
    uint8_t buf[8];
    uint16_t x = 0, y = 0;

    struct rt_touch_data *pdata = (struct rt_touch_data *)data_buf;

    gt1151_read_regs(object, GT1151_STATUS, &status, 1);

    if (status < 0x80)  // no data get
        return 0;

    else if (status == 0x80) { // no data get
        LOG_D("status: %x", status);
        gt1151_clear_status(object);
        return 0;
    }
    else {
        gt1151_clear_status(object);

        LOG_D("status: %x", status);

        gt1151_read_regs(object, GT1151_STATUS, buf, 8);

        x = ((uint16_t)buf[3] << 8) + buf[2];
        y = ((uint16_t)buf[5] << 8) + buf[4];

        if (x > GT1151_TOUCH_WIDTH - 1)  x = GT1151_TOUCH_WIDTH;
        if (y > GT1151_TOUCH_HEIGHT - 1) y = GT1151_TOUCH_HEIGHT;

        pdata->x_coordinate = x;
        pdata->y_coordinate = y;
        pdata->track_id = 1;
        pdata->event = RT_TOUCH_EVENT_DOWN;
        pdata->timestamp = rt_touch_get_ts();

        status = 0;

        LOG_D("\t x: %d, y: %d", x, y);
    }
    return 1;
}

rt_err_t  touch_gt1151_control(struct rt_touch_device *touch, int cmd, void *arg)
{
    RT_ASSERT(touch);
    RT_ASSERT(arg);

    struct gt1151_object *object = (struct gt1151_object *)touch->config.user_data;

    rt_mutex_take(object->mutex, RT_WAITING_FOREVER);

    switch (cmd) {
    case RT_TOUCH_CTRL_GET_ID : 
        gt1151_get_product_id(object, arg);
        break;
    case RT_TOUCH_CTRL_GET_INFO : 
        rt_memcpy(arg, &object->touch_device->info, sizeof(struct rt_touch_info));
        break;
    case RT_TOUCH_CTRL_SET_MODE :
        LOG_E("not support RT_TOUCH_CTRL_SET_MODE");
        break;
    case RT_TOUCH_CTRL_SET_X_RANGE : 
        gt1151_set_xy_range(object, 1, arg);
        break;
    case RT_TOUCH_CTRL_SET_Y_RANGE : 
        gt1151_set_xy_range(object, 0, arg);
        break; 
    case RT_TOUCH_CTRL_SET_X_TO_Y  : 
        object->hw_config[6] = object->hw_config[6] ^ (1 << 3);
        break;
    case RT_TOUCH_CTRL_DISABLE_INT : 
        LOG_E("not support RT_TOUCH_CTRL_DISABLE_INT");
        break;
    case RT_TOUCH_CTRL_ENABLE_INT  : 
        LOG_E("not support RT_TOUCH_CTRL_ENABLE_INT");
        break;
    default :    
        LOG_E("not supported command %d", cmd); 
        break;
    }

    gt1151_update_hw_config(object);

#if GT1151_DEBUG
    dump_hex(object->hw_config, GT1151_HW_CONFIG_LEN, 16);
#endif

    rt_mutex_release(object->mutex);

    return RT_EOK;
}

static struct rt_touch_ops touch_ops = {
    .touch_readpoint = touch_gt1151_readpoint,
    .touch_control   = touch_gt1151_control,
};

rt_err_t gt1151_hw_init(struct gt1151_object *object)
{
    /* initial hw config array */
    object->hw_config = (uint8_t *)rt_calloc(1, GT1151_HW_CONFIG_LEN);
    if (object->hw_config == RT_NULL) {
        LOG_E("rt_calloc ERROR");
        goto __exit;
    }

    /* hardware init */
    object->irq_pin = GT1151_IRQ_PIN;
    object->rst_pin = GT1151_RST_PIN;
    rt_pin_mode(object->irq_pin, PIN_MODE_OUTPUT);
//    rt_pin_mode(object->irq_pin, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(object->rst_pin, PIN_MODE_OUTPUT);
    rt_pin_write(object->rst_pin, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(object->rst_pin, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_mode(object->irq_pin, PIN_MODE_INPUT_PULLUP);

    /*TODO support interrupt pin */
//    rt_pin_attach_irq(object->irq_pin, PIN_IRQ_MODE_FALLING, irq_handler, RT_NULL);

    rt_thread_mdelay(100);

    /* initial i2c clinet */
    object->i2c_client = (struct rt_i2c_client *)rt_calloc(1, sizeof(struct rt_i2c_client));
    if (object->i2c_client == RT_NULL) {
        LOG_E("rt_calloc ERROR");
        goto __exit;
    }

    object->i2c_client->bus = (struct rt_i2c_bus_device *)rt_device_find(GT1151_BUS_NAME);
    if (object->i2c_client->bus == RT_NULL) {
        LOG_E("not find %s", GT1151_BUS_NAME);
        goto __exit;
    }

    if (rt_device_open((rt_device_t)object->i2c_client->bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open %s device failed", GT1151_BUS_NAME);
        goto __exit;
    }

    object->i2c_client->client_addr = GT1151_ADDRESS;
    gt1151_soft_reset(object);

    gt1151_get_hw_config(object);

    return RT_EOK;

__exit:
    if (object->i2c_client->bus)
        rt_free(object->i2c_client->bus);
    if (object->i2c_client)
        rt_free(object->i2c_client);
    if (object->hw_config)
        rt_free(object->hw_config);

    return RT_ERROR;
}

int gt1151_touch_register(struct gt1151_object *object)
{
    /* initial touch device */
    object->touch_device = (rt_touch_t)rt_calloc(1, sizeof(struct rt_touch_device));
    if (object->touch_device == RT_NULL) {
        LOG_E("rt_calloc ERROR");
        goto __exit;
    }

    object->touch_device->ops = &touch_ops;

    struct rt_touch_info info;
    info.type = RT_TOUCH_TYPE_CAPACITANCE;
    info.vendor = RT_TOUCH_VENDOR_GT;
    info.point_num = GT1151_SUPPORT_POINTS;
    info.range_x = GT1151_TOUCH_WIDTH;
    info.range_y = GT1151_TOUCH_HEIGHT;
    rt_memcpy(&object->touch_device->info, &info, sizeof(info));

    struct rt_touch_config config;
//    config.irq_pin = GT1151_IRQ_PIN;   // TODO support interrupt pin
    config.dev_name = "gt1151";
    config.user_data = object;
    rt_memcpy(&object->touch_device->config, &config, sizeof(config));

    // object->touch_device->irq_handle = ;      // TODO support interrupt pin

    rt_hw_touch_register(object->touch_device, "gt1151", RT_DEVICE_FLAG_RDONLY, RT_NULL);

    return RT_EOK;

__exit:
    if (object->touch_device)
        rt_free(object->touch_device);

    return RT_ERROR;
}

int gt1151_init(void)
{
    struct gt1151_object *object = (struct gt1151_object *)rt_calloc(1, sizeof(struct gt1151_object));
    if (object == RT_NULL)
        goto _exit;
    
    if (gt1151_hw_init(object) != RT_EOK)
        goto _exit;
    if (gt1151_touch_register(object) != RT_EOK)
        goto _exit;

    /* init mutex */
    object->mutex = rt_mutex_create("gt1151", RT_IPC_FLAG_FIFO);
    if (object->mutex == RT_NULL)
        goto _exit;

    rt_device_t touch_dev = rt_device_find("gt1151");
    RT_ASSERT(touch_dev != RT_NULL);
    rt_device_open(touch_dev, RT_DEVICE_FLAG_RDONLY);
    {
        /* set x range */
        rt_int32_t x = GT1151_TOUCH_WIDTH;
        rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_X_RANGE, &x); 
        /* set y range */
        rt_int32_t y = GT1151_TOUCH_HEIGHT;
        rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_Y_RANGE, &y);
    }
    rt_device_close(touch_dev);

    LOG_I("gt1151 init success");
    return RT_EOK;

_exit:
    if (object)
        rt_free(object);
    if (object->mutex)
        rt_mutex_delete(object->mutex);

    LOG_E("gt1151 init failed");
    return RT_ERROR;
}
INIT_DEVICE_EXPORT(gt1151_init);


