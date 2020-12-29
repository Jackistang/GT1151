
#include "gt1151.h"
#include <touch.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "gt1151_sample"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

void gt1151_test_thread(void *args)
{
    rt_thread_mdelay(1000);

    rt_device_t touch_dev = rt_device_find("gt1151");
    RT_ASSERT(touch_dev != RT_NULL);
    rt_device_open(touch_dev, RT_DEVICE_FLAG_RDONLY);

     /* 读 ID */
     rt_uint8_t read_id[4];
     rt_device_control(touch_dev, RT_TOUCH_CTRL_GET_ID, read_id);
     LOG_I("id = %d %d %d %d \n", read_id[0] - '0', read_id[1] - '0', read_id[2] - '0', read_id[3] - '0');

     /* 获取设备信息 */
     struct rt_touch_info info;
     rt_device_control(touch_dev, RT_TOUCH_CTRL_GET_INFO, &info);
     LOG_I("type       :%d", info.type);                       /* 类型：电容型/电阻型*/
     LOG_I("vendor     :%d", info.vendor);                     /* 厂商 */
     LOG_I("point_num  :%d", info.point_num);                  /* 支持的触点个数 */
     LOG_I("range_x    :%d", info.range_x);                    /* X 轴分辨率 */
     LOG_I("range_y    :%d\n", info.range_y);                  /* Y 轴分辨率*/

     /* 设置工作模式为中断模式 */
     rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_MODE, (void *)RT_DEVICE_FLAG_INT_RX);
     /* 设置工作模式为轮询模式 */
     rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_MODE, (void *)RT_DEVICE_FLAG_RDONLY);

    /* 交换 X、Y 轴坐标 */
//    rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_X_TO_Y, RT_NULL);

    struct rt_touch_data data;
    rt_memset(&data, 0, sizeof(data));
    while (1)
    {
        rt_device_read(touch_dev, 0, &data, 1);
        
        if (data.x_coordinate > 0 || data.y_coordinate > 0)
            LOG_I("\t x: %d, y: %d", data.x_coordinate, data.y_coordinate);

        rt_memset(&data, 0, sizeof(data));

        rt_thread_mdelay(20);
    }

    rt_device_close(touch_dev);
}

int gt1151_test(void)
{
    rt_thread_t tid = rt_thread_create("gt1151", gt1151_test_thread, NULL, 1024, 10, 10);
    rt_thread_startup(tid);

    return RT_EOK;
}
INIT_APP_EXPORT(gt1151_test);
