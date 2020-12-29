# touch 设备框架

## 关键数据结构

### rt_touch_info

```C
struct rt_touch_info
{
    rt_uint8_t     type;                       /* The touch type */
    rt_uint8_t     vendor;                     /* Vendor of touchs */
    rt_uint8_t     point_num;                  /* Support point num */
    rt_int32_t     range_x;                    /* X coordinate range */
    rt_int32_t     range_y;                    /* Y coordinate range */
};
```

| 成员        | 描述               | 值                                                           |
| ----------- | ------------------ | ------------------------------------------------------------ |
| `type`      | 触摸芯片的类型     | `RT_TOUCH_TYPE_CAPACITANCE` 电容触摸芯片<br />`RT_TOUCH_TYPE_RESISTANCE`   电阻触摸芯片 |
| `vendor`    | 触摸芯片的厂商     | `RT_TOUCH_VENDOR_GT` GTxx 系列<br />`RT_TOUCH_VENDOR_FT` FTxx 系列 |
| `point_num` | 支持同时读取的点数 | 一般为 1 - 10                                                |
| `range_x`   | x 坐标轴的范围     |                                                              |
| `range_y`   | y 坐标轴的范围     |                                                              |

### rt_touch_config

```C
struct rt_touch_config
{
    struct rt_device_pin_mode   irq_pin;       /* Interrupt pin, The purpose of this pin is to notification read data */
    char                        *dev_name;     /* The name of the communication device */
    void                        *user_data;
};
```

| 成员        | 描述                                                         |
| ----------- | ------------------------------------------------------------ | ---- |
| `irq_pin`   | 中断引脚，是一个 `rt_device_pin_mode` 结构体，同时包含引脚编号和引脚中断触发模式。 |      |
| `dev_name`  | 该 touch 设备的名称，由驱动注册时提供。                      |
| `user_data` | 用户数据，驱动编写时可以传递的私有数据。                     |

### rt_touch_ops

```C
struct rt_touch_ops
{
    rt_size_t (*touch_readpoint)(struct rt_touch_device *touch, void *buf, rt_size_t touch_num);
    rt_err_t (*touch_control)(struct rt_touch_device *touch, int cmd, void *arg);
};
```

| 成员              | 描述                                               |
| ----------------- | -------------------------------------------------- |
| `touch_readpoint` | touch 设备读触摸点的函数指针，由驱动与具体函数绑定 |
| `touch_control`   | touch 设备的控制命令函数指针，由驱动与具体函数绑定 |

其中 `cmd` 的值可以如下，驱动不需要全部支持

```C
/* Touch control cmd types */
#define  RT_TOUCH_CTRL_GET_ID            (0)   /* Get device id */
#define  RT_TOUCH_CTRL_GET_INFO          (1)   /* Get touch info */
#define  RT_TOUCH_CTRL_SET_MODE          (2)   /* Set touch's work mode. ex. RT_TOUCH_MODE_POLLING,RT_TOUCH_MODE_INT */
#define  RT_TOUCH_CTRL_SET_X_RANGE       (3)   /* Set x coordinate range */
#define  RT_TOUCH_CTRL_SET_Y_RANGE       (4)   /* Set y coordinate range */
#define  RT_TOUCH_CTRL_SET_X_TO_Y        (5)   /* Set X Y coordinate exchange */
#define  RT_TOUCH_CTRL_DISABLE_INT       (6)   /* Disable interrupt */
#define  RT_TOUCH_CTRL_ENABLE_INT        (7)   /* Enable interrupt */
```

### rt_touch_device

```C
typedef struct rt_touch_device *rt_touch_t;
struct rt_touch_device
{
    struct rt_device            parent;        /* The standard device */
    struct rt_touch_info        info;          /* The touch info data */
    struct rt_touch_config      config;        /* The touch config data */

    const struct rt_touch_ops  *ops;           /* The touch ops */
    rt_err_t (*irq_handle)(rt_touch_t touch);  /* Called when an interrupt is generated, registered by the driver */
};
```

rt_touch_device 是注册 touch 设备的核心数据结构。

| 成员         | 描述                                     | 值                       |
| ------------ | ---------------------------------------- | ------------------------ |
| `parent`     | 标准设备                                 | 由内核设置               |
| `info`       | touch 设备的信息数据                     | `rt_touch_info` 结构体   |
| `config`     | touch 设备的配置数据                     | `rt_touch_config` 结构体 |
| `ops`        | touch 设备的操作集合                     | `rt_touch_ops` 结构体    |
| `irq_handle` | touch 设备的中断处理函数，由驱动函数注册 |                          |

### rt_touch_data

touch 设备框架还抽象出了触摸数据 `rt_touch_data`：

```C
struct rt_touch_data
{
    rt_uint8_t          event;                 /* The touch event of the data */
    rt_uint8_t          track_id;              /* Track id of point */
    rt_uint8_t          width;                 /* Point of width */
    rt_uint16_t         x_coordinate;          /* Point of x coordinate */
    rt_uint16_t         y_coordinate;          /* Point of y coordinate */
    rt_tick_t           timestamp;             /* The timestamp when the data was received */
};
```

| 成员           | 描述                | 值                                                           |
| -------------- | ------------------- | ------------------------------------------------------------ |
| `event`        | 触摸事件            | `RT_TOUCH_EVENT_NONE`  无触摸事件<br />`RT_TOUCH_EVENT_UP`     触摸抬起事件<br />`RT_TOUCH_EVENT_DOWN`  触摸按下事件<br />`RT_TOUCH_EVENT_MOVE`  触摸移动事件 |
| `track_id`     | 该触摸点的 track id | 小于该触摸芯片支持的触摸点数                                 |
| `width`        | 该触摸点的宽度      |                                                              |
| `x_coordinate` | x 轴坐标            |                                                              |
| `y_coordinate` | y 轴坐标            |                                                              |
| `timestamp`    | 时间戳              | 由 `rt_touch_get_ts()` 获取                                  |



## 注册设备流程

首先需根据实际触摸芯片实现下面这两个函数，

```C
rt_size_t drv_xxx_readpoint(struct rt_touch_device *touch, void *buf, rt_size_t touch_num);
rt_err_t  drv_xxx_control  (struct rt_touch_device *touch, int cmd, void *arg);
```

然后实现一个 `struct rt_touch_ops` 结构体对象，并将其 `touch_readpoint` 和 `touch_control` 这两个函数指针和上述两个函数绑定。

然后构建一个 `struct rt_touch_device` 结构体对象，实现其内部的 `info` 和 `config` 结构体，`ops` 指针指向之前定义的 `struct rt_touch_ops` 结构体对象。根据需要配置 `irq_handle` 中断处理函数。然后调用 `rt_hw_touch_register` 注册 touch 设备。

