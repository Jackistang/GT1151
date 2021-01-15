# TODO

## 添加中断方式的支持

该模式目前未添加是因为下述情况还为解决：

我在轮询访问 GT1151 时发现，每次触摸抬起后 GT1151 的 0X814E 状态寄存器仍会产生三个 0x80 ，如果没有靠轮询去清除状态寄存器，GT1151 不会工作了。如果是中断方式使用 GT1151，这会出现永远无法读到触摸点的信息了。

如下图：

![](images/image-20201229155620035.png)

## 完善触摸点读取驱动

参考 goodix 提供的 Linux 读触摸驱动完善 RT-Thread GT1151 触摸点读取驱动。

参考 [gt1x_generic.c](https://github.com/goodix/gt1x_driver_generic) 的 1252 处的 `gt1x_touch_event_handler` 函数。



## 触摸抬起事件

以下思路参考 `gt1x_driver_generic` 项目。

使用一个 16 位的局部变量 `pre_index`，低 10 位用于表示 10 个触摸点的状态。

每次读取到相应 `track_id` 的触摸点数据时，将 `pre_index` 的相应位置位，并触发**触摸按下事件**。

若相应的 `track_id` 没有触摸点数据了，且 `pre_index` 相应位仍为 1，则清空相应位，并触发**触摸抬起事件**。

触摸抬起事件添加失败：

## 无触摸数据仍会产生中断

```C
	if (unlikely(!pre_event && !cur_event)) {
		GTP_DEBUG("Additional Int Pulse.");
	} else {
		pre_event = cur_event;
	}
```

