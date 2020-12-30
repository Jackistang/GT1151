# GT1151

## 介绍

适配 RT-Thread 的 GT1151 驱动，已对接 TOUCH 设备框架。

该驱动目前仅支持**一点**触摸，**轮询**方式，配合 GUI 框架能够简单使用。

### 目录结构

| 名称     | 说明                         |
| -------- | ---------------------------- |
| docs     | 文档目录                     |
| examples | 例子目录，并有相应的一些说明 |
| inc      | 头文件目录                   |
| src      | 源代码目录                   |

### 许可证

GT1151 package 遵循 Apache-2.0 License 许可，详见 `LICENSE` 文件。

### 依赖

RT-Thread I2C 设备框架。

## 如何打开 GT1151 

使用 GT1151 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripherals packages --->
        [*] touch package --->
           [*] gt1151
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 使用 GT1151 

在打开 GT1151 package 后，当进行 bsp 编译时，它会被加入到 bsp 工程中进行编译。

该驱动已对接 TOUCH 设备框架，默认设备名为 "gt1151"，用标准的 TOUCH 设备使用方式即可。

## 注意事项

无

## 联系方式 & 感谢

- 维护：Jackistang
- 主页：[https://github.com/Jackistang](https://github.com/Jackistang)

