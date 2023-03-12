
<h1 align="center" style="margin: 30px 0 30px; font-weight: bold;">NRF5_SDK</h1>
<h4 align="center">Nordic ble soft sdk for RT-THREAD</h4>
<p align="center">
	<a href="https://github.com/supperthomas/nrf5x_sdk/stargazers"><img src="https://img.shields.io/github/stars/supperthomas/nrf5x_sdk?style=flat-square&logo=GitHub"></a>
	<a href="https://github.com/supperthomas/nrf5x_sdk/network/members"><img src="https://img.shields.io/github/forks/supperthomas/nrf5x_sdk?style=flat-square&logo=GitHub"></a>
	<a href="https://github.com/supperthomas/nrf5x_sdk/watchers"><img src="https://img.shields.io/github/watchers/supperthomas/nrf5x_sdk?style=flat-square&logo=GitHub"></a>
	<a href="https://github.com/supperthomas/nrf5x_sdk/issues"><img src="https://img.shields.io/github/issues/supperthomas/nrf5x_sdk.svg?style=flat-square&logo=GitHub"></a>
</p>

# Rtthread_nordic_sdk

#### 介绍
基于Nordic  nRF5_SDK_16 的软件包，

目前该软件是基于16.0.0 版本的官方SDK进行移植和方便大家在rtthread的上面尝试nordic官方的softdevice。
目前仅支持MDK， 

GCC和IAR版本暂时没有经历支持和维护，欢迎PR一起来维护。



#### 软件架构
来源官方SDK

#### 使用说明

1.  进入menuconfig中
2.  选择on line package
3.  选择peripheral libraries and drivers

**请注意：**

使用之前请在rtthread工程中找到如下函数，做如下注释修改。才能使用正常softdevice的功能。

```
rt_hw_interrupt_disable    PROC
    EXPORT  rt_hw_interrupt_disable
    ;MRS     r0, PRIMASK
    ;CPSID   I
    BX      LR
    ENDP

;/*
; * void rt_hw_interrupt_enable(rt_base_t level);
; */
rt_hw_interrupt_enable    PROC
    EXPORT  rt_hw_interrupt_enable
    ;MSR     PRIMASK, r0
    BX      LR
    ENDP
```



#### 参与贡献

1.  supperthomas
2. chenyingchun0312
3. guohp1128
4. ylz0923
5. WaterFishJ
