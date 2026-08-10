/**
 * SysConfig.h 测试桩 — 提供飞控固件类型定义，脱离硬件依赖
 *
 * 原始 SysConfig.h 通过 McuConfig.h → stm32f4xx.h 等提供 u8/s8/u16 等类型，
 * 本桩文件用 <stdint.h> 直接定义，切断所有 MCU/BSP 依赖链。
 */
#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_

#include <stdint.h>

/* ---- 基本类型（原自 McuConfig.h / stm32f4xx.h）---- */
typedef uint8_t   u8;
typedef int8_t    s8;
typedef uint16_t  u16;
typedef int16_t   s16;
typedef uint32_t  u32;
typedef int32_t   s32;

/* ---- 向量类型（原始 SysConfig.h）---- */
typedef float vec3_f[3];
typedef float vec2_f[2];
typedef s32 vec3_s32[3];
typedef s32 vec2_s32[3];
typedef s16 vec3_s16[3];
typedef s16 vec2_s16[2];

/* ---- 系统常量（原始 SysConfig.h，Ano_Math 不依赖但可能间接引用）---- */
#define TICK_PER_SECOND  1000
#define TICK_US          (1000000 / TICK_PER_SECOND)
#define PWM_FRE_HZ       400
#define LED_NUM          4

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define HW_ALL     0xFF
#define SWJ_ADDR   0xAF
#define HW_TYPE    0x61
#define HW_VER     1
#define SOFT_VER   17
#define BL_VER     0
#define PT_VER     400

#define LED_R   0x01
#define LED_G   0x02
#define LED_B   0x04
#define LED_S   0x08
#define LED_ALL 0xFF

#endif /* _SYSCONFIG_H_ */
