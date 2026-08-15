#ifndef __ALARM_H__
#define __ALARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * 报警超时配置
 * ================================================================ */
#define ALARM_ESTOP_INCONSIST_TIMEOUT_MS    10u   /* 急停双路不一致持续超过此时间则报警 (ms) */

/* ================================================================
 * 报警标志结构体
 * 每个字段对应一路报警：0 = 正常，1 = 报警激活
 * 后续新增报警在此结构体中添加字段即可
 * ================================================================ */
typedef struct {
    uint8_t estop_inconsistent;     /* 急停双路信号状态不一致 */
    uint8_t flash_fault;            /* Flash 自检失败（W25Q128 读写异常）*/
    /* 在此处添加更多报警字段 */
} AlarmFlags_t;

/* ================================================================
 * 报警模块主结构体
 * ================================================================ */
typedef struct {
    AlarmFlags_t flags;             /* 各路报警状态 */
} Alarm_t;

extern Alarm_t g_alarm;             /* 全局报警实例，供外部读取 */

void Alarm_Init(void);              /* 上电初始化，清除所有报警   */
void Alarm_Update(void);            /* 主循环中周期调用           */

#ifdef __cplusplus
}
#endif

#endif /* __ALARM_H__ */
