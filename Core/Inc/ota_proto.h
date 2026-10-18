#ifndef OTA_PROTO_H
#define OTA_PROTO_H

#include <stdint.h>

/* ---- Flash 布局（与各链接脚本一致）---- */
#define OTA_BOOT_ADDR     0x08000000u   /* bootloader（32KB） */
#define OTA_APP_ADDR      0x08008000u   /* 应用固件 */
#define OTA_STATUS_ADDR   0x080FF000u   /* Sector 11 内的状态地址；Sector 11 为 128KB */

/* ---- OTA 状态区内容 ---- */
#define OTA_STATE_MAGIC   0x4F544159u   /* "OTAY" */
#define OTA_STATE_UPDATE  0x55555054u   /* 等待升级 */
#define OTA_STATE_READY   0x52504454u   /* 新固件有效 */
#define OTA_STATE_NONE    0x4E4F4E45u   /* 无操作 */

/* ---- 触发：Modbus 保持寄存器写这个值进入 OTA ---- */
#define OTA_TRIGGER_REG     49u
#define OTA_TRIGGER_MAGIC   0x4F54u     /* "OT" */

/* ---- 帧类型 ---- */
#define OTA_FRAME_SYNC    0x00u
#define OTA_FRAME_START   0x01u
#define OTA_FRAME_DATA    0x02u
#define OTA_FRAME_END     0x03u
#define OTA_FRAME_ABORT   0x04u

/* ---- 应答状态 / 结构 ---- */
#define OTA_RESP_ACK      0xA5u
#define OTA_ST_OK         0x00u
#define OTA_ST_CRC        0x01u
#define OTA_ST_SEQ        0x02u
#define OTA_ST_BUSY       0x03u
#define OTA_ST_ABORT      0x04u
#define OTA_ST_READY      0x05u

#define OTA_MAGIC         0xA5u
#define OTA_BUF_SIZE      512u

typedef struct {
    uint32_t magic;
    uint32_t state;
    uint32_t app_size;
    uint32_t app_crc;
} OtaStatus_t;

#endif /* OTA_PROTO_H */
