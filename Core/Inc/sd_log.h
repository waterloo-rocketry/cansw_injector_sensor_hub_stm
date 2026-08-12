#ifndef INJ_SENSOR_HUB_LOG_SD_H
#define INJ_SENSOR_HUB_LOG_SD_H

#include <stdint.h>

#include "canlib.h"
#include "common.h"

/*
 * Initialize logging buffers for SD card.
 */
void sd_log_init(void);

/*
 * Log a CAN message to SD card.
 *
 * Currently the buffer is not `volatile` so don't call this in an interrupt.
 * If we need to call in an interrupt I think we should manually copy instead of memcpy() and
 * pass a non-volatile copy to sd_fs_write_page().
 */
void sd_log_can_message(const can_msg_t *msg, uint32_t timestamp);

/*
 * Write next buffer (page) to SD card if it's full.
 */
void sd_log_flush(void);

#endif /* INJ_SENSOR_HUB_LOG_SD_H */
