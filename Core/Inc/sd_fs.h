#ifndef INJ_SENSOR_HUB_FS_SD_H
#define INJ_SENSOR_HUB_FS_SD_H

#include <stdint.h>

#include "common.h"

#define SD_PAGE_SIZE 4096 // in bytes

/*
 * Init littefs on the log SD card.
 */
w_status_t sd_fs_init(void);

/*
 * Write a page to littlefs on log SD card.
 */
void sd_fs_write_page(const uint8_t *page);

#endif /* INJ_SENSOR_HUB_FS_SD_H */
