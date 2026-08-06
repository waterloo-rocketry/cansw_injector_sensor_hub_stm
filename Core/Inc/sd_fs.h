#ifndef INJ_SENSOR_HUB_FS_SD_H
#define INJ_SENSOR_HUB_FS_SD_H

#include <stdint.h>

#include "common.h"

#define SD_PAGE_SIZE 4096 // in bytes

/*
 * Init FATFS on the log SD card.
 */
w_status_t sd_fs_init(void);

/*
 * Write a page to FATFS on log SD card.
 */
void sd_fs_write_page(const uint8_t *page);

/*
 * Get total number of bytes written so far to the SD card.
 */
uint32_t sd_fs_get_log_written_size(void);

/*
 * Get number of current log file, e.g. 1234 for log_1234.bin.
 */
uint32_t sd_fs_get_log_file_name(void);

/*
 * Return canlib offset error bit for FS error if there is one.
 */
uint32_t sd_fs_get_error(void);

#endif /* INJ_SENSOR_HUB_FS_SD_H */
