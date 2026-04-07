/*
 * fs_sd.c
 *
 * Interface for littlefs on SD card. Adapted from cansw_logger (ed06dfd).
 */

#include <stdint.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "lfs.h"

#include "common.h"
#include "littlefs_sd_shim.h"
#include "main.h"
#include "sd_fs.h"

#define SD_HANDLE hsd1
#define SD_MAX_FILES_PER_DIR 1000
#define MAX_SD_FILE_SIZE_BYTES (1024U * 1024U * 1024U) // 1 GiB
#define MAX_SD_FILE_SIZE_PAGES (MAX_SD_FILE_SIZE_BYTES / SD_PAGE_SIZE)

static lfs_t lfs;
static lfs_file_t current_log_file;

static uint32_t index_counter = 0;
static uint32_t page_counter = 0;

static void sd_fs_new_file(void) {
  // Create directory as necessary
  if ((index_counter % SD_MAX_FILES_PER_DIR) == 0) {
    char dir_name[100];
    sprintf(dir_name, "dir_%04lu", index_counter / SD_MAX_FILES_PER_DIR);
    lfs_mkdir(&lfs, dir_name);
  }

  // Choose file name
  char log_filename[100];
  sprintf(log_filename, "dir_%04lu/log_%04lu.bin",
      index_counter / SD_MAX_FILES_PER_DIR, index_counter % SD_MAX_FILES_PER_DIR);

  ++index_counter;

  // Update counter file
  lfs_file_t counter_file;
  lfs_file_open(&lfs, &counter_file, "/counter.bin",
      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
  lfs_file_write(&lfs, &counter_file, &index_counter, sizeof(index_counter));
  lfs_file_close(&lfs, &counter_file);

  if (lfs_file_open(&lfs, &current_log_file, log_filename,
      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL) != 0) {
  }

  page_counter = 0;
}

w_status_t sd_fs_init(void) {
  HAL_SD_InitCard(&SD_HANDLE);

  // LittleFS mount
  if (lfsshim_sd_mount_mbr(&lfs, &SD_HANDLE) != 0) {
    return W_FAILURE;
  }

  // Read the file count counter
  lfs_file_t counter_file;
  if (lfs_file_open(&lfs, &counter_file, "counter.bin", LFS_O_RDONLY) == 0) {
    lfs_file_read(&lfs, &counter_file, &index_counter, sizeof(index_counter));
    lfs_file_close(&lfs, &counter_file);
  }

  sd_fs_new_file();

  return W_SUCCESS;
}

void sd_fs_write_page(const uint8_t *page) {
  if (lfs_file_write(&lfs, &current_log_file, page, SD_PAGE_SIZE) < 0) {
    // TODO: handle error
  }
  ++page_counter;
  if (page_counter >= MAX_SD_FILE_SIZE_PAGES) {
    lfs_file_close(&lfs, &current_log_file);
    sd_fs_new_file();
  } else {
    lfs_file_sync(&lfs, &current_log_file);
  }
}
