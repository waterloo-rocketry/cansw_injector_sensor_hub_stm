/*
 * fs_sd.c
 *
 * Interface for littlefs on SD card. Adapted from cansw_logger (ed06dfd).
 */

#include <stdint.h>
#include <stdio.h>

#include "fatfs.h"
#include "stm32h7xx_hal.h"

#include "canlib.h"
#include "common.h"

#include "main.h"
#include "sd_fs.h"

#define SD_HANDLE hsd1
#define MAX_FILES_PER_DIR 1000
#define MAX_FILE_SIZE_BYTES (1024U * 1024U * 1024U) // 1 GiB
#define MAX_FILE_SIZE_PAGES (MAX_FILE_SIZE_BYTES / SD_PAGE_SIZE)

static FATFS fatfs;
static FIL logfile;

static uint32_t index_counter = 0;
static uint32_t page_counter = 0;
static FRESULT fs_result = FR_OK;

static void sd_fs_new_file(void) {
    unsigned int retval;
      // Create directory as nessary
      if ((index_counter % MAX_FILES_PER_DIR) == 0) {
          char dir_name[100];
          sprintf(dir_name, "dir_%04lu", index_counter / MAX_FILES_PER_DIR);
          f_mkdir(dir_name);
      }

      // Choose file name
      char log_filename[100];
      sprintf(
          log_filename,
          "dir_%04lu/log_%04lu.bin",
          index_counter / MAX_FILES_PER_DIR,
          index_counter % MAX_FILES_PER_DIR
      );

      ++index_counter;

      // Update counter file
      FIL counter_file;
      FRESULT res = f_open(&counter_file, "counter.bin", FA_WRITE | FA_CREATE_ALWAYS);
      res = f_write(&counter_file, &index_counter, sizeof(index_counter), &retval);
      res = f_close(&counter_file);

      fs_result = f_open(&logfile, log_filename, FA_WRITE | FA_OPEN_ALWAYS);

      page_counter = 0;

}

w_status_t sd_fs_init(void) {
  unsigned int retval;

      if (f_mount(&fatfs, "", 0) != FR_OK) {
          return W_IO_ERROR;
      }

      // Read the file count counter
      FIL counter_file;
      if (f_open(&counter_file, "counter.bin", FA_READ) == FR_OK) {
          f_read(&counter_file, &index_counter, sizeof(index_counter), &retval);
      }
      f_close(&counter_file);

      sd_fs_new_file();

      return W_SUCCESS;}

void sd_fs_write_page(const uint8_t *page) {
  unsigned int retval;
      fs_result = f_write(&logfile, page, SD_PAGE_SIZE, &retval);
      ++page_counter;

      if (page_counter >= MAX_FILE_SIZE_PAGES) {
          f_close(&logfile);
          sd_fs_new_file();
      } else {
          f_sync(&logfile);
      }
}

uint32_t sd_fs_get_log_written_size(void) {
  return page_counter * SD_PAGE_SIZE;
}

uint32_t sd_fs_get_log_file_name(void) {
  return index_counter - 1; // index_counter is index of next file
}

uint32_t sd_fs_get_error(void) {
    if(fs_result != FR_OK) {
        return 1 << E_FS_ERROR_OFFSET;
    }
    return 0;
}
