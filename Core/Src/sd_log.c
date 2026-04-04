/*
 * log_sd.c
 *
 * Log data to littlefs on SD card. Adapted from cansw_logger (ed06dfd).
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "canlib.h"
#include "common.h"

#include "sd_fs.h"
#include "sd_log.h"

#define SD_LOG_SIGNATURE_SIZE 4
#define SD_LOG_BUFFER_COUNT 4

static size_t fill_buffer_index = 0; // Buffer being filled with log data
static size_t flush_buffer_index = 0; // Next buffer to be written (flushed) to storage
static uint8_t page_number = 0;

/*
 * Each buffer represents a page.
 * On cansw_logger this is `volatile` but I think it's best not to for now since we're using memcpy
 * and currently we don't log anything in interrupts.
*/
static struct {
  size_t pointer;               // Write position
  uint8_t buffer[SD_PAGE_SIZE];
  bool ready_to_flush;          // true when buffer is full and thus ready to flush
} log_buffers[SD_LOG_BUFFER_COUNT];


void sd_log_init(void) {
  for (int i = 0; i < SD_LOG_BUFFER_COUNT; ++i) {
    log_buffers[i].pointer = 0;
    log_buffers[i].ready_to_flush = false;
  }
}

void sd_log_can_message(const can_msg_t *msg, uint32_t timestamp) {
  size_t pointer = log_buffers[fill_buffer_index].pointer;

  /*
   * Log message format:
   * - Bytes 0:3 store 29-bit rocketcan SID
   * - Bytes 4:7 store 32-bit timestamp
   * - Byte 8 stores data length (up to 8 bytes as per rocketcan spec)
   * - Next data_len bytes store data
   * The total length is (9 + data_len) bytes.
   */

  // If not enough space in buffer
  if ((SD_PAGE_SIZE - pointer) < (9 + msg->data_len)) {
    // Fill remaining space in buffer with 0xff
    for (; pointer < SD_PAGE_SIZE; ++pointer) {
      log_buffers[fill_buffer_index].buffer[pointer] = 0xff;
    }

    log_buffers[fill_buffer_index].pointer = 0;
    log_buffers[fill_buffer_index].ready_to_flush = true;
    fill_buffer_index =
        (fill_buffer_index >= (SD_LOG_BUFFER_COUNT - 1)) ?
            0 : (fill_buffer_index + 1);
    pointer = log_buffers[fill_buffer_index].pointer;
    ++page_number;
  }

  if (pointer < SD_LOG_SIGNATURE_SIZE) {
    // Write page signature. Make sure length matches SD_LOG_SIGNATURE_SIZE
    log_buffers[fill_buffer_index].buffer[0] = 'L';
    log_buffers[fill_buffer_index].buffer[1] = 'O';
    log_buffers[fill_buffer_index].buffer[2] = 'G';
    log_buffers[fill_buffer_index].buffer[3] = page_number & 0xff;
    log_buffers[fill_buffer_index].pointer = SD_LOG_SIGNATURE_SIZE;
    pointer = log_buffers[fill_buffer_index].pointer;
  }

  log_buffers[fill_buffer_index].buffer[pointer + 0] = msg->sid & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 1] = (msg->sid >> 8) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 2] = (msg->sid >> 16) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 3] = (msg->sid >> 24) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 4] = timestamp & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 5] = (timestamp >> 8) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 6] = (timestamp >> 16) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 7] = (timestamp >> 24) & 0xff;
  log_buffers[fill_buffer_index].buffer[pointer + 8] = msg->data_len;

  memcpy(log_buffers[fill_buffer_index].buffer + pointer + 9, msg->data,
      msg->data_len);

  log_buffers[fill_buffer_index].pointer += 9 + msg->data_len;
}

void sd_log_flush(void) {
  if (log_buffers[flush_buffer_index].ready_to_flush) {
    sd_fs_write_page(log_buffers[flush_buffer_index].buffer);
    log_buffers[flush_buffer_index].ready_to_flush = false;
    flush_buffer_index = (
        flush_buffer_index >= (SD_LOG_BUFFER_COUNT - 1) ?
            0 : (flush_buffer_index + 1));
  }
}
