#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define SEND_CLOLOR_NUM 4

typedef struct
{
  uint8_t id;
  uint8_t center_x;
  uint8_t center_y;
  uint8_t width;
  uint8_t length;
}send_color_data_t;


void register_iic_data_send(const QueueHandle_t result_i,
                            const QueueHandle_t result_o);