#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "color_detection.hpp"





void register_iic_data_send(const QueueHandle_t result_i,
                            const QueueHandle_t result_o);