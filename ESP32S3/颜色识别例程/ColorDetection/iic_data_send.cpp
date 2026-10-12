#include "iic_data_send.hpp"
#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x52

static QueueHandle_t xQueueResultI = NULL;
static QueueHandle_t xQueueResultO = NULL;

static const char *TAG = "iic_data_send";
static const int sdaPin = 47;
static const int sclPin = 48;
static const uint32_t i2cFrequency = 100000;

send_color_data_t send_color_data[4];

static uint8_t rec = 0xFF;
static uint8_t send_data[4] = {0};

static void iic_receive(int len)
{
  while(Wire.available())
  {
    rec = Wire.read();
  }  
}

static void iic_request()
{
    /* 发送红色色块数据 */
    if(rec == 0x00) 
    {
        send_data[0] = send_color_data[0].center_x;
        send_data[1] = send_color_data[0].center_y;
        send_data[2] = send_color_data[0].width;
        send_data[3] = send_color_data[0].length;

    }
    /* 发送绿色色块数据 */
    else if(rec == 0x01)
    {
        send_data[0] = send_color_data[1].center_x;
        send_data[1] = send_color_data[1].center_y;
        send_data[2] = send_color_data[1].width;
        send_data[3] = send_color_data[1].length;

    }
    /* 发送蓝色色块数据 */
    else if(rec == 0x02)
    {
        send_data[0] = send_color_data[2].center_x;
        send_data[1] = send_color_data[2].center_y;
        send_data[2] = send_color_data[2].width;
        send_data[3] = send_color_data[2].length;

    }
    /* 发送紫色色块数据 */
    else if(rec == 0x03)
    {
        send_data[0] = send_color_data[3].center_x;
        send_data[1] = send_color_data[3].center_y;
        send_data[2] = send_color_data[3].width;
        send_data[3] = send_color_data[3].length;
      
    }
    /* 发送色块id号 */
    else if(rec == 0x04)
    {
        send_data[0] = send_color_data[0].id;
        send_data[1] = send_color_data[1].id;
        send_data[2] = send_color_data[2].id;
        send_data[3] = send_color_data[3].id;      
    }
    /* 打包发送色块数据 */
    Wire.slaveWrite(send_data, sizeof(send_data));
}

static void task_process_handler(void *arg)
{
  /* IIC初始化 */
  Wire.begin((uint8_t)I2C_SLAVE_ADDRESS, sdaPin, sclPin, i2cFrequency);
  /* 注册接收数据的回调函数 */
  Wire.onReceive(iic_receive);
  /* 注册请求数据的回调函数 */
  Wire.onRequest(iic_request);

  while (true)
  {

    if (xQueueReceive(xQueueResultI, &send_color_data, portMAX_DELAY))
    {

        // printf("center_x: red:%d  green:%d  blue:%d  purple:%d\r\n", send_color_data[0].center_x, \
        //                                                              send_color_data[1].center_x, \
        //                                                              send_color_data[2].center_x, \
        //                                                              send_color_data[3].center_x);
    }
  }
}

void register_iic_data_send(const QueueHandle_t result_i,
                            const QueueHandle_t result_o)
{
  xQueueResultI = result_i;
  xQueueResultO = result_o;

  xTaskCreatePinnedToCore(task_process_handler, TAG, 5 * 1024, NULL, 5, NULL, 1);
}