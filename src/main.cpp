#include "HardwareSerial.h"
#include "freertos/queue.h"

QueueHandle_t queue;
TaskHandle_t task;

#define SIZE(temp) sizeof(temp) / sizeof(uint8_t)
#define UART_BUF_LEN 80
#define HEAD 0x55
#define TAIL 0xAA

void UnpackData(void);

#pragma pack(1)

typedef enum cmd_type_enum
{
  CMD_NONE = 0,
  GET_DATA_CMD = 0xBB,
  UPDATE_RTC_CMD = 0xBC,
  GET_DATA_ACK = 0xCB,
  CMD_TYPE_MAX = 255
} cmd_type_enum;

typedef enum uart_rcv_state_enum
{
  RCV_IDLE = 0,
  RCV_HEAD,
  RCV_LEN,
  RCV_CMD,
  RCV_DATA,
  RCV_STATE_MAX = 255
} uart_rcv_state_enum;

typedef struct sensor_data_struct
{
  float max;
  float avg;
  float min;
} sensor_data_struct;

typedef struct uart_data_struct
{
  uint8_t head;
  uint8_t data_len;
  uint8_t cmd_type;
  sensor_data_struct voltage;
  sensor_data_struct current;
  sensor_data_struct power;
  sensor_data_struct temperature;
  float mWh;
  float mAh;
  float MCU_VCC;
  uint16_t crc;
  uint8_t tail;
} uart_data_struct;

#pragma pack()

char UpdateinfoCMD[] = {0x55, 0x04, 0xBB, 0xAA};
uart_rcv_state_enum uart_state = RCV_HEAD;
cmd_type_enum uart_cmd_type;
uint8_t uart_rcv_buf[UART_BUF_LEN] = {0};
uart_data_struct uart_data = {0};
uint8_t uart_rcv_count = 0;
uint8_t uart_rcv_len = 0;
uint8_t uart_rcv_flag = 0;

void UART1_receiveTask(void *parameter)
{
  char c;
  while (1)
  {
    if (xQueueReceive(queue, &c, portMAX_DELAY))
    {
      uint8_t temp = c;
      if (uart_state == RCV_HEAD && temp == HEAD && uart_rcv_count == 0)
      {
        Serial.println("RCV_HEAD");
        uart_rcv_buf[uart_rcv_count] = temp;
        uart_state = RCV_LEN;
        uart_rcv_count++;
      }
      else if (uart_state == RCV_LEN)
      {
        Serial.println("RCV_LEN");
        /* 确认接收长度是否小于接收缓冲区,如果否重置接收流程 */
        if (temp <= UART_BUF_LEN)
        {
          uart_rcv_buf[uart_rcv_count] = temp;
          uart_rcv_len = temp - 2;
          uart_state = RCV_CMD;
          uart_rcv_count++;
        }
        else
        {
          Serial.println("err @ RCV_LEN");
          // memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
          uart_state = RCV_HEAD;
          uart_rcv_count = 0;
          uart_rcv_len = 0;
        }
      }
      else if (uart_state == RCV_CMD)
      {
        if (GET_DATA_ACK == temp) // 判断发来的数据包类型
        {
          Serial.println("RCV_CMD ok");
          uart_rcv_buf[uart_rcv_count] = temp;
          uart_rcv_len--;
          uart_cmd_type = (cmd_type_enum)(temp);
          uart_state = RCV_DATA;
          uart_rcv_count++;
        }
        else
        {
          Serial.println("RCV_CMD failed!");
          /* 非法指令,复位并重新接收 */
          uart_state = RCV_HEAD;
          uart_rcv_count = 0;
          uart_rcv_len = 0;
        }
      }
      else if (uart_state == RCV_DATA)
      {
        /* 没到最后一个数据,继续接收 */
        if (uart_rcv_len != 1)
        {
          uart_rcv_len--;
          uart_rcv_buf[uart_rcv_count++] = temp;
        }
        /* 准备接收最后一个数据 */
        else
        {
          /* 确认是否为最后一个数据 */
          if (uart_rcv_len == 1)
          {
            /* 确认最后一个数据是否为帧尾 */
            if (temp == TAIL)
            {
              uart_rcv_buf[uart_rcv_count++] = temp;
              Serial.printf("S0:Recv %d bytes ok!\r\n", uart_rcv_count);
              UnpackData();
            }
            else
            {
              Serial.println("RCV_DATA failed!");
              uart_state = RCV_HEAD;
              uart_rcv_count = 0;
              uart_rcv_len = 0;
            }
          }
        }
      }
    }
  }
}

void UnpackData(void)
{
  memcpy(&uart_data, uart_rcv_buf, uart_rcv_count);
  if (uart_data.head != HEAD || uart_data.tail != TAIL)
  {
    Serial.println("unpack failed!");
    memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
    uart_rcv_count = 0;
    uart_rcv_len = 0;
  }
  else
  {
    Serial.println("unpack ok!");
  }
}

void setup()
{
  queue = xQueueCreate(10, sizeof(char));
  xTaskCreate(UART1_receiveTask, "receiveTask", 1024, NULL, 1, NULL);

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 2, 3);
  Serial.println("S0:BOOT");
  Serial1.write(UpdateinfoCMD, sizeof(UpdateinfoCMD));
}

void loop()
{
  if (Serial1.available())
  {
    char ch = Serial1.read();
    xQueueSend(queue, &ch, portMAX_DELAY);
  }
}