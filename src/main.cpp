#include "HardwareSerial.h"
#include "freertos/queue.h"

QueueHandle_t queue;
TaskHandle_t task;

#define SIZE(temp) sizeof(temp) / sizeof(uint8_t)
#define UART_BUF_LEN 80
#define HEAD 0x55
#define TAIL 0xAA

void UnpackData(void);
uint16_t crc16(uint8_t *data, uint8_t len, uint16_t *table);

const uint16_t table[256] = {0x0, 0xa001, 0xe003, 0x4002, 0x6007, 0xc006, 0x8004, 0x2005, 0xc00e, 0x600f, 0x200d, 0x800c, 0xa009, 0x8, 0x400a, 0xe00b, 0x201d, 0x801c, 0xc01e, 0x601f,
                             0x401a, 0xe01b, 0xa019, 0x18, 0xe013, 0x4012, 0x10, 0xa011, 0x8014, 0x2015, 0x6017, 0xc016, 0x403a, 0xe03b, 0xa039, 0x38, 0x203d, 0x803c, 0xc03e, 0x603f,
                             0x8034, 0x2035, 0x6037, 0xc036, 0xe033, 0x4032, 0x30, 0xa031, 0x6027, 0xc026, 0x8024, 0x2025, 0x20, 0xa021, 0xe023, 0x4022, 0xa029, 0x28, 0x402a, 0xe02b,
                             0xc02e, 0x602f, 0x202d, 0x802c, 0x8074, 0x2075, 0x6077, 0xc076, 0xe073, 0x4072, 0x70, 0xa071, 0x407a, 0xe07b, 0xa079, 0x78, 0x207d, 0x807c, 0xc07e, 0x607f,
                             0xa069, 0x68, 0x406a, 0xe06b, 0xc06e, 0x606f, 0x206d, 0x806c, 0x6067, 0xc066, 0x8064, 0x2065, 0x60, 0xa061, 0xe063, 0x4062, 0xc04e, 0x604f, 0x204d, 0x804c,
                             0xa049, 0x48, 0x404a, 0xe04b, 0x40, 0xa041, 0xe043, 0x4042, 0x6047, 0xc046, 0x8044, 0x2045, 0xe053, 0x4052, 0x50, 0xa051, 0x8054, 0x2055, 0x6057, 0xc056,
                             0x205d, 0x805c, 0xc05e, 0x605f, 0x405a, 0xe05b, 0xa059, 0x58, 0xa0e9, 0xe8, 0x40ea, 0xe0eb, 0xc0ee, 0x60ef, 0x20ed, 0x80ec, 0x60e7, 0xc0e6, 0x80e4, 0x20e5,
                             0xe0, 0xa0e1, 0xe0e3, 0x40e2, 0x80f4, 0x20f5, 0x60f7, 0xc0f6, 0xe0f3, 0x40f2, 0xf0, 0xa0f1, 0x40fa, 0xe0fb, 0xa0f9, 0xf8, 0x20fd, 0x80fc, 0xc0fe, 0x60ff,
                             0xe0d3, 0x40d2, 0xd0, 0xa0d1, 0x80d4, 0x20d5, 0x60d7, 0xc0d6, 0x20dd, 0x80dc, 0xc0de, 0x60df, 0x40da, 0xe0db, 0xa0d9, 0xd8, 0xc0ce, 0x60cf, 0x20cd, 0x80cc, 0xa0c9, 0xc8,
                             0x40ca, 0xe0cb, 0xc0, 0xa0c1, 0xe0c3, 0x40c2, 0x60c7, 0xc0c6, 0x80c4, 0x20c5, 0x209d, 0x809c, 0xc09e, 0x609f, 0x409a, 0xe09b, 0xa099, 0x98, 0xe093, 0x4092, 0x90, 0xa091,
                             0x8094, 0x2095, 0x6097, 0xc096, 0x80, 0xa081, 0xe083, 0x4082, 0x6087, 0xc086, 0x8084, 0x2085, 0xc08e, 0x608f, 0x208d, 0x808c, 0xa089, 0x88, 0x408a, 0xe08b, 0x60a7, 0xc0a6,
                             0x80a4, 0x20a5, 0xa0, 0xa0a1, 0xe0a3, 0x40a2, 0xa0a9, 0xa8, 0x40aa, 0xe0ab, 0xc0ae, 0x60af, 0x20ad, 0x80ac, 0x40ba, 0xe0bb, 0xa0b9, 0xb8, 0x20bd, 0x80bc, 0xc0be, 0x60bf,
                             0x80b4, 0x20b5, 0x60b7, 0xc0b6, 0xe0b3, 0x40b2, 0xb0, 0xa0b1};

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

uint16_t crc16(uint8_t *data, uint8_t len, uint16_t *table)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = (crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return crc;
}

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
    uint16_t crc_temp = crc16((uint8_t *)&uart_data.data_len, uart_data.data_len - 4, (uint16_t *)table);
    if (crc_temp == uart_data.crc)
    {
      Serial.println("unpack ok, crc check ok!");
    }
    else
    {
      Serial.println("unpack ok, crc check failed.");
    }
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