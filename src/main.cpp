#include <HardwareSerial.h>
#include <freertos/queue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <MQTT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include "driver/rtc_io.h"
#include "main.hpp"

QueueHandle_t queue;
TaskHandle_t app_main_task_handle;
TimerHandle_t MyTimer;
SemaphoreHandle_t xSemaphore = NULL;

WiFiClientSecure espClient;
MQTTClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_Server[0], 8 * 3600);

unsigned long lastMsg = 0, boot = 0;
char msg[MSG_BUFFER_SIZE];
char target[MSG_BUFFER_SIZE];
uint8_t NTP_Count = 0, WiFi_Count = 0;

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

void app_main(void *parameter)
{
  for (;;)
  {
    if (!client.connected())
    {
      reconnect();
    }
    vTaskDelay(1000);
    UpdateMsg();
    Serial.println("app_main Suspend");
    vTaskSuspend(app_main_task_handle);
  }
}

void UnpackData(void)
{
  memcpy(&uart_data, uart_rcv_buf, uart_rcv_count);
  if (uart_data.head != HEAD || uart_data.tail != TAIL)
  {
    Serial.println("unpack failed!");
    UartRecvErrcb();
  }
  else
  {
    uint16_t crc_temp = crc16((uint8_t *)&uart_data.data_len, uart_data.data_len - 4, (uint16_t *)table);
    if (crc_temp == uart_data.crc)
    {
      Serial.println("unpack ok, crc check ok!");
      memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
      uart_state = RCV_HEAD;
      uart_rcv_count = 0;
      uart_rcv_len = 0;
      uart_rcv_flag = 1;
    }
    else
    {
      Serial.println("unpack ok, crc check failed.");
      UartRecvErrcb();
    }
  }
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi_Count > 15)
      ESP.deepSleep(300e6);
    delay(500);
    Serial.print(".");
    WiFi_Count++;
  }

  randomSeed(micros());

  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
  IPAddress serverIP;
  const char *alidns_host = "dns.alidns.com";
  ip_addr_t alidns;
  WiFi.hostByName(alidns_host, serverIP);
  if (serverIP)
  {
    Serial.print("default DNS from DHCP test ok =>");
    Serial.println(serverIP);
  }
  else
  {
    IP_ADDR4(&alidns, 223, 6, 6, 6);
    dhcps_dns_setserver(&alidns);
    Serial.println("default DNS fault, switching to alidns.");
    WiFi.hostByName("dns.alidns.com", serverIP);
    Serial.println(serverIP);
  }
}

void reconnect()
{
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.print("\nconnecting...");
  espClient.setInsecure();

  while (!client.connect("ESP32C3", mqtt_username, mqtt_password))
  {
    Serial.print(".");
  }
  Serial.println("\nConnected to MQTT broker.");
}

void UART1SendCMD(void)
{
  Serial.println("Sent UpdateinfoCMD to UART1!");
  Serial1.write(UpdateinfoCMD, sizeof(UpdateinfoCMD));
}

void setup()
{
  Serial.begin(115200);
  Serial.println("S0:BOOT");

  queue = xQueueCreate(10, sizeof(char));
  xSemaphore = xSemaphoreCreateBinary();
  // xTaskCreate(UART1_receiveTask, "receiveTask", 1024, NULL, 1, NULL);

  Serial1.begin(9600, SERIAL_8N1, 2, 3);

  Serial.println(F("Intializing ..."));
  setup_wifi();
  espClient.setInsecure();
  client.begin(mqtt_broker, mqtt_port, espClient);
  reconnect();
  timeClient.begin();
  timeClient.update();
  do
  {
    if (NTP_Count > sizeof(NTP_Server))
    {
      NTP_Count = 0;
      ESP.deepSleep(60e6);
    }
    Serial.print("Updating time @ " + String(NTP_Server[NTP_Count]) + "=>");
    timeClient.setPoolServerName(NTP_Server[NTP_Count]);
    timeClient.update();
    boot = timeClient.getEpochTime();
    Serial.println(timeClient.getFormattedTime());
    NTP_Count++;
    delay(500);
  } while (boot < 1000000000);

  xTaskCreate(app_main, "app_main", 10240, NULL, 2, &app_main_task_handle);

  MyTimer = xTimerCreate("MyTimer", pdMS_TO_TICKS(1), pdTRUE, 0, MyTimerCallback);
  if (MyTimer == NULL)
  {
    Serial.println("MyTimer create failed.");
  }
  else
  {
    if (xTimerStart(MyTimer, 0) != pdPASS)
    {
      Serial.println("MyTimer start failed.");
    }
    else
    {
      Serial.println("MyTimer started!");
    }
  }
  if (!client.connected())
  {
    reconnect();
  }
}

void loop()
{
  if (Serial1.available())
  {
    char ch = Serial1.read();
    uint8_t temp = ch;
    if (uart_state == RCV_HEAD && temp == HEAD && uart_rcv_count == 0)
    {
      Serial.print("RCV_HEAD...");
      uart_rcv_buf[uart_rcv_count] = temp;
      uart_state = RCV_LEN;
      uart_rcv_count++;
    }
    else if (uart_state == RCV_LEN)
    {
      Serial.print("RCV_LEN...");
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
        UartRecvErrcb();
      }
    }
    else if (uart_state == RCV_CMD)
    {
      if (GET_DATA_ACK == temp) // 判断发来的数据包类型
      {
        Serial.println("RCV_CMD ok");
        uart_rcv_buf[uart_rcv_count] = temp;
        uart_rcv_len--;
        uart_cmd_type = (cmd_type_enum)temp;
        uart_state = RCV_DATA;
        uart_rcv_count++;
      }
      else
      {
        Serial.println("RCV_CMD failed!");
        /* 非法指令,复位并重新接收 */
        UartRecvErrcb();
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
            Serial.printf("S1:Recv %d bytes ok!\r\n", uart_rcv_count);
            UnpackData();
          }
          else
          {
            Serial.println("RCV_DATA failed!");
            UartRecvErrcb();
          }
        }
      }
    }
  }
  delay(10);
}

void UartRecvErrcb(void)
{
  memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
  uart_state = RCV_HEAD;
  uart_rcv_count = 0;
  uart_rcv_len = 0;
  Serial.println("failed on recv, request new:");
  UART1SendCMD();
}

void MyTimerCallback(TimerHandle_t xTimer)
{
  const TickType_t xNewPeriod = pdMS_TO_TICKS(60000);
  xTimerChangePeriod(xTimer, xNewPeriod, 0);
  Serial.print("MyTimer callback:");
  Serial.println("app_main Resume");
  vTaskResume(app_main_task_handle);
}

void UpdateMsg()
{
  Serial.println("Update msg:");
  digitalWrite(LED_BUILTIN, HIGH);
  UART1SendCMD();
  do
  {
    Serial.println("wait recv done... ");
    vTaskDelay(500);
  } while (!uart_rcv_flag);
  GenerateMsg("%d", ESP.getFreeHeap(), "freeram");
  GenerateMsg("%lu", boot, "Boot");
  GenerateMsg("%d", WiFi_Count, "WiFi");
  GenerateMsg("%d", NTP_Count, "NTP");
  GenerateMsg("%d", WiFi.RSSI(), "RSSI");

  GenerateFloatMsg("%.3f", uart_data.mAh, "mAh");
  GenerateFloatMsg("%.3f", uart_data.mWh, "mWh");
  GenerateFloatMsg("%.3f", uart_data.MCU_VCC, "MCU_VCC");
  GenerateQueueData(uart_data.voltage, "voltage");
  GenerateQueueData(uart_data.current, "current");
  GenerateQueueData(uart_data.power, "power");
  GenerateQueueData(uart_data.temperature, "temperature");

  uart_rcv_flag = 0;
  digitalWrite(LED_BUILTIN, LOW);
}

void GenerateQueueData(sensor_data_struct data, const char *codename)
{
  snprintf(msg, MSG_BUFFER_SIZE, "%.3f", data.max);
  snprintf(target, MSG_BUFFER_SIZE, "Crazy/%s/%s/%s", BOARD_NAME, codename, "max");
  PrintandUpdateMsg(target, msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%.3f", data.min);
  snprintf(target, MSG_BUFFER_SIZE, "Crazy/%s/%s/%s", BOARD_NAME, codename, "min");
  PrintandUpdateMsg(target, msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%.3f", data.avg);
  snprintf(target, MSG_BUFFER_SIZE, "Crazy/%s/%s/%s", BOARD_NAME, codename, "avg");
  PrintandUpdateMsg(target, msg);
}

void GenerateMsg(const char *fmt, uint32_t data, const char *codename)
{
  snprintf(msg, MSG_BUFFER_SIZE, fmt, data);
  snprintf(target, MSG_BUFFER_SIZE, "Crazy/%s/%s", codename, BOARD_NAME);
  PrintandUpdateMsg(target, msg);
}

void GenerateFloatMsg(const char *fmt, float data, const char *codename)
{
  snprintf(msg, MSG_BUFFER_SIZE, fmt, data);
  snprintf(target, MSG_BUFFER_SIZE, "Crazy/%s/%s", codename, BOARD_NAME);
  PrintandUpdateMsg(target, msg);
}

void PrintandUpdateMsg(char target[], char msg[])
{
  Serial.println("Publish " + String(target) + " " + String(msg));
  client.publish(target, msg);
  delay(10);
}