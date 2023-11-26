#include "HardwareSerial.h"
#include "freertos/queue.h"

QueueHandle_t queue;
TaskHandle_t task;

void receiveTask(void *parameter)
{
  char c;
  while (1)
  {
    // 从队列接收字符
    if (xQueueReceive(queue, &c, portMAX_DELAY))
    {
      // 打印字符
      Serial.print(c);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 2, 3);
  Serial.println("S0:BOOT");
  Serial1.println("S1:BOOT");

  queue = xQueueCreate(10, sizeof(char));
  xTaskCreate(receiveTask, "receiveTask", 1024, NULL, 1, NULL);
}

void loop()
{
  if (Serial1.available())
  {
    char ch = Serial1.read();
    xQueueSend(queue, &ch, portMAX_DELAY);
  }
}