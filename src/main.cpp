#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <ESP32Servo.h>

#define TRG1 14
#define ECH1 27
#define TRG2 26
#define ECH2 25
#define TRG3 21
#define ECH3 19
#define DHTPIN 17
#define LUZ 33
#define SERV 16
#define AGUA 22
#define BOTON 32
#define ARRSIZE 10

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/15, /* dc=*/4, /* reset=*/2); // Nokia 5110 Display
Servo fly;
TaskHandle_t Task1;
TaskHandle_t Task2;
void inicializarPines()
{
  pinMode(BOTON, INPUT_PULLUP);
  fly.attach(SERV, 1000, 2000);
  fly.write(30);
}
void control(void *pvParameters)
{
  inicializarPines();
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  for (;;)
  {
    fly.write(200);

    vTaskDelay(400000 / portTICK_PERIOD_MS);
  }
}

void pantalla(void *pvParameters)
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawStr(0, 9, "Te amo Male <3");

  u8g2.sendBuffer();

  for (;;)
  {
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 9, "Te amo Male <3");
    // show seconds
    u8g2.setCursor(0, 18);
    u8g2.print("x ");
    u8g2.print(millis() / 300);

    u8g2.sendBuffer();
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  /* pin task to core 1 */
  xTaskCreatePinnedToCore(
      pantalla,   /* Task function. */
      "pantalla", /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      2,          /* priority of the task */
      &Task1,     /* Task handle to keep track of created task */
      1);
  xTaskCreatePinnedToCore(
      control,    /* Task function. */
      "pantalla", /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      2,          /* priority of the task */
      &Task2,     /* Task handle to keep track of created task */
      1);
}
/* pin task to core 1 */

void loop()
{
}