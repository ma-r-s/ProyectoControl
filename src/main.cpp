#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <ESP32Servo.h>

#define TRG1 14
#define ECH1 27
#define SERV 16
#define BOTON 32

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/15, /* dc=*/4, /* reset=*/2); // Nokia 5110 Display
Servo fly;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
int dist = 0;
void inicializarPines()
{
  pinMode(BOTON, INPUT_PULLUP);
  fly.attach(SERV, 1000, 2000);
  fly.write(30);
}
void control(void *pvParameters)
{
  inicializarPines();

  for (;;)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    fly.write(74);
  }
}

void pantalla(void *pvParameters)
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setDrawColor(1);

  for (;;)
  {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 9, "Distancia");
    // show seconds
    u8g2.setCursor(0, 18);
    u8g2.print("cm: ");
    u8g2.print(dist);

    u8g2.sendBuffer();
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}
int leerDistancia(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = min(duration * 0.034 / 2, 100.0); // Speed of sound wave divided by 2 (go and back)
  return distance;
}
void sensor(void *pvParameters)
{
  pinMode(TRG1, OUTPUT);
  pinMode(ECH1, INPUT);
  for (;;)
  {
    dist = leerDistancia(TRG1, ECH1);
    Serial.print(millis());
    Serial.print(",");
    Serial.println(dist);
    vTaskDelay(1 / portTICK_PERIOD_MS);
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
  xTaskCreatePinnedToCore(
      sensor,     /* Task function. */
      "pantalla", /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      2,          /* priority of the task */
      &Task3,     /* Task handle to keep track of created task */
      1);
}
/* pin task to core 1 */

void loop()
{
}