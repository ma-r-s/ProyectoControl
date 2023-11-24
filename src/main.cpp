#include <Arduino.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define TRG1 14
#define ECH1 27
#define SERV 16
#define BOTON 32

double Kp = 0;
double Ki = 0.1;
double Kd = 0;

double input;
double driverOut = 0;
double setPoint = 30.0; // Change to float

PID myPID(&input, &driverOut, &setPoint, Kp, Ki, Kd, DIRECT);

Servo fly;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
float dist = 0; // Change from int to float

void inicializarPines()
{
  pinMode(BOTON, INPUT_PULLUP);
  fly.attach(SERV, 1000, 2000);
  fly.write(30);
}

void control(void *pvParameters)
{
  inicializarPines();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  myPID.SetMode(AUTOMATIC);
  for (;;)
  {

    input = dist;
    double error = setPoint - input;
    myPID.Compute();
    double controlSignal = driverOut;
    Serial.print("Error: ");
    Serial.println(error);

    // Print the control signal value
    Serial.print("Control Signal: ");
    Serial.println(controlSignal);

    // fly.write(driverOut);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

float leerDistancia(int trigPin, int echoPin) // Change return type to float
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0; // Change to float
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
      control,
      "pantalla",
      10000,
      NULL,
      2,
      &Task2,
      1);
  xTaskCreatePinnedToCore(
      sensor,
      "pantalla",
      10000,
      NULL,
      2,
      &Task3,
      1);
}

void loop()
{
}
