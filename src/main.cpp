#include <Arduino.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define TRG1 14
#define ECH1 27
#define SERV 16
#define BOTON 32

// Sintesis directa P
// double l = 0.4;
// double Kp = 1.811 / (l * 1.169);
// double Ki = 0;
// double Kd = 0;

// Sintesis directa PI
// double l = 4;
// double Kp = 0.7758 / (l * 0.5082);
// double Ki = 1 / (l * 0.5082);
// double Kd = 0;

// Sintesis directa PID
// double l = 8;
// double Kp = 1.811 / (l * 1.169);
// double Ki = 1.917 / (l * 1.169);
// double Kd = 1 / (l * 1.169);

// Ziegler-Nichols PID
// double Kp = 0.323;
// double Ki = 1.433;
// double Kd = 0.112;

// Autotune
// double Kp = 0.597;
// double Ki = 0.931;
// double Kd = 0.0;

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
  Serial.println("Tiempo, Distancia, Error, setPoint, Control");
  inicializarPines();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  myPID.SetMode(AUTOMATIC);
  for (;;)
  {

    input = dist;
    double error = setPoint - input;
    myPID.Compute();
    double controlSignal = driverOut;
    Serial.print(millis());
    Serial.print(",");
    Serial.print(dist);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.print(setPoint);
    Serial.print(",");
    Serial.println(controlSignal);

    fly.write(driverOut);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

float prevDist = 0.0;
float leerDistancia(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0; // Change to float

  // Set a threshold for acceptable difference
  float threshold = 30.0;

  // Check if the difference is too large
  if (abs(distance - prevDist) > threshold)
  {
    // If so, return the previous distance
    return prevDist;
  }

  // Update prevDist with the current distance
  prevDist = distance;

  // Return the current distance with an offset
  return distance - 7.77;
}

void sensor(void *pvParameters)
{
  pinMode(TRG1, OUTPUT);
  pinMode(ECH1, INPUT);
  for (;;)
  {
    dist = leerDistancia(TRG1, ECH1);
  }
}

void ponerSalida(void *pvParameters)
{
  for (;;)
  {
    setPoint = 60.0;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // setPoint = 30.0;
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
  xTaskCreatePinnedToCore(
      ponerSalida,
      "pantalla",
      10000,
      NULL,
      2,
      &Task1,
      1);
}

void loop()
{
}
