#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <math.h>
#include <ESP32Servo.h>
#include <MadgwickRileyCustom.h>

Madgwick filter;

const char *ssid = "Riley_Parachute_ESP32_MPU6050_Server";
const char *password = "12345678";

Servo mortarServo;
int mortarServoPin = 18;

Servo payloadServo;
int payloadServoPin = 19;

WebServer server(80);
WebSocketsServer webSocket(81);
Adafruit_MPU6050 mpu;

unsigned long lastUpdateTime = 0;
const int updateInterval = 5;

bool useROffset = false;
bool useQuaternions = false;
bool useAbsoluteAccel = true;
bool useKnownOffsetSpinCorrection = false;

float absoluteAccelThreshold = 2.5;
float knownOffset = 0;

bool calibrationDone = false;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float ax_sum = 0, ay_sum = 0, az_sum = 0;

bool fallDetected = false;
bool parachuteDeployed = false;
String dataLog = "Time,AX,AY,AZ,|A|,GX,GY,GZ,CalibrationLoss,FallDetected\n";

void calibrateSensors();
void sendSensorData(unsigned long currentMillis);

const int MAX_SAMPLES = 200;
float ax_data[MAX_SAMPLES], ay_data[MAX_SAMPLES], az_data[MAX_SAMPLES];
float gx_data[MAX_SAMPLES], gy_data[MAX_SAMPLES], gz_data[MAX_SAMPLES];
int sample_count = 0;

const float LEARNING_RATE = 0.01;
const int MAX_ITERS = 100;

float rx = 0.0, ry = 0.0, rz = 0.0;
float calibrationLoss = 0.0;

void setupWiFi()
{
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void handleRoot()
{
  File file = SPIFFS.open("/index.html", "r");
  if (!file)
  {
    server.send(500, "text/plain", "Failed to load index.html");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}

void handleAppJS()
{
  File file = SPIFFS.open("/scripts/app.js", "r");
  if (!file)
  {
    server.send(404, "text/plain", "app.js not found");
    return;
  }
  server.streamFile(file, "application/javascript");
  file.close();
}

void handleChartJS()
{
  File file = SPIFFS.open("/scripts/chart.js", "r");
  if (!file)
  {
    server.send(404, "text/plain", "chart.js not found");
    return;
  }
  server.streamFile(file, "application/javascript");
  file.close();
}

void handleStyleCSS()
{
  File file = SPIFFS.open("/styles/style.css", "r");
  if (!file)
  {
    server.send(404, "text/plain", "style.css not found");
    return;
  }
  server.streamFile(file, "text/css");
  file.close();
}

void handleSvg(String svgName)
{
  String path = "/svg/" + svgName + ".svg";
  File file = SPIFFS.open(path, "r");
  if (!file)
  {
    Serial.println("SVG file not found: " + path);
    server.send(404, "text/plain", "SVG file not found");
    return;
  }
  server.streamFile(file, "image/svg+xml");
  file.close();
}

void handleDownloadData()
{
  File file = SPIFFS.open("/dataLog.csv", "r");
  if (!file)
  {
    server.send(404, "text/plain", "No data file found");
    return;
  }

  server.streamFile(file, "text/csv");
  file.close();

  SPIFFS.remove("/dataLog.csv");

  file = SPIFFS.open("/dataLog.csv", "w");
  if (file)
  {
    file.print("Time,AX,AY,AZ,|A|,GX,GY,GZ,CalibrationLoss,FallDetected\n");
    file.close();
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (type == WStype_CONNECTED)
  {
    Serial.println("Client connected.");
    unsigned long esp32Time = millis();
    String timeSyncMessage = "{\"timeSync\": " + String(esp32Time) + "}";
    webSocket.sendTXT(num, timeSyncMessage);
  }
  else if (type == WStype_TEXT)
  {
    String message = String((char *)payload);
    if (message == "resetFall")
    {
      mortarServo.write(
      fallDetected = false;
      parachuteDeployed = false;
      Serial.println("Fall Status Reset");
    }
    else if (message == "deployParachute")
    {
      fallDetected = true;
      parachuteDeployed = false;
      Serial.println("Parachute Deployed Manually");
    }
    else if (message == "deployPayload")
    {
      payloadServo.write(0);
      Serial.println("Payload Deployed Manually");
    }
    else if (message == "recalibrate")
    {
      sample_count = 0;
      calibrationDone = false;
      rx = ry = rz = 0.0;
      calibrateSensors();
      Serial.println("Recalibration Initiated");
    }
    else if (message.startsWith("setBeta:"))
    {
      String betaValue = message.substring(8);
      float newBeta = betaValue.toFloat();
      if (newBeta > 0 && newBeta < 10)
      {
        filter.beta = newBeta;
        Serial.print("Updated beta: ");
        Serial.println(filter.beta);
      }
    }
    else if (message == "toggleR:on")
    {
      useROffset = true;
      Serial.println("r offset enabled");
    }
    else if (message == "toggleR:off")
    {
      useROffset = false;
      Serial.println("r offset disabled");
    }
    else if (message == "toggleQuat:on")
    {
      useQuaternions = true;
      Serial.println("quaternion gravity correction enabled");
    }
    else if (message == "toggleQuat:off")
    {
      useQuaternions = false;
      Serial.println("quaternion gravity correction disabled");
    }
    else if (message == "toggleAbsolute:on")
    {
      useAbsoluteAccel = true;
      Serial.println("Using raw absolute acceleration for fall detection");
    }
    else if (message == "toggleAbsolute:off")
    {
      useAbsoluteAccel = false;
      Serial.println("Using quaternion-corrected acceleration for fall detection");
    }
    else if (message.startsWith("setAbsoluteThreshold:"))
    {
      String val = message.substring(21);
      absoluteAccelThreshold = val.toFloat();
      Serial.print("Updated absoluteAccelThreshold: ");
      Serial.println(absoluteAccelThreshold);
    }
    else if (message == "toggleKnownOffsetSpinCorrection:on")
    {
      useKnownOffsetSpinCorrection = true;
      Serial.println("Using known offset correction");
    }
    else if (message == "toggleKnownOffsetSpinCorrection:off")
    {
      useKnownOffsetSpinCorrection = false;
      Serial.println("Not using known offset correction");
    }
    else if (message.startsWith("setKnownOffset:"))
    {
      String val = message.substring(15);
      knownOffset = val.toFloat();
      Serial.print("Updated knownOffset: ");
      Serial.println(knownOffset);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  setupWiFi();

  if (!SPIFFS.begin(true))
  {
    Serial.println("Failed to mount SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");

  filter.begin(100);

  filter.beta = 0.1f;

  if (SPIFFS.exists("/dataLog.csv"))
  {
    SPIFFS.remove("/dataLog.csv");
    Serial.println("Old data log deleted.");
  }

  File file = SPIFFS.open("/dataLog.csv", "w");
  if (file)
  {
    file.println("Time,AX,AY,AZ,|A|,GX,GY,GZ,CalibrationLoss,FallDetected");
    file.close();
    Serial.println("New data log initialized with headers.");
  }
  else
  {
    Serial.println("Failed to create new data log.");
  }

  server.on("/", handleRoot);
  server.on("/scripts/app.js", HTTP_GET, handleAppJS);
  server.on("/scripts/chart.js", HTTP_GET, handleChartJS);
  server.on("/styles/style.css", HTTP_GET, handleStyleCSS);
  server.on("/downloadData", HTTP_GET, handleDownloadData);
  server.on("/svg/parachute.svg", HTTP_GET, []()
            { handleSvg("parachute"); });
  server.on("/svg/reset.svg", HTTP_GET, []()
            { handleSvg("reset"); });
  server.on("/svg/download.svg", HTTP_GET, []()
            { handleSvg("download"); });
  server.on("/svg/connection.svg", HTTP_GET, []()
            { handleSvg("connection"); });
  server.on("/svg/plane.svg", HTTP_GET, []()
            { handleSvg("plane"); });
  server.on("/svg/deploy.svg", HTTP_GET, []()
            { handleSvg("deploy"); });
  server.on("/svg/chip.svg", HTTP_GET, []()
            { handleSvg("chip"); });

  server.begin();

  if (!mpu.begin())
  {
    Serial.println("MPU6050 initialization failed!");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  calibrateSensors();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  mortarServo.attach(mortarServoPin);
  mortarServo.write(65);

  payloadServo.attach(payloadServoPin);
  payloadServo.write(85);
}

void loop()
{
  server.handleClient();
  webSocket.loop();
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateTime >= updateInterval)
  {
    lastUpdateTime = currentMillis;
    sendSensorData(currentMillis);
  }
  if (fallDetected == true && parachuteDeployed == false)
  {
    mortarServo.write(115);
    parachuteDeployed = true;
  }
}

void calibrateSensors()
{
  const int calibrationSamples = 100;

  ax_sum = 0;
  ay_sum = 0;
  az_sum = 0;

  for (int i = 0; i < calibrationSamples; i++)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;

    collectSpinSample(a.acceleration.x, a.acceleration.y, a.acceleration.z,
                      g.gyro.x, g.gyro.y, g.gyro.z);

    delay(10);
  }

  ax_offset = ax_sum / calibrationSamples;
  ay_offset = ay_sum / calibrationSamples;
  az_offset = az_sum / calibrationSamples;

  if (sample_count > 10)
  {
    estimateOffsetGD(rx, ry, rz);
  }

  calibrationDone = true;
  Serial.println("Calibration complete");
}

void collectSpinSample(float ax, float ay, float az, float gx, float gy, float gz)
{
  int index = sample_count % MAX_SAMPLES;
  ax_data[index] = ax;
  ay_data[index] = ay;
  az_data[index] = az;
  gx_data[index] = gx;
  gy_data[index] = gy;
  gz_data[index] = gz;
  sample_count++;
}

void crossProduct(float *a, float *b, float *result)
{
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

float calculateLoss(float rx_val, float ry_val, float rz_val)
{
  float totalLoss = 0.0;
  for (int i = 0; i < sample_count; i++)
  {
    float omega[3] = {gx_data[i], gy_data[i], gz_data[i]};
    float r[3] = {rx_val, ry_val, rz_val};
    float oxr[3], acc_pred[3];
    crossProduct(omega, r, oxr);
    crossProduct(omega, oxr, acc_pred);
    float error_x = acc_pred[0] - ax_data[i];
    float error_y = acc_pred[1] - ay_data[i];
    float error_z = acc_pred[2] - az_data[i];
    totalLoss += (error_x * error_x + error_y * error_y + error_z * error_z);
  }
  return sample_count > 0 ? totalLoss / sample_count : 0.0;
}

void estimateOffsetGD(float &rx, float &ry, float &rz)
{
  float rxi = rx, ryi = ry, rzi = rz;
  float delta = 0.001;

  for (int iter = 0; iter < MAX_ITERS; iter++)
  {
    float loss = calculateLoss(rxi, ryi, rzi);
    float loss_dx = calculateLoss(rxi + delta, ryi, rzi);
    float loss_dy = calculateLoss(rxi, ryi + delta, rzi);
    float loss_dz = calculateLoss(rxi, ryi, rzi + delta);

    float grad_rx = (loss_dx - loss) / delta;
    float grad_ry = (loss_dy - loss) / delta;
    float grad_rz = (loss_dz - loss) / delta;

    rxi -= LEARNING_RATE * grad_rx;
    ryi -= LEARNING_RATE * grad_ry;
    rzi -= LEARNING_RATE * grad_rz;
  }

  calibrationLoss = calculateLoss(rxi, ryi, rzi);
  rx = rxi;
  ry = ryi;
  rz = rzi;

  Serial.print("Estimated Offset: ");
  Serial.print(rx);
  Serial.print(", ");
  Serial.print(ry);
  Serial.print(", ");
  Serial.println(rz);
}

int sustainedFallCounter = 0;
const int requiredFallSamples = 1;
const float FALL_LOWER_THRESHOLD = 7.0;
const float FALL_UPPER_THRESHOLD = 11.0;

void sendSensorData(unsigned long currentMillis)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gx_rad = g.gyro.x;
  float gy_rad = g.gyro.y;
  float gz_rad = g.gyro.z;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  static unsigned long lastMicros = micros();
  unsigned long nowMicros = micros();
  float deltaT = (nowMicros - lastMicros) / 1e6f;
  lastMicros = nowMicros;

  filter.updateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, deltaT);

  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  float ax_corrected = ax;
  float ay_corrected = ay;
  float az_corrected = az;

  if (useQuaternions)
  {
    float g_x = 2 * (q1 * q3 - q0 * q2) * 9.81;
    float g_y = 2 * (q0 * q1 + q2 * q3) * 9.81;
    float g_z = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 9.81;

    ax_corrected -= g_x;
    ay_corrected -= g_y;
    az_corrected -= g_z;
  }

  float a_absolute = sqrt(ax_corrected * ax_corrected + ay_corrected * ay_corrected + az_corrected * az_corrected);

  float a_absolute_corrected = a_absolute;

  float gx_corrected = gx_rad;
  float gy_corrected = gy_rad;
  float gz_corrected = gz_rad;

  if (useKnownOffsetSpinCorrection)
  {
    float g_absolute = sqrt(gx_rad * gx_rad + gy_rad * gy_rad + gz_rad * gz_rad);

    float a_centrifugal = knownOffset * g_absolute * g_absolute;

    a_absolute_corrected = a_absolute - a_centrifugal;
  }

  if (useROffset && sample_count > 10)
  {
    float omega[3] = {gx_corrected, gy_corrected, gz_corrected};
    float r[3] = {rx, ry, rz};
    float oxr[3], acc_pred[3];
    crossProduct(omega, r, oxr);
    crossProduct(omega, oxr, acc_pred);

    float error_x = acc_pred[0] - ax_corrected;
    float error_y = acc_pred[1] - ay_corrected;
    float error_z = acc_pred[2] - az_corrected;

    calibrationLoss = 0.9 * calibrationLoss + 0.1 * (error_x * error_x + error_y * error_y + error_z * error_z);
  }

  bool fallConditionMet = false;

  if (useAbsoluteAccel)
  {
    float rawAccelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    fallConditionMet = rawAccelMagnitude < absoluteAccelThreshold;
  }
  else
  {
    fallConditionMet = (a_absolute >= FALL_LOWER_THRESHOLD && a_absolute <= FALL_UPPER_THRESHOLD);
  }

  if (fallConditionMet)
  {
    sustainedFallCounter++;
  }
  else
  {
    sustainedFallCounter = 0;
  }

  if (sustainedFallCounter >= requiredFallSamples && !parachuteDeployed)
  {
    fallDetected = true;
    mortarServo.write(115);
    Serial.println("Fall Detected and Parachute Deployed");
    parachuteDeployed = true;
  }

  float seconds = currentMillis / 1000.0;
  String dataEntry = String(seconds) + "," +
                     String(ax_corrected) + "," +
                     String(ay_corrected) + "," +
                     String(az_corrected) + "," +
                     String(a_absolute) + "," +
                     String(gx_corrected) + "," +
                     String(gy_corrected) + "," +
                     String(gz_corrected) + "," +
                     String(calibrationLoss) + "," +
                     String(fallDetected) + "," +
                     String(q0) + "," + String(q1) + "," + String(q2) + "," + String(q3) + "\n";

  File file = SPIFFS.open("/dataLog.csv", "a");
  if (file)
  {
    file.print(dataEntry);
    file.close();
  }

  StaticJsonDocument<350> jsonDoc;
  jsonDoc["time"] = seconds;
  jsonDoc["ax"] = ax_corrected;
  jsonDoc["ay"] = ay_corrected;
  jsonDoc["az"] = az_corrected;
  jsonDoc["aa"] = a_absolute_corrected;
  jsonDoc["gx"] = gx_corrected;
  jsonDoc["gy"] = gy_corrected;
  jsonDoc["gz"] = gz_corrected;
  jsonDoc["cl"] = calibrationLoss;
  jsonDoc["fall"] = fallDetected;
  jsonDoc["q0"] = q0;
  jsonDoc["q1"] = q1;
  jsonDoc["q2"] = q2;
  jsonDoc["q3"] = q3;
  jsonDoc["rx"] = rx;
  jsonDoc["ry"] = ry;
  jsonDoc["rz"] = rz;
  jsonDoc["roll"] = roll;
  jsonDoc["pitch"] = pitch;
  jsonDoc["yaw"] = yaw;

  String jsonString;
  serializeJson(jsonDoc, jsonString);
  webSocket.broadcastTXT(jsonString);

  static unsigned long lastOffsetUpdate = 0;
  int validSamples = min(sample_count, MAX_SAMPLES);
  if (millis() - lastOffsetUpdate >= 500 && validSamples >= 50)
  {
    estimateOffsetGD(rx, ry, rz);
    lastOffsetUpdate = millis();
  }
}
