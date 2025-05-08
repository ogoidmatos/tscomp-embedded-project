#include <Arduino_LSM9DS1.h>
#include <Wire.h>
extern TwoWire Wire1;

#define ECHO 12
#define TRIG 2
#define SENSOR_ADDRESS 0x6b
#define CTRL_ACCEL 0x20
#define ACCEL_OUT_X 0x28
#define CTRL_REG8 0x22
#define INFRARED A0
#define DIST_CENTER 2.5
#define RED 22
#define BLUE 24
#define GREEN 23


volatile bool signalOn = false;
volatile int startTime;
volatile int endTime;
bool calibrated = false;
bool needCalibration = true;
int irCutoff = 0;

volatile long duration;
volatile int count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(12), ultrassoundTime, CHANGE);
  initWire();
  digitalWrite(RED, HIGH);
  delay(1000);
  digitalWrite(GREEN, HIGH);
  delay(1000);
  digitalWrite(BLUE, HIGH);
  delay(1000);
}

void loop() {
  float x, y, z;
  float distance;
  double distanceIR;

  float weights[5];
  double sensorAngle;
  int32_t irVoltage = 0;
  duration = 0;
  count = 0;

  for (int i = 0; i < 100; i++) {
    // Obtain distance with ultrassound sensor
    if (!signalOn) {
      digitalWrite(TRIG, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
    }
    getAcceleration(&x, &y, &z);

    irVoltage += analogRead(INFRARED);
  }
  irVoltage /= 100;

  distance = duration / count * (0.034 / 2) + 1;  // 1cm of offset
  Serial.print(distance);

  // IR sensor calibration
  if (needCalibration) {
    calibration(distance, irVoltage, weights);
  }

  // IR sensor usage
  if (calibrated && distance < 10) {
    distanceIR = modelIR(irVoltage, 0) * weights[0] + modelIR(irVoltage, 1) * weights[1] + modelIR(irVoltage, 2) * weights[2] + modelIR(irVoltage, 3) * weights[3] + modelIR(irVoltage, 4) * weights[4];
  }

  // verify the need to calibrate
  if (abs(distance - distanceIR) < 0.3) {
    needCalibration = false;
  } else if (distanceIR > 1 && distance < 10 && distance > 5) {
    needCalibration = true;
    calibrated = false;
  }

  // Determine IR Voltage Cutoff
  if (distance > 30) {
    irCutoff = analogRead(INFRARED);
  }

  Serial.print('\t');
  Serial.print(distanceIR);

  sensorAngle = atan(y / z);  // angle in radians

  distance = distance * cos(sensorAngle);

  distanceIR = distanceIR * cos(sensorAngle);

  Serial.print('\t');
  Serial.print(irVoltage);
  Serial.print('\t');
  Serial.print(sensorAngle);
  Serial.print('\t');

  Serial.print("Final Distance: ");
  if (distance > 10 || sensorAngle > 0.4 || (distance > 5 && sensorAngle > 0.1) || !calibrated) {
    Serial.println(distance);
  } else if (irVoltage < 1020) {
    Serial.println(distanceIR);
  } else {
    Serial.println("Too close, IR sensor saturated");
  }
}

void ultrassoundTime() {
  if (!signalOn) {
    startTime = micros();
    signalOn = true;
  } else {
    endTime = micros();
    signalOn = false;
    duration = duration + endTime - startTime;
    count++;
  }
}

int initWire() {
  Wire1.begin();

  Wire1.beginTransmission(SENSOR_ADDRESS);
  Wire1.write(CTRL_REG8);
  Wire1.write(0x05);
  Wire1.endTransmission();

  delay(10);  // Wait for reset of sensor

  Wire1.beginTransmission(SENSOR_ADDRESS);
  Wire1.write(CTRL_ACCEL);
  Wire1.write(0x60);
  Wire1.endTransmission();
}

void getAcceleration(float* x, float* y, float* z) {
  Wire1.beginTransmission(SENSOR_ADDRESS);
  Wire1.write(ACCEL_OUT_X);
  Wire1.endTransmission();

  Wire1.requestFrom(SENSOR_ADDRESS, 6);

  int16_t data[3];
  uint8_t* temp = (uint8_t*)data;

  for (int i = 0; i < sizeof(data); i++) {
    temp[i] = Wire1.read();
  }

  *x = data[0] * 2.0 / 32768.0;
  *y = data[1] * 2.0 / 32768.0;
  *z = data[2] * 2.0 / 32768.0;
}

void calibration(float distance, int irVoltage, float* weights) {
  float distIR[5];
  float errorIR[5];

  // calibrates if ultra sound distance is between 8 cm and 9 cm and the object in front reflects light
  // We say that an object reflects light if, in calibration distance, the voltage from the IR receiver is higher than without an object in front
  if (10 > distance && distance > 5 && irVoltage != 0) {
    calibrated = true;

    // Calculate the distance using 4 different models, corresponding to different colors and the errors relative to each model
    for (int i = 0; i < 5; i++) {
      distIR[i] = modelIR(irVoltage, i);
      errorIR[i] = abs(distance - distIR[i]);
    }

    // Find model with the lowest error
    int min_value = errorIR[0];
    int min_idx = 0;
    for (int i = 1; i < 5; i++) {
      if (errorIR[i] < min_value) {
        min_value = errorIR[i];
        min_idx = i;
      }
    }

    // Turn on leds depending on which model is closest to ultra sound distance
    // If models 0 or 5 are the closest (models 0 and 5 represent the darkest objects) we turn on the blue led
    if (min_idx == 0 || min_idx == 5) {
      digitalWrite(BLUE, LOW);
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, HIGH);
    }
    // If model 2 is the closest (model 2 represents objects that are neither dark nor bright) we turn on a yellow light
    else if (min_idx == 2) {
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
    }
    // If model 1 or 3 are the closest (both models represent bright objects) we turn on a white light
    else {
      digitalWrite(BLUE, LOW);
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
    }
    // Find model with the second lowest error
    min_value = 100;
    int min_idx2 = 0;
    for (int i = 0; i < 5; i++) {
      if (errorIR[i] < min_value && i != min_idx) {
        min_value = errorIR[i];
        min_idx2 = i;
      }
    }
    // Calculate weights for each model (the two with highest errors have 0 weight, the other two have weights proporcional to their proximity to the distance obtained with ultra sound)
    // This calibration process ensures that the distance given by the IR sensor is the same as the ultra sound sensor, for the distance chosen for calibration
    for (int i = 0; i < 5; i++) {
      weights[i] = 0;
    }
    weights[min_idx] = abs((distance - distIR[min_idx2]) / (distIR[min_idx] - distIR[min_idx2]));
    weights[min_idx2] = 1 - weights[min_idx];
  }
}

// Function returns the current model for translating the voltage from the IR receiver into distance, according to the color model chosen.
double modelIR(int voltage, int model) {
  switch (model) {
    case 0:
      return 732.37 * pow(voltage, -1.108);
    case 1:
      return 562.29 * pow(voltage, -0.802);
    case 2:
      return 182.4 * pow(voltage, -0.655);
    case 3:
      return 354.72 * pow(voltage, -0.734);
    case 4:
      return 336.71 * pow(voltage, -0.987);
  }
}