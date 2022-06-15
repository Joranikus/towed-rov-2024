#include <ping1d.h>
#include <Wire.h>
#include "MS5837.h"
#include "SoftwareSerial.h"
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>


Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_9DOF                dof   = Adafruit_9DOF();
#include <SimpleKalmanFilter.h>
const int LEAKAGE_DETECTOR = 2;
static const uint8_t arduinoRxPin = 7; //Serial1 rx
static const uint8_t arduinoTxPin = 8; //Serial1 tx
static Ping1D ping { Serial2 };
SoftwareSerial software_serial { 20, 21 };

const int UPDATE_DEPTH = 0;
const int UPDATE_PING = 1;
const int UPDATE_PITCH = 2;
const int UPDATE_ROLL = 3;
const int UPDATE_ACCELERATION = 4;
const int UPDATE_YAW = 5;
const int UPDATE_TEMP = 6;

float depth = 0.00;
float pressure = 0.00;
float depthSeafloor = 0.00;
float temp1 = 0.00;
float q = 0.1;
float depth_rov_offset = 0.0;
float depth_beneath_rov_offset = 0.0;
boolean kalmanOnOff = true;
String dataString = "";
String s;

String a0 = "";
String a1 = "";
String a2 = "";
String a3 = "";
String d2 = "";
String d3 = "";
String d4 = "";
int D2 = 2;
int D3 = 3;
int D4 = 4;
String software_uart = "";
String i2c_variable = "";
MS5837 sensor;

int turnToSend = UPDATE_DEPTH;

boolean tempPress = false;

//charaters to define type of varible sent
char * a = "<depth:";
char * b = "<pressure:";
char * c = "<temperature:";
char* n = "<depth_beneath_rov:";
char* k = ">";
const long imu_intervall = 10;
const long transmit_interval = 20;
unsigned long previousMillis = 0;

unsigned long previous_imu_update = 0;
String inputString = "";
bool stringComplete = false;

bool reset_ardu = false;

const float ALPHA_COMP_FILTER = 0.94;
const float ALPHA_LP_FILTER = 0.05;
float calibration_gyro_X = -0.49656;
float calibration_gyro_Y = 0.01109;
float calibration_gyro_Z = -1.17348;


float roll_to_send = 0;
float pitch_to_send = 0;

float pitch = 0.00;
float roll = 0.00;
float yaw = 0;

float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float vertical_accel = 0;




int count = 0;
void initSensors()
{
  if (!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }

  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while (1);
  }
}


void set_init_values()
{
  sensors_vec_t   orientation;
  sensors_event_t accel_event, mag_event;
  accelmag.getEvent(&accel_event, &mag_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    accel_x = accel_event.acceleration.x;
    accel_y = accel_event.acceleration.y;
    accel_z = accel_event.acceleration.z;
    roll = orientation.roll;
    pitch = orientation.pitch;
  }
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {

    yaw = orientation.heading;
  }
}


void calibrate_gyro() {
  int n_cycles = 200;
  double x_tot = 0;
  double y_tot = 0;
  double z_tot = 0;
  for (int i = 1; i <= n_cycles; i++) {
    sensors_vec_t   orientation;
    sensors_event_t accel_event, mag_event, gyro_event;
    gyro.getEvent(&gyro_event);
    x_tot += gyro_event.gyro.x * 180 / PI;
    y_tot += gyro_event.gyro.y * 180 / PI;
    z_tot += gyro_event.gyro.z * 180 / PI;


    delay(10);
  }
  calibration_gyro_X = -x_tot / n_cycles;
  calibration_gyro_Y = -y_tot / n_cycles;
  calibration_gyro_Z = -z_tot / n_cycles;
}


void setup() {
  // Use 9600 bits per second to communicate with the Ping dev  ice
  Serial2.begin(9600);

  // Use built in Serial port to communicate with the Arduino IDE Serial Monitor
  Serial.begin(57600);
  while (!reset_ardu) {
  Serial.println("<SensorArduino:0>");
    String msg = Serial.readString();
    String part01 = getValue(msg, ':', 0);

    part01.replace("<","");
if (part01 == "start"){
      reset_ardu = true;
    }
  }
  Serial.println("<SensorArduino:0>");  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(LEAKAGE_DETECTOR, INPUT);
  //  software_serial.begin(4800);
  // wait until communication with the Ping device is established
  // and the Ping device is successfully initialized
//  while (!ping.initialize()) {
//    Serial.println(F("echosounder did not initialize"));
//  }
  Wire.begin();
  Wire.setClock(10000);
  while (!sensor.init()) {
    Serial.println(F("pressure not initialized!"));
    delay(1000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1029); // kg/m^3 (freshwater, 1029 for seawater)
  gyro.enableAutoRange(true);
  initSensors();
  calibrate_gyro();
  set_init_values();
  while (!Serial) {
    //wait to connect
  }

  delay(2000);

}

void loop() {
  // Checks if data is received from serial
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar != '<' ) {
      inputString += inChar;
      inputString.trim();
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '>') {
      stringComplete = true;
      translateString(inputString);
      inputString = "";
    }
  }

  //read sensor data and sends as char array one at a time
  float dt_imu = millis() - previous_imu_update;
  if (dt_imu >= imu_intervall)
    sensor_fusion(dt_imu);
  previous_imu_update = millis();

  unsigned long currentMillis = millis();
  unsigned long dt = currentMillis - previousMillis ;
  if (dt > transmit_interval) {



    // reads sensor data
    // pullup on leakage detector. Normally high

    if (digitalRead(LEAKAGE_DETECTOR) == HIGH) {
      Serial.print(F("<water_leakage: "));
      Serial.print("True");
      Serial.println(F(">"));
    }
    else if (turnToSend == UPDATE_DEPTH)
    {
      char str_depth[6]; // Depth
      for (int i = 0; i < 6; i++) {
        dtostrf(depth, 5, 2, str_depth);
      }
      char string_to_rpi[32] = "";
      strcat(string_to_rpi, a);
      strcat(string_to_rpi, str_depth);
      strcat(string_to_rpi, k);
      Serial.println(string_to_rpi);
      turnToSend = UPDATE_PING;
      previousMillis = currentMillis;
      while (!ping.update()) {
        Serial.println(F("Ping device update failed"));
      }
      byte ping_confidence = ping.confidence();
      if (ping.confidence() > 85) {
        depthSeafloor = (ping.distance() / float(1000)) + depth_beneath_rov_offset  ;
      } else {
        depthSeafloor = -1;
      }

    }
    else if (turnToSend == UPDATE_PING)
    {
      char str_depthSeafloor[8]; // Depth
      for (int i = 0; i < 8; i++) {
        dtostrf(depthSeafloor, 2, 2, str_depthSeafloor);
      }
      char string_to_rpi2[32] = "";
      strcat(string_to_rpi2, n);
      strcat(string_to_rpi2, str_depthSeafloor);
      strcat(string_to_rpi2, k);
      Serial.println(string_to_rpi2);
      turnToSend = UPDATE_PITCH;
      previousMillis = currentMillis;

    }
    else if (turnToSend == UPDATE_PITCH)
    {
      Serial.print(F("<pitch: "));
      Serial.print(pitch);
      Serial.println(F(">"));
      turnToSend = UPDATE_ROLL;
      previousMillis = currentMillis;
    }
    else if (turnToSend == UPDATE_ROLL)
    {
      Serial.print(F("<roll: "));
      Serial.print(roll);
      Serial.println(F(">"));
      turnToSend = UPDATE_DEPTH;
      previousMillis = currentMillis;
      // vertical_accel = getVerticalAcceleration(-pitch, roll, accel_x, accel_y, accel_z);
    }
    else if (turnToSend == UPDATE_ACCELERATION)
    {
      Serial.print(F("<vertical_acceleration: "));
      Serial.print(vertical_accel);
      Serial.println(F(">"));
      turnToSend = UPDATE_YAW;
      previousMillis = currentMillis;
      sensor.read();
      temp1 = sensor.temperature();
      depth = sensor.depth() + depth_rov_offset;
      pressure = sensor.pressure();


    }
    else if (turnToSend == UPDATE_YAW)
    {
      Serial.print(F("<yaw: "));
      Serial.print((yaw));
      Serial.println(F(">"));
      turnToSend = UPDATE_TEMP;
      previousMillis = currentMillis;

    }
    else if (turnToSend == UPDATE_TEMP)
    {
      //Changes between sending temperatur and pressure(since it is only used for visualisation in the GUI)
      if (tempPress) {
        char str_pressure[6]; // Depth
        for (int i = 0; i < 6; i++) {
          dtostrf(pressure, 4, 2, str_pressure);
        }
        char string_to_rpi3[32] = "";
        strcat(string_to_rpi3, b);
        strcat(string_to_rpi3, str_pressure);
        strcat(string_to_rpi3, k);
        Serial.println(string_to_rpi3);
        tempPress = false;
      } else {
        char str_temp[4]; // Depth
        for (int i = 0; i < 4; i++) {
          dtostrf(temp1, 1, 1, str_temp);
        }
        char string_to_rpi4[32] = "";
        strcat(string_to_rpi4, c);
        strcat(string_to_rpi4, str_temp);
        strcat(string_to_rpi4, k);
        Serial.println(string_to_rpi4);
        tempPress = true;
      }
      previousMillis = currentMillis;
      turnToSend = UPDATE_DEPTH;

    }

    previousMillis = millis();
  }


}


void sensor_fusion(float dt) {
  dt = dt * 0.001;
  sensors_vec_t   orientation;
  sensors_event_t accel_event, mag_event, gyro_event;

  gyro.getEvent(&gyro_event);
  /* Get a new sensor event */
  accelmag.getEvent(&accel_event, &mag_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {

    double alpha = 0.9;
    double gyro_pitch = gyro_event.gyro.y * 180 / PI + calibration_gyro_X;
    double gyro_roll = gyro_event.gyro.x * 180 / PI + calibration_gyro_X;

    pitch = complementary_filter(alpha, pitch, gyro_pitch, orientation.pitch, dt);
    roll = complementary_filter(alpha, roll, gyro_roll, orientation.roll, dt);


  }
}


void updateImuData(unsigned long dt_imu) {
  dt_imu = dt_imu * 0.001;
  float pitch_acc = pitch;
  float roll_acc = roll;
  float yaw_acc = yaw;

  sensors_vec_t   orientation;
  sensors_event_t accel_event, mag_event, gyro_event;
  double pitch_gyro = -(gyro_event.gyro.y + calibration_gyro_Y) * 180 / PI;
  double roll_gyro =  -(gyro_event.gyro.x + calibration_gyro_X) * 180 / PI;
  double yaw_gyro = -(gyro_event.gyro.z + calibration_gyro_Z) * 180 / PI;


  accelmag.getEvent(&accel_event, &mag_event);
  if (dof.accelGetOrientation(&accel_event, &orientation) && dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    pitch_acc = orientation.pitch;
    roll_acc = orientation.roll;
    yaw_acc = orientation.heading;
    accel_x  = low_pass_filter(ALPHA_LP_FILTER, accel_event.acceleration.x, accel_x);
    accel_y  = low_pass_filter(ALPHA_LP_FILTER, accel_event.acceleration.y, accel_y);
    accel_z  = low_pass_filter(ALPHA_LP_FILTER, accel_event.acceleration.z, accel_z);


  }
  pitch = complementary_filter(ALPHA_COMP_FILTER, pitch, pitch_gyro, pitch_acc, dt_imu);
  roll = complementary_filter(ALPHA_COMP_FILTER, roll, roll_gyro, roll_acc, dt_imu);
  //yaw = complementary_filter(ALPHA_COMP_FILTER, yaw, yaw_gyro, yaw_acc, dt_imu);
  yaw = 0;
}


float complementary_filter(float alpha, float prev_angle, float gyro_value, float accel_value, float dt) {
  float filtered_value  = alpha * (prev_angle + gyro_value * dt) + (1 - alpha) * accel_value;
  return filtered_value;
}


float low_pass_filter(float alpha, float new_val, float prev_val) {
  float filtered_value  = alpha * new_val + (1 - alpha) * prev_val;
  return filtered_value;
}


float getVerticalAcceleration(float roll_angle, float pitch_angle, float accel_x, float accel_y, float accel_z) {

  //the imu is rotated in the rov
  pitch_angle = pitch_angle * PI / 180; //degrees to radians
  roll_angle = roll_angle * PI / 180;
  roll_angle += PI / 2; //rotate measured angles
  pitch_angle += PI / 2;
  float accel_vertical_x = accel_x * cos(pitch_angle) * sin(roll_angle) /
                           sqrt(cos(roll_angle) * cos(roll_angle) * sin(pitch_angle) * sin(pitch_angle) + sin(roll_angle) * sin(roll_angle));
  float accel_vertical_y = accel_y * sin(pitch_angle) * cos(roll_angle) /
                           sqrt(cos(roll_angle) * cos(roll_angle) * sin(pitch_angle) * sin(pitch_angle) + sin(roll_angle) * sin(roll_angle));
  float accel_vertical_z = accel_z * sin(pitch_angle) * sin(roll_angle) /
                           sqrt(cos(roll_angle) * cos(roll_angle) * sin(pitch_angle) * sin(pitch_angle) + sin(roll_angle) * sin(roll_angle));
  return -accel_vertical_y - accel_vertical_x + accel_vertical_z;
}


//separates string message
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


//Translate incomming messages and check if a known message
void translateString(String s) {
  String part01 = getValue(s, ':', 0);
  String part02 = getValue(s, ':', 1);
  if (part01.equals("depth_rov_offset")) {

    responde("depth_rov_offset:True");
    depth_rov_offset = part02.toFloat();

  }
  else if (part01.equals("depth_beneath_rov_offset")) {
    depth_beneath_rov_offset = part02.toFloat();
    responde("depth_beneath_rov_offset:True");
  }
  else if (part01.equals("A0")) {
    a0 = part02;
    responde("A0:True");
  }
  else if (part01.equals("A1")) {
    a1 = part02;
    responde("A1:True");
  }
  else if (part01.equals("A2")) {
    a2 = part02;
    responde("A2:True");
  }
  else if (part01.equals("A3")) {
    a3 = part02;
    responde("A3:True");
  }
  else if (part01.equals("D2")) {
    d2 = part02;
    responde("D2:True");
  }
  else if (part01.equals("D3")) {
    d3 = part02;
    responde("D3:True");
  }
  else if (part01.equals("D4")) {
    d4 = part02;
    responde("D4:True");
  }
  else if (part01.equals("Software_uart")) {
    software_uart = part02;
    responde("Software_uart:True");
  }
}


void responde(String response) {
  Serial.println("<" + response + ">");
}
