/**
  This programs control the wing angles, by calculating a desired angle with PID controllers, and sets the motors position.
   Author: Slepe-ROV 2021
   Edit by: TowedROV 2022
*/

#include <PID_v1.h>
#include <SoftwareSerial.h>
#include  "WingStepper.h"

//Pins connected to the Teensy
const int DIR_PIN_SB    = 2;
const int STEP_PIN_SB   = 3;
const int SENSOR_PIN_SB = 4;

const int DIR_PIN_PORT    = 5;
const int STEP_PIN_PORT   = 6;
const int SENSOR_PIN_PORT = 7;

// class for stepper
WingStepper stepper_sb(STEP_PIN_SB, DIR_PIN_SB, SENSOR_PIN_SB);
WingStepper stepper_port(STEP_PIN_PORT, DIR_PIN_PORT, SENSOR_PIN_PORT);

// calibration status
bool cal_status_sb = false;
bool cal_status_port = false;

char charIn = ' ';
char lastCharIn;

//Wing positions
double target_angle_sb = 0;
double target_angle_port = 0;

//Feedback GUI
String data_string;

//Switch case modes
const int CALIBRATE_STATE = 0;
const int MANUAL_STATE = 1;
const int AUTO_DEPTH_STATE = 2;
int state = MANUAL_STATE;

//Flag
boolean testing = false;
bool boolWingPos;

// LIMITS
int max_wing_angle = 360;
int max_stepper_pos = max_wing_angle * stepper_sb.get_angle_to_step_const();
int min_stepper_pos = -max_stepper_pos;

double max_pid_output = 15;
double min_sea_floor_distance = 10;
double max_trim = 15;

//SET POINTS
double set_point_depth = 0; //set point Depth/Sea floor
double set_point_roll = 0; //set point TRIM

const int pitch_readings_length = 50;
double pitch_readings[pitch_readings_length];
double pitch_compensation = 0;
double minimum_change_pitch = 3;
// Sensors
double depth = 0, echo_distance, roll = 0, pitch = 0;
// Controller outputs
double wing_angle = 0, trim_angle = 0; //Outputs controllers
double manual_wing_pos = 0;

//PID params
double pid_depth_p = 0, pid_depth_i = 0, pid_depth_d = 0;
double pid_roll_p = 0, pid_roll_i = 0, pid_roll_d = 0;

//Timer
unsigned long time_intervall = 50;
unsigned long last_update_wing_pos = 0;

// PID controllers
PID pid_depth = PID(&depth, &wing_angle, &set_point_depth, pid_depth_p, pid_depth_i, pid_depth_d, REVERSE);
PID pid_trim = PID(&roll, &trim_angle, &set_point_roll, pid_roll_p, pid_roll_i, pid_roll_i, DIRECT);


// moves both stepper towards target angle
void step_both() {
  // step if stepper is at wrong angle
  if (target_angle_sb != stepper_sb.get_angle()) {
    stepper_sb.rotate(target_angle_sb);
  }
  if (target_angle_port != stepper_port.get_angle()) {
    stepper_port.rotate(target_angle_port);
  }
}

/**
  Set a new target mode
   @param The desired target mode.
*/
bool setTargetMode(int newTargetMode, int wing_pos = 0) {
  bool mode_set = false;
  if (newTargetMode == MANUAL_STATE) {
    pid_depth.SetMode(MANUAL);
    pid_trim.SetMode(MANUAL);
    state = MANUAL_STATE;
    manual_wing_pos = wing_pos;
    mode_set = true;
  }

  else if (newTargetMode == AUTO_DEPTH_STATE) {
    state = AUTO_DEPTH_STATE;
    set_point_depth = depth;
    pid_depth.SetMode(AUTOMATIC);
    pid_trim.SetMode(AUTOMATIC);
    pid_depth.SetTunings(pid_depth_p, pid_depth_i, pid_depth_d);
    pid_trim.SetTunings(pid_roll_p, pid_roll_i, pid_roll_d);
    mode_set = true;
  }
  return mode_set;
}


/**
  Maps the values, returning a double TODO: Remove, not used
  only manual and depth mode is included at this stage.
   @param value is the value that is being mapped, in and outputs sets the range.
*/
double mapf(double value, double minIn, double maxIn, double minOut, double maxOut) {
  double x = (value - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
  return constrain(x, minOut, maxOut);
}


void compensateWingToPitch() {
  target_angle_sb -= pitch_compensation;
  target_angle_port -= pitch_compensation;
}


/**
  Trims the wing angles. compensate by increasing the trim on the opposite side if the max value is reached.
*/
void trimWingPos() {
  if (wing_angle + trim_angle > max_wing_angle) {

    target_angle_sb = max_wing_angle;
    target_angle_port = max_wing_angle - 2 * trim_angle;
  }
  else if (wing_angle - trim_angle < -max_wing_angle) {

    target_angle_sb = -max_wing_angle + 2 * trim_angle;
    target_angle_port = -max_wing_angle;
  } else {
    target_angle_sb = wing_angle - trim_angle;
    target_angle_port = wing_angle + trim_angle;
  }
  target_angle_sb = constrain(target_angle_sb, -max_wing_angle, max_wing_angle);
  target_angle_port = constrain(target_angle_port, -max_wing_angle, max_wing_angle);
}


double get_average_array(double values[], int length_array) {
  double summed_values = 0;
  for (int i = 0; i < length_array; i++) {
    summed_values += values[i];
  }
  double average_value = summed_values / length_array;
  return average_value;
}


void shift_array_left(double array_to_edit[], int length_array, double new_val) {
  for (int i = 1; i < length_array; i++) {
    array_to_edit[i - 1] = array_to_edit[i];
  }
  array_to_edit[length_array - 1] = new_val;
}


void set_pitch_compensation() {
  double average_pitch = get_average_array(pitch_readings, pitch_readings_length);
  if (abs(average_pitch - pitch_compensation) >= minimum_change_pitch) {
    pitch_compensation = average_pitch;
  }
}


/**
  Updates the GUI with the actual wing positions.
*/
void updateWingPosGUI(double pos_sb, double pos_port) {

  //print angle for port stepper
  String dataToSend = "<wing_pos_port:";
  dataToSend.concat(stepper_port.get_angle() + pitch); //TODO why send pitch here?
  dataToSend.concat(">");
  Serial.println(dataToSend);

  //print angle for starbord stepper
  dataToSend = "<wing_pos_sb:";
  dataToSend.concat(stepper_sb.get_angle() + pitch);
  dataToSend.concat(">");
  Serial.println(dataToSend);

}


/**
  Split the incoming strings, part01 is the header/command while part02 is the value.
   @s is a string from the raspberry pi. ROV main computer.
*/
void translateString(String s) {

  String part01 = getValue(s, ':', 0);
  String part02 = getValue(s, ':', 1);
  double double_part02 = part02.toFloat(); // toDouble does not work with teensy 4.0

  if (part01.equals("reset")) {
    state = CALIBRATE_STATE;
    cal_status_sb = false;
    cal_status_port = false;
  }

  else if (part01.equals("auto_mode")) {
    if (part02.equals("True")) {
      setTargetMode(AUTO_DEPTH_STATE);
      Serial.println("<auto_mode:True>");
    } else if (part02.equals("False")) {
      setTargetMode(MANUAL_STATE);
      Serial.println("<auto_mode:True>");
      manual_wing_pos = (int) target_angle_sb; //TODO why int and not double/float?
    } else {
      Serial.println("<auto_mode:False>");
    }
  }

  else if (part01.equals("manual_wing_pos")) {
    if (double_part02 < max_wing_angle && double_part02 > -max_wing_angle) {
      manual_wing_pos = double_part02;
      Serial.println("<manual_wing_pos:True>");
    } else {
      Serial.println("<manual_wing_pos:False>");
    }
  }

  //SENSORS
  else if (part01.equals("emergency_surface")) {
    setTargetMode(MANUAL_STATE, max_wing_angle);
    state = MANUAL_STATE;
    Serial.println("<emergency_surface:True>");
  }

  else if (part01.equals("depth")) {
    depth = part02.toInt();
  }

  else if (part01.equals("roll")) {
    roll = part02.toInt();
  }

  else if (part01.equals("pitch")) {
    double pitch = double_part02;
    depth = pitch;
    shift_array_left(pitch_readings, pitch_readings_length, pitch);
    set_pitch_compensation();
  }

  //SET POINTS
  else if (part01.equals("set_point_depth")) {
    set_point_depth = double_part02;
    Serial.println("<set_point_depth:True>");
  }

  //PID DEPTH
  else if (part01.equals("pid_depth_p")) {
    pid_depth_p = double_part02;
    if (pid_depth_p >= 0) {
      pid_depth.SetTunings(pid_depth_p, pid_depth_i, pid_depth_d);
      Serial.println("<pid_depth_p:True>");
    } else {
      Serial.println("<pid_depth_p:False>");
    }
  }

  else if (part01.equals("pid_depth_i")) {
    pid_depth_i = double_part02;
    if (pid_depth_i >= 0) {
      pid_depth.SetTunings(pid_depth_p, pid_depth_i, pid_depth_d);
      Serial.println("<pid_depth_i:True>");
    } else {
      Serial.println("<pid_depth_i:False>");
    }
  }

  else if (part01.equals("pid_depth_d")) {
    pid_depth_d = double_part02;
    if (pid_depth_d >= 0) {
      pid_depth.SetTunings(pid_depth_p, pid_depth_i, pid_depth_d);
      Serial.println("<pid_depth_d:True>");
    } else {
      Serial.println("<pid_depth_d:False>");
    }
  }

  //PID roll
  else if (part01.equals("pid_roll_p")) {
    pid_roll_p = double_part02;
    if (pid_depth_d >= 0) {
      pid_trim.SetTunings(pid_roll_p, pid_roll_i, pid_roll_d);
      Serial.println("<pid_roll_p:True>");
    } else {
      Serial.println("<pid_roll_p:False>");
    }
  }

  else if (part01.equals("pid_roll_i")) {
    pid_roll_i = double_part02;
    if (pid_roll_i >= 0) {
      pid_trim.SetTunings(pid_roll_p, pid_roll_i, pid_roll_d);
      Serial.println("<pid_roll_i:True>");
    } else {
      Serial.println("<pid_roll_i:False>");
    }
  }

  else if (part01.equals("pid_roll_d")) {
    pid_roll_d = double_part02;
    if (pid_roll_d >= 0) {
      pid_trim.SetTunings(pid_roll_p, pid_roll_i, pid_roll_d);
      Serial.println("<pid_roll_d:True>");
    } else {
      Serial.println("<pid_roll_d:False>");
    }
  }
}


/**
  splits the string into two parts.
*/
String getValue(String data, char separator, int index)
{
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


//*** SETUP ****************************************************************
void setup() {
  // Serial.begin(57600); unnecessary with Teensy

  // Wait for start command
  bool reset_ardu = false;
  while (!reset_ardu) {
    Serial.println("<StepperArduino:0>");
    String msg = Serial.readString();
    String part01 = getValue(msg, ':', 0);

    part01.replace("<", "");
    if (part01 == "start") {
      reset_ardu = true;
    }
    delay(1);
  }

  // turn the PIDs on and set min/max output
  pid_depth.SetOutputLimits(-max_pid_output, max_pid_output);
  pid_trim.SetOutputLimits(-max_trim, max_trim);
  pid_depth.SetMode(MANUAL);
  pid_trim.SetMode(MANUAL);
  data_string.reserve(200);

  // fix pin configuration
  stepper_sb.begin();
  stepper_sb.flip_ref_direction();
  stepper_port.begin();

  while (!Serial) { //TODO: unnecessery? need start cmd to continue anyway
    // wait to connect
  }
  Serial.setTimeout(0);
}


//*** LOOP ******************************************************************
void loop() {

  switch (state) {

    // calibrate both steppers to center
    case CALIBRATE_STATE: {

        // continue calibration until finished
        if (!cal_status_sb) {
          cal_status_sb = stepper_sb.calibrate();
        }
        if (!cal_status_port) {
          cal_status_port = stepper_port.calibrate();
        }

        // continue when stepper is finished calibrated
        if (cal_status_sb && cal_status_port) {
          setTargetMode(MANUAL_STATE);
          Serial.println("<reset:True>");
        }

        // write something to serial, and the program will stop TODO remove
        while (Serial.available());

      }
      break;

    // set manual wing angle from serial
    case MANUAL_STATE: {
        target_angle_sb = manual_wing_pos;
        target_angle_port = manual_wing_pos;
        step_both();
      }
      break;

    // auto mode to find correct wing angle
    case AUTO_DEPTH_STATE: {

        pid_depth.Compute();
        Serial.print("wing angle: ");
        Serial.println(wing_angle);
        pid_trim.Compute();
        if (trim_angle != 0) {
          trimWingPos();
        } else {
          target_angle_sb = wing_angle;
          target_angle_port = wing_angle;
        }
        step_both();
      }
      break;
  }

  compensateWingToPitch();

  // for sending wing position to RPi
  unsigned long update_wing_pos = millis() - last_update_wing_pos;
  if (update_wing_pos > time_intervall) {
    //updateWingPosGUI(stepper_sb.get_angle(), stepper_port.get_angle());
    last_update_wing_pos = millis();
  }

  char c = ' ';
  if (Serial.available()) {
    c = Serial.read();
  }

  if (c == '>') {
    data_string.trim();
    translateString(data_string);
    data_string = "";
  }
  else if (c != ' ' && c != '\n' && c != '<') {
    data_string +=  c;
  }
}
