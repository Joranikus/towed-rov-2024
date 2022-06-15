/**
  This programs control the wing angles, by calculating a desired angle with PID controllers, and sets the motors position.
   Author: Slepe-ROV 2021
*/

/**
  Sjekk maks og min stepper pos etter reset
  Sjekk reset pos

*/
bool test_from_terminal = false;
#include <PID_v1.h>
#include <SoftwareSerial.h>
char charIn = ' ';
char lastCharIn;
//Defining pins connected to the arduino
const int direction_pin_sb = 2;
const int step_pin_sb = 3;
const int sensor_sb = 4;

const int direction_pin_port = 5;
const int step_pin_port = 6;
const int sensor_port = 7;

//Stepper positions
int current_pos_sb, current_pos_port;
int step_delay = 1;
byte stepper_pulse_port = HIGH;
byte stepper_pulse_sb = HIGH;

//Wing positions
double wing_angle_sb;
double wing_angle_port;

//Feedback GUI
String data_string;

//Switch case modes
const int MANUAL_MODE = 1;
const int AUTO_DEPTH_MODE = 0;
int target_mode = MANUAL_MODE;

//Flag
boolean has_been_reset = false;
boolean testing = false;
bool boolWingPos;

// LIMITS
int angle_to_step_const = 55;         //step = angle * k, k = (1.8/99) = 55,  1.8 is degree/step, 99 is gear ratio
float step_to_angle_const = 0.01818;  //angle = step * k, k = (99/1.8) = 0.01818     //note: not in use
int max_wing_angle = 25;
int max_stepper_pos = max_wing_angle * angle_to_step_const;
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
unsigned long last_step_port = 0;
unsigned long last_step_sb = 0;
unsigned long last_update_wing_pos = 0;

//Reset at start
bool reset_ardu = false;

// PID controllers
PID pid_depth = PID(&depth, &wing_angle, &set_point_depth, pid_depth_p, pid_depth_i, pid_depth_d, REVERSE);
PID pid_trim = PID(&roll, &trim_angle, &set_point_roll, pid_roll_p, pid_roll_i, pid_roll_i, DIRECT);

void setup() {
  //Serial.begin(57600); unnecessary with teensy 4.0
  
  while (!reset_ardu) {
    Serial.println("<StepperArduino:0>");
    String msg = Serial.readString();
    String part01 = getValue(msg, ':', 0);

    part01.replace("<", "");
    if (part01 == "start") {
      reset_ardu = true;
    }
  }
  //turn the PIDs on and set min/max output
  pid_depth.SetOutputLimits(-max_pid_output, max_pid_output);
  pid_trim.SetOutputLimits(-max_trim, max_trim);
  pinMode(direction_pin_port, OUTPUT);
  pinMode(step_pin_port, OUTPUT);
  pinMode(sensor_port, INPUT);
  pinMode(direction_pin_sb, OUTPUT);
  pinMode(step_pin_sb, OUTPUT);
  pinMode(sensor_sb, INPUT);
  pid_depth.SetMode(MANUAL);
  pid_trim.SetMode(MANUAL);
  data_string.reserve(200);
  while (!Serial) {
    //wait to connect
  }
  Serial.setTimeout(0);
}


void loop() {

  if (has_been_reset) {
    unsigned long update_wing_pos = millis() - last_update_wing_pos;
    switch (target_mode) {

      case MANUAL_MODE:
        wing_angle_sb = constrain(manual_wing_pos, -max_wing_angle, max_wing_angle);
        wing_angle_port = constrain(manual_wing_pos, -max_wing_angle, max_wing_angle);
        break;

      case AUTO_DEPTH_MODE:

        pid_depth.Compute();
        Serial.print("wing angle: ");
        Serial.println(wing_angle);
        pid_trim.Compute();
        if (trim_angle != 0) {
          trimWingPos();
        } else {
          wing_angle_sb = wing_angle;
          wing_angle_port = wing_angle;
        }
        break;
    }
    compensateWingToPitch();

    int step_position_sb = wing_angle_sb * angle_to_step_const;
    int step_position_port = wing_angle_port * angle_to_step_const;

    if (step_position_sb != current_pos_sb) {
      moveStepperSb(step_position_sb);
    }
    if (step_position_port != current_pos_port) {
      moveStepperPort(step_position_port);
    }


    if (update_wing_pos > time_intervall)
    {
      updateWingPosGUI(current_pos_sb, current_pos_port);
      last_update_wing_pos = millis();
    }
  }

  char c = ' ';

  if (Serial.available()) {
    c = Serial.read();
  }

  if (c == '>') {
    data_string.trim();
    translateString(data_string);
    data_string = "";

  } else if (c != ' ' && c != '\n' && c != '<') {
    data_string +=  c;
  }

}


/**
   Moves the stepper motor on starboard side on step towards the desired position

   @param The desired stepper position.
*/
void moveStepperSb(int desired_pos) {
  long dt = millis() - last_step_sb;
  if (dt > step_delay) {
    if (desired_pos > current_pos_sb && desired_pos <= max_stepper_pos) {
      digitalWrite(direction_pin_sb, LOW);
      digitalWrite(step_pin_sb, stepper_pulse_sb);
      last_step_sb = millis();
      if (stepper_pulse_sb == LOW) {
        current_pos_sb ++;
      }
    }
    else if (desired_pos < current_pos_sb && !digitalRead(sensor_sb)) {
      digitalWrite(direction_pin_sb, HIGH);
      digitalWrite(step_pin_sb, stepper_pulse_sb);
      last_step_sb = millis();
      if (stepper_pulse_sb == LOW) {
        current_pos_sb --;
      }
    }
    stepper_pulse_sb = !stepper_pulse_sb;
  }
}


/**
   Moves the stepper motor on port side on step towards the desired position, alternating between high and low pulses.

   @param The desired stepper position.
*/
void moveStepperPort(int desired_pos) {
  long dt = millis() - last_step_port;
  if (dt > step_delay) {
    if (desired_pos > current_pos_port && desired_pos <= max_stepper_pos) {
      digitalWrite(direction_pin_port, LOW);
      digitalWrite(step_pin_port, stepper_pulse_port);
      if (stepper_pulse_port == LOW) {
        current_pos_port ++;
      }
    }
    else if (desired_pos < current_pos_port && !digitalRead(sensor_port)) {
      digitalWrite(direction_pin_port, HIGH);
      digitalWrite(step_pin_port, stepper_pulse_port);
      if (stepper_pulse_port == LOW) {
        current_pos_port --;
      }
    }
    last_step_port = millis();
    stepper_pulse_port = !stepper_pulse_port;
  }
}


/**
  Set a new target mode
   @param The desired target mode.
*/
bool setTargetMode(int newTargetMode, int wing_pos = 0) {
  bool mode_set = false;
  if (newTargetMode == MANUAL_MODE) {
    pid_depth.SetMode(MANUAL);
    pid_trim.SetMode(MANUAL);
    target_mode = MANUAL_MODE;
    manual_wing_pos = wing_pos;
    mode_set = true;
  }

  else if (newTargetMode == AUTO_DEPTH_MODE) {
    target_mode = AUTO_DEPTH_MODE;
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
  Maps the values, returning a double
  only manual and depth mode is included at this stage.
   @param value is the value that is being mapped, in and outputs sets the range.
*/
double mapf(double value, double minIn, double maxIn, double minOut, double maxOut) {
  double x = (value - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
  return constrain(x, minOut, maxOut);
}


void compensateWingToPitch() {
  wing_angle_sb -= pitch_compensation;
  wing_angle_port -= pitch_compensation;
}


/**
  Trims the wing angles. compensate by increasing the trim on the opposite side if the max value is reached.
*/
void trimWingPos() {
  if (wing_angle + trim_angle > max_wing_angle) {

    wing_angle_sb = max_wing_angle;
    wing_angle_port = max_wing_angle - 2 * trim_angle;
  }
  else if (wing_angle - trim_angle < -max_wing_angle) {

    wing_angle_sb = -max_wing_angle + 2 * trim_angle;
    wing_angle_port = -max_wing_angle;
  } else {
    wing_angle_sb = wing_angle - trim_angle;
    wing_angle_port = wing_angle + trim_angle;
  }
  wing_angle_sb = constrain(wing_angle_sb, -max_wing_angle, max_wing_angle);
  wing_angle_port = constrain(wing_angle_port, -max_wing_angle, max_wing_angle);
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
  if (boolWingPos) {

    double current_angle_port = mapf(pos_port, min_stepper_pos, max_stepper_pos, -max_wing_angle, max_wing_angle);
    String dataToSend = "<wing_pos_port:";
    dataToSend.concat(current_angle_port + pitch);
    dataToSend.concat(">");
    Serial.println(dataToSend);
  } else {

    double current_angle_sb = mapf(pos_sb, min_stepper_pos, max_stepper_pos, -max_wing_angle, max_wing_angle);
    String dataToSend = "<wing_pos_sb:";
    dataToSend.concat(current_angle_sb + pitch);
    dataToSend.concat(">");
    Serial.println(dataToSend);
  }
  boolWingPos = !boolWingPos;
}


/**
  Split the incoming strings, part01 is the header/command while part02 is the value.
   @s is a string from the raspberry pi. ROV main computer.
*/
void translateString(String s) {

  String part01 = getValue(s, ':', 0);
  String part02 = getValue(s, ':', 1);
  double double_part02 = part02.toFloat();

  if (part01.equals("reset")) {
    if (test_from_terminal) {
      has_been_reset = true;
    } else {
      resetStepper();
    }
    Serial.println("<reset:True>");
  }

  else if (part01.equals("auto_mode")) {
    if (part02.equals("True")) {
      setTargetMode(AUTO_DEPTH_MODE);
      Serial.println("<auto_mode:True>");
    } else if (part02.equals("False")) {
      setTargetMode(MANUAL_MODE);
      Serial.println("<auto_mode:True>");
      manual_wing_pos = (int) wing_angle_sb;
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
    setTargetMode(MANUAL_MODE, max_wing_angle);
    target_mode = MANUAL_MODE;
    Serial.println("<emergency_surface:True>");
  }

  else if (part01.equals("depth")) {
  println("depth of stystem",part02, part02.toInt)
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


/**
   Resets the steppers by stepping them back to the endposition, then setting the positions as
   negative before the desired position is set to zero. the negative values must be calibrated when changing gears.
   Notice that there might be a difference for each side, so calibrating should be done one motor at the time.
*/
void resetStepper() {

  while (!digitalRead(sensor_port) && !digitalRead(sensor_sb)) {
    digitalWrite(direction_pin_port, HIGH);
    digitalWrite(direction_pin_sb, HIGH);
    digitalWrite(step_pin_port, HIGH);
    digitalWrite(step_pin_sb, HIGH);
    delay(step_delay);
    digitalWrite(step_pin_port, LOW);
    digitalWrite(step_pin_sb, LOW);
    delay(step_delay);

  }
  while (!digitalRead(sensor_sb)) {

    digitalWrite(direction_pin_sb, HIGH);
    delay(step_delay);
    digitalWrite(step_pin_sb, HIGH);
    delay(step_delay);
    digitalWrite(step_pin_sb, LOW);

  }
  while (!digitalRead(sensor_port)) {
    digitalWrite(direction_pin_port, HIGH);
    delay(step_delay);
    digitalWrite(step_pin_port, HIGH);
    delay(step_delay);
    digitalWrite(step_pin_port, LOW);
  }
  //these must be calibrated after adjustments in the transmission system.
  current_pos_sb = min_stepper_pos;
  current_pos_port = min_stepper_pos;
  int wantedPos = 0;

  while (current_pos_sb != wantedPos || current_pos_sb != wantedPos) {
    moveStepperPort(wantedPos);
    moveStepperSb(wantedPos);
  }

  setTargetMode(MANUAL_MODE);
  has_been_reset = true;
}
