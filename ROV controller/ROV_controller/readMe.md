# WingStepper.h  
WingStepper is a simple library for controlling a stepper.  
  
Author: towedROV NTNU 2022. 
  
Note: see the StepperWing_Demo.ino for demonstration and stepper testing.

## Initialize class  

Use ```WingStepper <stepper name> (STEP_PIN, DIR_PIN, SENSOR_PIN)``` to construct class.  

in ```void setup()``` use ```<stepper name>.begin()``` to activate class.  


## Calibration    

```<stepper name>.calibrate()``` will callibrate stepper position.  
@ returns true when calibration is complete.  

Stepper will move in both direction and store the position of the magnets.  
It will then calculate the center between the two points.  
Note: that the fucntion does not complete the calibration, but only iterates towards the goal. This is to enable Teensy to process other tasks while the steppers calibrates.  
Function can be run in a while loop and a check to see if function is finished.  
Also note that the calibration does not rotate the stepper to zero after calibration.

## Movement  
```<stepper name>.rotate(float desired_angle)```  
@returns true when at right position.  
This function will rotate to the set position.  
Note: as with calibrate, this function only iterates towards the goal and has to be run many times to finish.  

## Other functions  
```set_step_delay_us(int us)```  
Change this to change the speed of the stepper.  (lower is faster)
Fastest is close to 600 depending on load.   

```<stepper name>.flip_ref_direction()```  
Use this command if the stepper rotates the wrong direction.  

```<stepper name>.set_offset_angle(float offset_angle)```  
Use this to manually calibrate wing if normal calibration is off center.  

```<stepper name>.get_angle()```  
Returns stepper position in degree from center.  
Returnvalue is float.  

```<stepper name>.get_pos()```  
Return stepper position in amount of steps from center.  
Returnvalue is int.  

```<stepper name>.set_angle_to_step_const (int new_constant)```  
```<stepper name>.set_step_to_angle_const (int new_constant)```  
Use this fucntions to change the standard gear and stepper configuation.  
The vaule is calculated by using stepper degree/step from datasheet and gear ratio.