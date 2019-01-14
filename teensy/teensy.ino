#include <ArduinoHardware.h>
#include <control/drive_values.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int PWM_CENTER = 9830; //  15% duty cycle
int PWM_LOWER = 6554;   //  10% duty cycle
int PWM_HIGHER = 13108;  //  20% duty cycle
int SERVO_PIN = 6;
int MOTOR_PIN = 5;
int KILL_PIN = 2;
unsigned long duration = 0;
boolean emergency_stop = false;

void messageDrive(const control::drive_values &pwm);
void messageEmergencyStop(const std_msgs::Bool &flag);

// ROS
ros::NodeHandle nh;
ros::Subscriber<control::drive_values> control_serial_drive_parameters("control_serial_drive_parameters", &messageDrive);
ros::Subscriber<std_msgs::Bool> control_emergency_stop("control_emergency_stop", &messageEmergencyStop);

void messageDrive(const control::drive_values &pwm) {
  if (emergency_stop) {
    return;
  }

  int safe_pwm_angle = min(PWM_HIGHER, max(PWM_LOWER, pwm.pwm_angle));
  int safe_pwm_drive = min(PWM_HIGHER, max(PWM_LOWER, pwm.pwm_drive));

  analogWrite(SERVO_PIN, safe_pwm_drive);
  analogWrite(MOTOR_PIN, safe_pwm_angle);
}

void messageEmergencyStop(const std_msgs::Bool &flag) {
  emergency_stop = flag.data;
  if (emergency_stop) {
    analogWrite(SERVO_PIN, PWM_CENTER);
    analogWrite(MOTOR_PIN, PWM_CENTER);
  }
}

void setup() {
  analogWriteFrequency(SERVO_PIN, 100);
  analogWriteFrequency(MOTOR_PIN, 100);
  analogWriteResolution(16);
  analogWrite(SERVO_PIN, PWM_CENTER);
  analogWrite(MOTOR_PIN, PWM_CENTER);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, INPUT);

  nh.initNode();
  nh.subscribe(control_serial_drive_parameters);
  nh.subscribe(control_emergency_stop);
}

void loop() {
  nh.spinOnce();
  duration = pulseIn(KILL_PIN, HIGH, 30000);
  while (duration > 1900) {
    duration = pulseIn(KILL_PIN, HIGH, 30000);
    analogWrite(SERVO_PIN, PWM_CENTER);
    analogWrite(MOTOR_PIN, PWM_CENTER);
  }
}
