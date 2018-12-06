#include <ArduinoHardware.h>
#include <f1tenths_controller/drive_values.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

boolean flagStop = false;
int pwm_center_value = 9830; //  15% duty cycle
int pwm_lowerlimit = 6554;   //  10% duty cycle
int pwm_upperlimit = 13108;  //  20% duty cycle

int servoPin = 6;
int motorPin = 5;

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

int kill_pin = 2;
unsigned long duration = 0;

void messageDrive(const f1tenths_controller::drive_values &pwm) {
  if (!flagStop) {
    str_msg.data = pwm.pwm_drive;
    chatter.publish(&str_msg);

    if (pwm.pwm_drive < pwm_lowerlimit) {
      analogWrite(servoPin, pwm_lowerlimit); //  Safety lower limit
    } else if (pwm.pwm_drive > pwm_upperlimit) {
      analogWrite(servoPin, pwm_upperlimit); //  Safety upper limit
    } else {
      analogWrite(servoPin, pwm.pwm_drive); //  Incoming data
    }

    if (pwm.pwm_angle < pwm_lowerlimit) {
      analogWrite(motorPin, pwm_lowerlimit); //  Safety lower limit
    } else if (pwm.pwm_angle > pwm_upperlimit) {
      analogWrite(motorPin, pwm_upperlimit); //  Safety upper limit
    } else {
      analogWrite(motorPin, pwm.pwm_angle); //  Incoming data
    }
  } else {
    analogWrite(servoPin, pwm_center_value);
    analogWrite(motorPin, pwm_center_value);
  }
}

void messageEmergencyStop(const std_msgs::Bool &flag) {
  flagStop = flag.data;
  if (flagStop) {
    analogWrite(servoPin, pwm_center_value);
    analogWrite(motorPin, pwm_center_value);
  }
}

ros::Subscriber<f1tenths_controller::drive_values> sub_drive("control_serial_drive_parameters", &messageDrive);
ros::Subscriber<std_msgs::Bool> sub_stop("control_emergency_stop", &messageEmergencyStop);

void setup() {
  analogWriteFrequency(servoPin, 100);
  analogWriteFrequency(motorPin, 100);
  analogWriteResolution(16);
  analogWrite(servoPin, pwm_center_value);
  analogWrite(motorPin, pwm_center_value);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, INPUT);
  //  digitalWrite(2,LOW);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_stop);

  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  duration = pulseIn(kill_pin, HIGH, 30000);
  while (duration > 1900) {
    duration = pulseIn(kill_pin, HIGH, 30000);
    analogWrite(servoPin, pwm_center_value);
    analogWrite(motorPin, pwm_center_value);
  }
}
