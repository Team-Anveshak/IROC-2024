// IROC 2024 Drive PCB Code

#include <ros.h>
#include <drive/IROC.h>

#define NUM_MOTORS 4

#define LEFT_FRONT_DIR 13
#define LEFT_FRONT_PWM 15
#define LEFT_REAR_DIR 12
#define LEFT_REAR_PWM 2
#define RIGHT_FRONT_DIR 27
#define RIGHT_FRONT_PWM 4
#define RIGHT_REAR_DIR 26
#define RIGHT_REAR_PWM 5

struct MotorDriver{
  int dir_pin;
  int pwm_pin;
};

MotorDriver motordriver[NUM_MOTORS] = {
  
  {LEFT_FRONT_DIR, LEFT_FRONT_PWM},    // LEFT FRONT MOTOR
  {LEFT_REAR_DIR, LEFT_REAR_PWM},      // LEFT REAR MOTOR
  {RIGHT_FRONT_DIR, RIGHT_FRONT_PWM},  // RIGHT FRONT MOTOR
  {RIGHT_REAR_DIR, RIGHT_REAR_PWM}     // RIGHT REAR MOTOR
};

float drive_commands[NUM_MOTORS / 2];

ros::NodeHandle nh;

void driveCallback(const drive::IROC& joy_msg){

  for (int i=0; i < NUM_MOTORS / 2; i++){
    drive_commands[i] = joy_msg.drivecommands[i];
  } 
}

ros::Subscriber<drive::IROC> drive_sub("/iroc/joy", &driveCallback);

void setup() {
  
  nh.initNode();
  Serial.begin(115200);
  for (int i = 0; i < NUM_MOTORS; i++){
    pinMode(motordriver[i].dir_pin, OUTPUT);
    pinMode(motordriver[i].pwm_pin, OUTPUT);
  }
  nh.subscribe(drive_sub);

}

void loop() {

  for (int i = 0; i < NUM_MOTORS; i++){
    if (drive_commands[0] > 0){
      analogWrite(motordriver[i].pwm_pin, drive_commands[0]);
      digitalWrite(motordriver[i].dir_pin,(i < NUM_MOTORS / 2) ? LOW : HIGH);
    }
    else if (drive_commands[0] < 0){
      analogWrite(motordriver[i].pwm_pin, -drive_commands[0]);
      digitalWrite(motordriver[i].dir_pin,(i < NUM_MOTORS / 2) ? HIGH : LOW);
    }
    else if (drive_commands[1] > 0){
      analogWrite(motordriver[i].pwm_pin, drive_commands[1]);
      digitalWrite(motordriver[i].dir_pin, HIGH);
    }
    else if (drive_commands[1] < 0){
      analogWrite(motordriver[i].pwm_pin, -drive_commands[1]);
      digitalWrite(motordriver[i].dir_pin, LOW);
    }
  }
  nh.spinOnce();
  delay(1);

}
