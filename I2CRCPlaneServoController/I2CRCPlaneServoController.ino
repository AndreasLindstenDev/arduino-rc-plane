#include <Wire.h>
#include <Servo.h>

int8_t drivetrain_max = 2000;
int8_t drivetrain_min = 1000;

int left_aileron_servo_pin = 5;
int left_elevator_servo_pin = 10;
int right_aileron_servo_pin = 6;
int right_elevator_servo_pin = 11;
int drivetrain_servo_pin = 3;

// Used to receive commands from other arduino
volatile char rcv_char;
volatile int angle;
volatile bool new_command_available = false;
String command = "";

Servo left_aileron_servo;
Servo left_elevator_servo;
Servo right_aileron_servo;
Servo right_elevator_servo;
Servo drivetrain_servo;
Servo servos[4];

void setup() {
  //armDrivetrain(drivetrain_max, drivetrain_min); // Has a critical section. Drivetrain is now run from other Arduino
  Serial.begin(9600);
  Wire.begin(4);
  Wire.onReceive(readCommand);
  left_aileron_servo.attach(left_aileron_servo_pin);
  left_elevator_servo.attach(left_elevator_servo_pin);
  right_aileron_servo.attach(right_aileron_servo_pin);
  right_elevator_servo.attach(right_elevator_servo_pin);
  drivetrain_servo.attach(drivetrain_servo_pin);
  servos[0] = left_aileron_servo;
  servos[1] = left_elevator_servo;
  servos[2] = right_aileron_servo;
  servos[3] = right_elevator_servo;
  for (int i = 0; i <= 3; i++) {
    moveServoToAngle(50, servos[i]);
  }
  i2cScanner();
}

void i2cScanner()
{
  Serial.println("Starting scan");
  //while(!Serial); // Wait for serial monitor
  Serial.println("\nI2C Scanner");
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
  Serial.print("I2C device found at address 0x");
  if (address<16)
  Serial.print("0");
  Serial.print(address,HEX);
  Serial.println(" !");
  nDevices++;
  }
  else if (error==4)
  {
  Serial.print("Unknow error at address 0x");
  if (address<16)
  Serial.print("0");
  Serial.println(address,HEX);
  }
  }
  if (nDevices == 0)
  Serial.println("No I2C devices found\n");
  else
  Serial.println("done\n");
}

// Specify the pwm used to control the motors
void armDrivetrain(int max, int min)
{
  noInterrupts();
  drivetrain_servo.writeMicroseconds(max);
  delay(4000);
  drivetrain_servo.writeMicroseconds(min);
  delay(2000);
  interrupts();
}
 
void loop() {
  delay(10);
  noInterrupts();
  if(new_command_available)
  {
    executeCommand();
  }
  interrupts();
  //testServos();
}

void testServos() {
  for (int i = 0; i <= 3; i++) {
    for (int angle = 10; angle <= 90; angle += 20) {
      moveServoToAngle(angle, servos[i]);
      delay(200);
    }
  }
}

void readCommand(int howMany)
{
  Serial.print("Received on the i2c bus, ");
  command = "";
  while (1 < Wire.available()) {
    //Serial.println("flag1");
    rcv_char = Wire.read();
    command.concat(rcv_char);
  }
  // recieve byte as an integer
  angle = Wire.read();
  // left aileron servo
  new_command_available = true;
}

void executeCommand()
{
  // Critical section
  new_command_available = false;
  Serial.print("executing command: ");
  Serial.println(command);
  if (command == "la") {
    Serial.print("la command: ");
    Serial.println(angle);
    // The servos are mirrored so same angle works for both left and right
    moveServoToAngle(100-angle, left_aileron_servo);
    moveServoToAngle(100-angle, right_aileron_servo);
  }
  // left elevator servo
  if (command == "le") {
    Serial.print("le command: ");
    Serial.println(angle);
    moveServoToAngle(100 - angle, left_elevator_servo);
    moveServoToAngle(angle, right_elevator_servo);
  }
  // drivetrain (not used here at the moment, signal output already sent to servo from other arduino)
  if (command == "en") {
    Serial.print("en command");
    Serial.print(angle);
    if(angle > 60)
    {
      int val = map(angle, 60, 100, drivetrain_min, drivetrain_max);
      drivetrain_servo.writeMicroseconds(val);
    }
    else
    {
      drivetrain_servo.writeMicroseconds(drivetrain_min);
    }
  }
}

// Angle should be between 0 and 100
void moveServoToAngle(int angle_percent, Servo servo) {
  if(angle_percent > 100)
  {
    angle_percent = 100;
  }
  if(angle_percent < 0)
  {
    angle_percent = 0;
  }
  int val = map(angle_percent, 0, 100, 800, 2200);
  servo.writeMicroseconds(val);
}