#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Constants for servo range
#define SERVO_MIN 0
#define SERVO_MAX 100

// PID gains (adjust these for better control)
#define PITCH_GAIN 3.5
#define ROLL_GAIN 3.5

int8_t drivetrain_max = 2000;
int8_t drivetrain_min = 1000;

int left_aileron_servo_pin = 5;
int left_elevator_servo_pin = 10;
int right_aileron_servo_pin = 6;
int right_elevator_servo_pin = 11;
int drivetrain_servo_pin = 3;

Servo left_aileron_servo;
Servo left_elevator_servo;
Servo right_aileron_servo;
Servo right_elevator_servo;
Servo drivetrain_servo;
Servo servos[4];

void setup() {
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    //while (1);
  }

  //armDrivetrain(drivetrain_max, drivetrain_min); // Has a critical section. Drivetrain is now run from other Arduino
  Serial.begin(9600);
  Wire.begin(4);
  Wire.onReceive(executeCommand);
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

void executeCommand(int howMany) {
  Serial.println("received on the i2c bus");
  char rcv_char;
  int angle;
  String command = "";
  while (1 < Wire.available()) {
    rcv_char = Wire.read();
    command.concat(rcv_char);
  }
  // recieve byte as an integer
  angle = Wire.read();
  // left aileron servo
  
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

  // drivetrain (not used here at the moment, signal output sent to servo from other arduino)
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

  // regualtor (this function crashes the whole system)
  if (/*command == "re"*/false)
  {
    // Read magnetometer
    sensors_event_t event;
    mag.getEvent(&event);
    float pitch = event.magnetic.x; // X-axis represents pitch
    float roll = event.magnetic.y;  // Y-axis represents roll
    float z = event.magnetic.z;     // Z-axis for upside-down detection
    // Calculate course change
    // Check if the airplane is upside down
    bool isUpsideDown = (z > 0);
    // Desired pitch and roll (neutral position)
    float desiredPitch = 10.0; // Each magnetometer needs to be calibrated
    float desiredRoll = -10.0; // TODO: Remove hardcoded values.
    // Calculate pitch and roll errors
    float pitchError = desiredPitch - pitch;
    float rollError = desiredRoll - roll;
    // Adjust control values using proportional control
    float elevatorOutput = PITCH_GAIN * pitchError;
    float aileronOutput = ROLL_GAIN * rollError;
    // Clamp outputs to servo range
    elevatorOutput = constrain(elevatorOutput, SERVO_MIN, SERVO_MAX) + 50; // +50, as 50 represent servo in neutral position
    aileronOutput = constrain(aileronOutput, SERVO_MIN, SERVO_MAX) + 50; // +50, ...
    // Invert controls if upside down
    if (isUpsideDown)
    {
      elevatorOutput = SERVO_MAX - elevatorOutput;
      aileronOutput = SERVO_MAX - aileronOutput;
    }
    // Adjust servos
    moveServoToAngle(100-aileronOutput, left_aileron_servo);
    moveServoToAngle(100-aileronOutput, right_aileron_servo);
    moveServoToAngle(100 - elevatorOutput, left_elevator_servo);
    moveServoToAngle(elevatorOutput, right_elevator_servo);
    // Debugging
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" Roll: "); Serial.print(roll);
    Serial.print(" Z: "); Serial.print(z);
    Serial.print(" Elevator: "); Serial.print(elevatorOutput);
    Serial.print(" Aileron: "); Serial.println(aileronOutput);

  }
}

// Angle should be between 0 and 100
void moveServoToAngle(int angle_percent, Servo servo) {
  if(angle_percent > 90)
  {
    angle_percent = 90;
  }
  if(angle_percent < 10)
  {
    angle_percent = 10;
  }
  int val = map(angle_percent, 0, 100, 800, 2200);
  servo.writeMicroseconds(val);
}