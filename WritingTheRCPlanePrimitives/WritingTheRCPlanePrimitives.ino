#include <Splash.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<printf.h>

#define len(arr) sizeof (arr)/sizeof (arr[0])
#define OLED_RESET 4
// Constants for servo range
#define SERVO_MIN 0
#define SERVO_MAX 100
#define PITCH_GAIN 2.5
#define ROLL_GAIN 2.5
  
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

Servo right_aileron_servo;
Servo right_elevator_servo;
Servo drivetrain_servo;
Servo test_limits_servo;

RF24 radio(10, 9, 4000000);

const byte address[6] = "00001";
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false; // true = TX role, false = RX role

uint8_t right_joystick_vrx_percent = 50;
uint8_t right_joystick_vry_percent = 50;
uint8_t left_joystick_vrx_percent = 50;
uint8_t placeholder_value = 0;
uint8_t right_aileron_servo_pin = 5;
uint8_t right_elevator_servo_pin = 6;
uint8_t right_joystick_analog_pin_vrx = A2; // elevators or aileron
uint8_t right_joystick_analog_pin_vry = A3; // elevators or aileron
uint8_t left_joystick_analog_pin_vrx = A0; // drivetrain
uint8_t test_limits_servo_pin = 3;
uint8_t drivetrain_pin = 5;
uint8_t drivetrain_max = 2000;
uint8_t drivetrain_min = 1000;

float desiredPitch = 2.5; // Each magnetometer needs to be calibrated upon vehicle start
float desiredRoll = 0.0; //

bool isSender = false;
bool isReciever = false;
bool connection_link_ok = false;

unsigned long last_time = 0;

typedef void (*MenuFunction) ();

struct command_and_value
{
  char command_chr_arr[3];
  byte value;
};

struct MenuPage;

struct MenuItem
{
  int8_t menu_str_index;
  MenuPage *next_menu_page;
  MenuFunction fMenuItemAction;
};

struct MenuPage
{
  MenuPage *previous_menu_page;
  MenuItem *menu_items;
  int8_t num_menu_items;
  int8_t selected_item;
};

void testMenuFunction()
{
  Serial.println(F("Test function"));
}

void sendHello()
{
  //sendMessage("Hello");
}

void sendLa30()
{
  transmittCommandByWiFi("la", 30);
}

void sendLa60()
{
  transmittCommandByWiFi("la", 60);
}

void recieveCommand()
{
  command_and_value cav;
  receiveCommandByWiFi(&cav);
}
void control()
{
  flyPlaneTransmitter();
}

void fly()
{
  flyPlaneReciever();
}

MenuFunction fTestMenuFunction = testMenuFunction;
MenuFunction fsendHello; // = sendHello;
MenuFunction fRecieveMessage; // = recieveMessage;
MenuFunction fFly = fly;
MenuFunction fControl = control;
MenuFunction fsendLa30 = sendLa30;
MenuFunction fsendLa60 = sendLa60;
MenuFunction fDetermineServoLimits = determineServoLimits;

MenuPage fly_menu_page;
MenuPage test_menu_1_page;
MenuPage test_menu_2_page;
MenuPage test_sender_menu_page;
MenuPage test_reciever_menu_page;
MenuPage current_menu_page;

const char menu_str_0[] PROGMEM = "      Fly menu   -->>";
const char menu_str_1[] PROGMEM = "Be plane";
const char menu_str_2[] PROGMEM = "Be controller";
const char menu_str_3[] PROGMEM = "<<-- Test menu 1 -->>";
const char menu_str_4[] PROGMEM = "Test drivetrain";
const char menu_str_5[] PROGMEM = "Test sender";
const char menu_str_6[] PROGMEM = "Test Reciever";
const char menu_str_7[] PROGMEM = "Send la 30";
const char menu_str_8[] PROGMEM = "Send la 60";
const char menu_str_9[] PROGMEM = "Recieve message";
const char menu_str_10[] PROGMEM = "<<-- Test menu 2     ";
const char menu_str_11[] PROGMEM = "Servo limits";

const char *const string_table[] PROGMEM = {
  menu_str_0,
  menu_str_1,
  menu_str_2,
  menu_str_3,
  menu_str_4,
  menu_str_5,
  menu_str_6,
  menu_str_7,
  menu_str_8,
  menu_str_9,
  menu_str_10,
  menu_str_11
};

MenuItem fly_menu_items[3] = {
  {0, &test_menu_1_page, NULL},
  {1, NULL, fFly},
  {2, NULL, fControl}
};

MenuItem test_menu_1_items[4] = {
  {3, &test_menu_2_page, NULL},
  {4, NULL, NULL},
  {5, &test_sender_menu_page, fTestMenuFunction},
  {6, &test_reciever_menu_page, NULL}
};

MenuItem test_sender_menu_items[4] = {
  {7, NULL, fsendLa30},
  {8, NULL, fsendLa60},
};

MenuItem test_reciever_menu_items[1] = {
  {9, NULL, recieveCommand}
};

MenuItem test_menu_2_items[2] = {
  {10, NULL, NULL},
  {11, NULL, fDetermineServoLimits}
};

// In all this code, any analog value is treated as a percent between functions
// Any analog return value is converted to a percent before returned from function
// Any analog parameter is given as a percent, and mapped to suitable range in function
void setup() 
{ 
  drivetrain_servo.attach(5, 1000, 2000);
  armDrivetrain();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setRotation(2);
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    //while (1);
  }
  initializeMenuSystem();
  armTranceiver();
  i2cScanner();
  //right_aileron_servo.attach(right_aileron_servo_pin);
  //right_elevator_servo.attach(right_elevator_servo_pin);
  //test_limits_servo.attach(test_limits_servo_pin);
}

void loop() 
{
  //char buffer[21];
  //getSerialMessage(buffer, 21);
  //drawMessageOnScreen(buffer, 2);
  //drawTestEngineMenuPage();
  //testDisplayJoystickPercent(right_joystick_analog_pin_vrx, right_joystick_analog_pin_vry);
  //testDisplayJoystickServo(right_joystick_analog_pin_vrx, right_joystick_analog_pin_vry, right_aileron_servo, right_elevator_servo);
  //testDrivetrain(A0, drivetrain_servo);
  runMenu(right_joystick_analog_pin_vrx, right_joystick_analog_pin_vry);
  //testI2CSendServoCommand();
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

void flyPlaneTransmitter()
{
  drawMessageOnScreen("Sender\nmode", 2);
  while(true)
  {
    bool right_joystick_vrx_neutral = false;
    bool right_joystick_vry_neutral = false;
    bool left_joystick_vrx_neutral = true; // Don't use this in the logic for now
    // Middle position of servo is at 50 percent
    right_joystick_vrx_percent = readJoyStickPercent(right_joystick_analog_pin_vrx);
    right_joystick_vry_percent = readJoyStickPercent(right_joystick_analog_pin_vry);
    left_joystick_vrx_percent = readJoyStickPercent(left_joystick_analog_pin_vrx);
    if(right_joystick_vrx_percent > 35 && right_joystick_vrx_percent < 65)
    {
      right_joystick_vrx_neutral = true;
    }
    if(right_joystick_vry_percent > 35 && right_joystick_vry_percent < 65)
    {
      right_joystick_vry_neutral = true;
    }
    //if(left_joystick_vrx_percent > 35 && left_joystick_vrx_percent < 65)
    //{
    //  left_joystick_vrx_neutral = true;
    //}
    if(right_joystick_vrx_neutral && right_joystick_vry_neutral && left_joystick_vrx_neutral)
    {
      delay(50);
      transmittCommandByWiFi("en", (byte)left_joystick_vrx_percent);
      delay(50);
      transmittCommandByWiFi("re", (byte)placeholder_value); /* Transmitt command to use onboard regulator
      to level the plane (crude autopilot) */

    }
    else
    {
      // Delay some time to give capacior time to recharge?
      delay(50);
      transmittCommandByWiFi("la", (byte)right_joystick_vry_percent);
      delay(50);
      transmittCommandByWiFi("le", (byte)right_joystick_vrx_percent);
      delay(50);
      transmittCommandByWiFi("en", (byte)left_joystick_vrx_percent);
    }
  }
}

void flyPlaneReciever()
{
  byte old_val;
  command_and_value cav;
  char en_command[3] = "en"; // Used to sanity check incomming data
  char re_command[3] = "re";
  {
    // Read magnetometer and use its initial values when plane is on the 
    // ground as the desired pitch and roll of the plane
    sensors_event_t event;
    mag.getEvent(&event);
    desiredPitch = event.magnetic.x; // X-axis represents pitch
    desiredRoll = event.magnetic.y;  // Y-axis represents roll
  }
  drawMessageOnScreen("Plane\nmode", 2);
  while(true)
  {
    //sendI2CServoCommand(re_command, cav.value);
    //delay(100);
    //continue;
    if(connection_link_ok)
    {
      unsigned long compare_val = last_time + 3000UL;
      unsigned long current_val = millis();
      if(compare_val < current_val)
      {
        connection_link_ok = false;
        drivetrain_servo.writeMicroseconds(1000); // Motors off
        drawMessageOnScreen("Lost connection\nMotor off", 1);
      }
    }
    if(receiveCommandByWiFi(&cav))
    {
      if (strcmp(cav.command_chr_arr, en_command) == 0) // Sanity check of recieved message
      { 
        if(connection_link_ok == false) //Save cycles by only drawing on status change
        {
          drawMessageOnScreen("Plane\nmode", 2);
        }
        connection_link_ok = true; /* As all command types are sent continuously, only
        update upon receiving engine command to save time*/
        last_time = millis();
        throttleDrivetrain(&cav); // Use this arduino to control plane
      }
      else if (strcmp(cav.command_chr_arr, re_command) == 0) // Sanity check of recieved message
      {
        flyByRegulator(); 
      }
      else
      {
        sendI2CServoCommand(cav.command_chr_arr, cav.value); // Let second arduino control ailerons and elevators
      }
    }
  }
}

void flyByRegulator()
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
    // Calculate pitch and roll errors
    float pitchError = desiredPitch - pitch;
    float rollError = desiredRoll - roll;
    // Adjust control values using proportional control
    float elevatorOutput = (PITCH_GAIN * pitchError) + 50; // +50, as 50 represent servo in neutral position
    float aileronOutput = (ROLL_GAIN * rollError) + 50; // +50, ...
    // Clamp outputs to servo range
    elevatorOutput = constrain(elevatorOutput, SERVO_MIN, SERVO_MAX); 
    aileronOutput = constrain(aileronOutput, SERVO_MIN, SERVO_MAX); 
    // Invert controls if upside down
    if (isUpsideDown)
    {
      Serial.println("Upside down");
      elevatorOutput = SERVO_MAX - elevatorOutput;
      aileronOutput = SERVO_MAX - aileronOutput;
    }
    // Adjust servos
    command_and_value cav;
    cav.command_chr_arr[0] = 'l';
    cav.command_chr_arr[1] = 'a';
    cav.command_chr_arr[2] = '\0';
    cav.value = aileronOutput;
    sendI2CServoCommand(cav.command_chr_arr, cav.value);
    delay(20);
    cav.command_chr_arr[0] = 'l';
    cav.command_chr_arr[1] = 'e';
    cav.command_chr_arr[2] = '\0';
    cav.value = elevatorOutput;
    sendI2CServoCommand(cav.command_chr_arr, cav.value);

    delay(10);
    // Debugging
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" Roll: "); Serial.print(roll);
    Serial.print(" Z: "); Serial.print(z);
    Serial.print( "Upside down: "); z > 1 ? Serial.print("yes") : Serial.print("no");
    Serial.print(" Elevator: "); Serial.print(elevatorOutput);
    Serial.print(" Aileron: "); Serial.println(aileronOutput);
}

// command can be la, le, ra, and re
void sendI2CServoCommand(char *command, byte angle)
{
    Wire.beginTransmission(4);
    Wire.write(command);
    Wire.write(angle);
    Wire.endTransmission();
}

void testI2CSendServoCommand()
{
  for(byte angle = 10; angle <= 90; angle += 10)
  {
      sendI2CServoCommand("re", angle);
      delay(200);
  }
}

void initializeMenuSystem()
{
  fly_menu_page = {NULL, fly_menu_items, 3, 0};
  test_menu_1_page = {&fly_menu_page, test_menu_1_items, 4, 0};
  test_menu_2_page = {&test_menu_1_page, test_menu_2_items, 2, 0};
  test_sender_menu_page = {&test_menu_1_page, test_sender_menu_items, 2, 0};
  test_reciever_menu_page = {&test_menu_1_page, test_reciever_menu_items, 1, 0};
  current_menu_page = fly_menu_page;
}

void armTranceiver()
{
  while(!radio.begin())
  {
    drawMessageOnScreen("Could not\nconnect to\nradio", 1);
  }
  radio.setPALevel(RF24_PA_LOW);
  printf_begin();
  radio.printDetails();
}


void transmittCommandByWiFi(const char *command, const byte value)
{
  delay(30);
  if(!isSender)
  {
    radio.openWritingPipe(address);
    radio.closeReadingPipe(0);
    radio.stopListening();
    isSender = true;
    isReciever = false;
  }
  //Serial.println("Waiting to send");
  char command_chr_arr[3];
  strncpy(command_chr_arr, command, 2);
  command_chr_arr[2] = value;
  radio.write(&command_chr_arr, 3);
  Serial.print("Sent: " + String(command_chr_arr[0]) + String(command_chr_arr[1]) + " ");
  Serial.println(value);
  delay(50);
}

bool receiveCommandByWiFi(command_and_value *cav)
{
  //Serial.println("receiveCommandByWiFi - flag 1");
  delay(10);
  if(!isReciever)
  {
    //Serial.println("receiveCommandByWiFi - flag 2");
    radio.openReadingPipe(0, address);
    radio.startListening();
    isSender = false;
    isReciever = true;
  }
  if(!radio.available())
  {
    Serial.println("Could not recieve.");
    return false; // Signal no message was recieved
  }
  while (radio.available()) {
    //Serial.println("receiveCommandByWiFi - flag 4");
    char rec_chr_arr[3];
    radio.read(&rec_chr_arr, 3);
    cav->command_chr_arr[0] = rec_chr_arr[0];
    cav->command_chr_arr[1] = rec_chr_arr[1];
    //cav->command_chr_arr[0] = 'e';
    //cav->command_chr_arr[1] = 'n';
    cav->command_chr_arr[2] = '\0';
    cav->value = rec_chr_arr[2];
    //cav->value = 60;
    Serial.print("Received command: ");
    Serial.print(cav->command_chr_arr);
    Serial.print(" and value: ");
    Serial.println(cav->value);
    //Serial.println("Flag1");
    //Serial.println("receiveCommandByWiFi - flag 5");
    return true;
  }
}

void determineServoLimits()
{
  int milliseconds = 1000;
  int joy_stick_percent = 50;
  char msg_1[7] = "Val:  ";
  char msg_2[9] = "Joy:    ";
  char tmp[5] = "    ";
  char message[25];
  char num[5];
  strcpy(&message[0], msg_1);
  while(true)
  {
    joy_stick_percent = readJoyStickPercent(right_joystick_analog_pin_vrx);
    if(joy_stick_percent > 80)
    {
      milliseconds += 50;
    }
    if(joy_stick_percent < 10)
    {
      milliseconds -= 50;
    }
    if(snprintf(&message[6], 5, "%d", milliseconds) == 3)
    {
      strcpy(&message[9], " ");
    }
    strcpy(&message[10], "\n");
    strcpy(&message[11], msg_2);
    snprintf(&message[19], 3, "%d", joy_stick_percent);
    test_limits_servo.writeMicroseconds(milliseconds);
    drawMessageOnScreen(message, 2);
    if(readJoyStickPercent(right_joystick_analog_pin_vry) > 80)
    {
      break;
    }
  }
}

void getSerialMessage(char *message, int max_size) {
  int counter = 0;
  Serial.println("Enter data:");
  while(!Serial.available()) {} // Halt until data available
  delay(20); // Wait for data
  while(Serial.available() && counter < max_size - 1)
  {
    message[counter] = Serial.read();
    counter++;
  }
  message[counter] = '\0';
  Serial.print(message);
}

void drawMessageOnScreen(char *message, int text_size)
{
  display.clearDisplay();
  display.setTextSize(text_size);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(message);
  display.display();
}

void drawMessageOnScreen(char *message, int text_size, int row)
{
  display.clearDisplay();
  display.setTextSize(text_size);
  display.setTextColor(WHITE);
  display.setCursor(8*row, 0);
  display.println(message);
  display.display();
}


void runMenu(int analog_pin_vrx, int analog_pin_vry)
{
  int8_t * selected_item = &current_menu_page.selected_item;
  right_joystick_vrx_percent = readJoyStickPercent(analog_pin_vrx);
  right_joystick_vry_percent = readJoyStickPercent(analog_pin_vry);
  Serial.println("vrx: " + String(right_joystick_vrx_percent) + " vry: " + String(right_joystick_vry_percent));
  if(right_joystick_vrx_percent < 10 && *selected_item < current_menu_page.num_menu_items - 1)
  {
    *selected_item += 1;
  }
  else if(right_joystick_vry_percent > 80)
  {
    if(current_menu_page.menu_items[*selected_item].fMenuItemAction)
    {
      current_menu_page.menu_items[*selected_item].fMenuItemAction();
    }
    if(current_menu_page.menu_items[*selected_item].next_menu_page)
    {
      current_menu_page = (*current_menu_page.menu_items[*selected_item].next_menu_page);
    }
  }
  else if(right_joystick_vrx_percent > 80 && *selected_item > 0)
  {
    *selected_item -= 1;
  }
  else if(right_joystick_vry_percent < 10)
  {    
    if(current_menu_page.previous_menu_page)
    {
      current_menu_page = (*current_menu_page.previous_menu_page);
    }
  }
  drawMenu(1);
}

void drawMenu(int text_size)
{
  int8_t num_menu_items = current_menu_page.num_menu_items;
  int8_t selected_item = current_menu_page.selected_item;
  MenuItem *menu_items = current_menu_page.menu_items;
  char buffer[22]; // Width of display for text size 1 is 21 charachters
  display.clearDisplay();
  display.setTextSize(text_size);
  display.setTextColor(WHITE);
  for(int8_t i = 0; i < num_menu_items; i++)
  {
    strcpy_P(buffer, (char *)pgm_read_word(&(string_table[menu_items[i].menu_str_index])));
    display.setCursor(0, i * 8);
    if(i == selected_item)
    {
      drawHighlighted(buffer);
    }
    else
    {
      display.print(buffer);
    }
  }
  display.display();
}

void drawHighlighted(const char *message)
{
  int cursor_x = display.getCursorX() + 1;
  int cursor_y = display.getCursorY();
  display.setCursor(cursor_x, cursor_y);
  int width = 6 * strlen(message);
  int height = 8;
  display.fillRect(cursor_x - 1, cursor_y - 1, width + 1, height + 1, WHITE);
  display.setTextColor(BLACK);
  display.print(message);
  display.setTextColor(WHITE);
}

void drawTestEngineMenuPage()
{
  display.clearDisplay();
  int x0 = 2;
  int y0 = 1;
  int width = 128;
  int height = 32;
  int letter_length = 6; //pixels
  display.fillRoundRect(x0, y0, width - 2 * x0, height - 2 * y0, 4, WHITE);
  display.fillRoundRect(2 * x0, 2 * y0, width - 4 * x0, height - 4 * y0, 4, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(4 * x0, 4 * y0);
  display.println("Engine 1");
  display.setCursor(4 * x0 + 9 * letter_length, 4 * y0);
  display.println("Engine 2");
  display.display();
}

void testDisplayJoystickPercent(int analog_pin_vrx, int analog_pin_vry)
{
  right_joystick_vrx_percent = readJoyStickPercent(analog_pin_vrx);
  right_joystick_vry_percent = readJoyStickPercent(analog_pin_vry);
  String string1 = "VRX: ";
  String string2 = string1 + right_joystick_vrx_percent;
  String string3 = "\nVRY: ";
  String string4 = string3 + right_joystick_vry_percent;
  String message = string2 + string4;
  message = message + " ";
  Serial.println(message);
  char buffer[42];
  message.toCharArray(buffer, message.length());
  drawMessageOnScreen(buffer, 2);
  delay(200);
}

void testDisplayJoystickServo(int analog_pin_vrx, int analog_pin_vry, Servo right_aileron_servo, Servo right_elevator_servo)
{
  right_joystick_vrx_percent = readJoyStickPercent(analog_pin_vrx);
  right_joystick_vry_percent = readJoyStickPercent(analog_pin_vry);
  String string1 = "VRX: ";
  String string2 = string1 + right_joystick_vrx_percent;
  String string3 = "\nVRY: ";
  String string4 = string3 + right_joystick_vry_percent;
  String message = string2 + string4;
  moveServoToAngle(right_joystick_vrx_percent, right_aileron_servo);
  moveServoToAngle(right_joystick_vry_percent, right_elevator_servo);
  //drawMessageOnScreen(message, 2);
}

// Angle should be between 0 and 100
void moveServoToAngle(int angle_percent, Servo servo)
{
  int val = map(angle_percent, 0, 100, 800, 2200);
  servo.writeMicroseconds(val);
}

// A return of 50 means that Joystick is in default position
int readJoyStickPercent(int analog_pin)
{
  float sensor_val = analogRead(analog_pin);
  // This order of operation is requered or there will be computational errors idk why
  float return_val = sensor_val * 100;
  return_val = return_val / 1024;
  return return_val;
}

void armDrivetrain()
{
  
  drivetrain_servo.writeMicroseconds(2000);    // Send the signal to the ESC
  delay(2000);
  drivetrain_servo.writeMicroseconds(1000);
  delay(2000);
  //drivetrain_servo.writeMicroseconds(1050);
  //drawMessageOnScreen("Waiting", 2);
  //delay(4000);
  /*
  for(int i = 0; i < 100; i++)
  {
    servo.writeMicroseconds(max);
    drawMessageOnScreen("Arm drive\nmax", 2);
  }
    for(int i = 0; i < 100; i++)
  {
    servo.writeMicroseconds(min);
    drawMessageOnScreen("Arm drive\nmin", 2);
  }
  */
  //delay(8000);
  //servo.writeMicroseconds(min);
  //drawMessageOnScreen("Arm drive\nmin", 2);
  //delay(2000);
}

void throttleDrivetrain(command_and_value *cav)
{
  //Serial.print("en command");
  //Serial.print(cav->value);
  if(cav->value > 55)
  {
    int val = map(cav->value, 55, 100, 1000, 2000);
    drivetrain_servo.writeMicroseconds(val);
  }
  else
  {
    drivetrain_servo.writeMicroseconds(1000);
  }
}

void testDrivetrain(int analog_pin_vrx, Servo drivetrain_servo)
{
  left_joystick_vrx_percent = readJoyStickPercent(analog_pin_vrx);
  String string1 = "VRX: ";
  String string2 = string1 + left_joystick_vrx_percent;
  String message = string2;
  //throttleDrivetrain(drivetrain_max, drivetrain_min, left_joystick_vrx_percent, drivetrain_servo);
  //drawMessageOnScreen(message, 2);
}