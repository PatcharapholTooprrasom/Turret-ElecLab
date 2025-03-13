#include <IRremote.h>
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Picopixel.h>

const int IR_PIN = PA0;
const int RELAY1_PIN = PA3;
const int RELAY2_PIN = PA4;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SERVO360_PIN PA8
#define SERVO180_PIN PB4

Servo servo360;
Servo servo180;

int servo180Pos = 109;
int servo360Current = 90; // Current position of 360 servo
int servo360Target = 90;  // Target position
int servo180range = 45;
const int SERVO180_STEP = 1;
const int SERVO360_STEP = 1; // Step size for 360 servo (smaller = slower)
const unsigned long CMD_TIMEOUT = 200;
const unsigned long REPEAT_DELAY = 300;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // Time between steps (ms)
const unsigned long RELAY_DEFAULT_TIME = 500;   // 0.5 วินาที
const unsigned long RELAY_EXTEND_TIME = 300;    // เพิ่มเวลา 500ms ทุกครั้งที่กดซ้ำ

bool relay1State = false;
bool relay2State = false;
unsigned long relay1StartTime = 0;
unsigned long relay1EndTime = 0;
bool relay1Extended = false;

IRrecv irrecv(IR_PIN);

unsigned long lastCmdTime = 0;
unsigned long lastServoUpdate = 0;
enum Command
{
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  OK,
  STAR,
  SQUARE
};
Command lastCommand = NONE;
bool newPress = false;

int displayangle(int angle)
{
  return 90 - (angle - 109);
}

void updateDisplay()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Servo Control");
  display.print("180: ");
  display.println(displayangle(servo180Pos));
  display.print("360: ");
  display.println(servo360Current); // Show current position instead of target
  display.print("Relay1: ");
  display.println(relay1State ? "ON" : "OFF");
  display.print("Relay2: ");
  display.println(relay2State ? "ON" : "OFF");
  display.display();
}

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn();

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  servo360.attach(SERVO360_PIN);
  servo180.attach(SERVO180_PIN);
  servo180.write(servo180Pos);
  servo360.write(servo360Current); // Initialize with current position

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED failed");
    while (1)
      ;
  }
  display.setFont(&Picopixel);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  updateDisplay();
}

void processIRCommand()
{
  unsigned long rawData = irrecv.decodedIRData.decodedRawData;
  bool isRepeat = irrecv.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;

  if (!isRepeat)
  {
    newPress = true;
    lastCommand = NONE;
    if (rawData == 2907897600)
      lastCommand = UP;
    else if (rawData == 3877175040)
      lastCommand = DOWN;
    else if (rawData == 4144561920)
    {
      servo360Target = 180;
      lastCommand = LEFT;
    }
    else if (rawData == 2774204160)
    {
      servo360Target = 0;
      lastCommand = RIGHT;
    }
    else if (rawData == 3810328320)
    {
      lastCommand = OK;
      if (!relay1State)
      {
        // เริ่มต้นเปิด relay
        relay1State = true;
        relay1StartTime = millis();
        relay1EndTime = relay1StartTime + RELAY_DEFAULT_TIME;
        digitalWrite(RELAY1_PIN, HIGH);
        updateDisplay();
      }
    }
    else if (rawData == 3910598400)
    {
      lastCommand = STAR;
    }
    else if (rawData == 4061003520)
    {
      lastCommand = SQUARE;
    }
  }

  if ((newPress || isRepeat) && lastCommand != NONE)
  {
    switch (lastCommand)
    {
    case UP:
      servo180Pos = constrain(servo180Pos + SERVO180_STEP, 109 - 25, 109 + 15);
      break;
    case DOWN:
      servo180Pos = constrain(servo180Pos - SERVO180_STEP, 109 - 25, 109 + 15);
      break;
    case OK:
      if (relay1State)
      {
        // ขยายเวลาเปิด relay
        relay1EndTime = min(relay1EndTime + RELAY_EXTEND_TIME, millis() + RELAY_EXTEND_TIME);
      }
      break;
    case SQUARE:
      relay2State = !relay2State;
      digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
      delay(250);
      break;
    case STAR:
      servo180Pos = 109;
      break;
    }
    newPress = false;
    updateDisplay();
  }

  servo180.write(servo180Pos);
  lastCmdTime = millis();
}

void loop()
{
  // จัดการ relay1 timeout
  if (relay1State && millis() > relay1EndTime)
  {
    relay1State = false;
    digitalWrite(RELAY1_PIN, LOW);
    updateDisplay();
  }

  // Handle 360 servo timeout
  if (millis() - lastCmdTime > CMD_TIMEOUT)
  {
    servo360Target = 90;
  }

  // Smooth 360 servo movement
  if (millis() - lastServoUpdate >= SERVO_UPDATE_INTERVAL)
  {
    if (servo360Current != servo360Target)
    {
      if (servo360Current < servo360Target)
      {
        servo360Current = min(servo360Current + SERVO360_STEP, servo360Target);
      }
      else
      {
        servo360Current = max(servo360Current - SERVO360_STEP, servo360Target);
      }
      servo360.write(servo360Current);
      updateDisplay();
    }
    lastServoUpdate = millis();
  }

  if (irrecv.decode())
  {
    processIRCommand();
    irrecv.resume();
  }

  // Debounce logic
  static unsigned long lastRepeatTime = 0;
  if (lastCommand == UP || lastCommand == DOWN)
  {
    if (millis() - lastRepeatTime > REPEAT_DELAY && !irrecv.isIdle())
    {
      processIRCommand();
      lastRepeatTime = millis();
    }
  }
}