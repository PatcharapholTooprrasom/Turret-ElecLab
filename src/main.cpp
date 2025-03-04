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

int servo180Pos = 92;
int servo360Target = 90;
int servo180range = 45;
const int SERVO180_STEP = 1;
const unsigned long CMD_TIMEOUT = 200;
const unsigned long REPEAT_DELAY = 300;

bool relay1State = false;
bool relay2State = false;

IRrecv irrecv(IR_PIN);

unsigned long lastCmdTime = 0;
enum Command {
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

int displayangle(int angle) {
  if (angle < 92)
    return 90 - (92 - angle) * 44 / 45;
  else
    return 90 + (angle - 92) * 43 / 45;
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Servo Control");
  display.print("180: ");
  display.println(displayangle(servo180Pos));
  display.print("360: ");
  display.println(servo360Target);
  display.print("Relay1: ");
  display.println(relay1State ? "ON" : "OFF");
  display.print("Relay2: ");
  display.println(relay2State ? "ON" : "OFF");
  display.display();
}

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  servo360.attach(SERVO360_PIN);
  servo180.attach(SERVO180_PIN);
  servo180.write(servo180Pos);
  servo360.write(servo360Target);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while (1);
  }
  display.setFont(&Picopixel);
  display.setTextSize(2);  // Reduced size to fit all information
  display.setTextColor(SSD1306_WHITE);
  updateDisplay();
}

void processIRCommand() {
  unsigned long rawData = irrecv.decodedIRData.decodedRawData;
  bool isRepeat = irrecv.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;

  if (!isRepeat) {
    newPress = true;
    lastCommand = NONE;
    if (rawData == 3877175040)
      lastCommand = UP;
    else if (rawData == 2907897600)
      lastCommand = DOWN;
    else if (rawData == 4144561920) {
      servo360Target = 180;
      lastCommand = LEFT;
    } else if (rawData == 2774204160) {
      servo360Target = 0;
      lastCommand = RIGHT;
    } else if (rawData == 3810328320) {
      lastCommand = OK;
    } else if (rawData == 3910598400) {  // STAR pressed
      relay1State = !relay1State;
      digitalWrite(RELAY1_PIN, relay1State ? HIGH : LOW);
      lastCommand = STAR;
    } else if (rawData == 4061003520) {  // SQUARE pressed
      relay2State = !relay2State;
      digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
      lastCommand = SQUARE;
    }
  }

  // Handle movement
  if ((newPress || isRepeat) && lastCommand != NONE) {
    switch (lastCommand) {
      case UP:
        servo180Pos = constrain(servo180Pos + SERVO180_STEP, 90 - servo180range, 90 + servo180range + 5);
        break;
      case DOWN:
        servo180Pos = constrain(servo180Pos - SERVO180_STEP, 90 - servo180range, 90 + servo180range - 5);
        break;
      case OK:
        servo180Pos = 92;
        break;
    }
    newPress = false;
  }

  servo180.write(servo180Pos);
  servo360.write(servo360Target);

  static int last180DisplayPos = -1;
  if (servo180Pos != last180DisplayPos || relay1State != digitalRead(RELAY1_PIN) || relay2State != digitalRead(RELAY2_PIN)) {
    last180DisplayPos = servo180Pos;
    updateDisplay();
  }

  lastCmdTime = millis();
}

void loop() {
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    servo360Target = 90;
    servo360.write(servo360Target);
  }

  if (irrecv.decode()) {
    processIRCommand();
    irrecv.resume();
  }

  // Debounce logic for single press
  static unsigned long lastRepeatTime = 0;
  if (lastCommand == UP || lastCommand == DOWN) {
    if (millis() - lastRepeatTime > REPEAT_DELAY && !irrecv.isIdle()) {
      processIRCommand();
      lastRepeatTime = millis();
    }
  }
}