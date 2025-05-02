#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_I2C_Multiplexer.h>
#include "DFRobot_Heartrate.h"
#include "AK09918.h"
#include "ICM20600.h"
#include <math.h>

// I2C multiplexer setup
#define MULTIPLEXER_ADDR 0x70
DFRobot_I2C_Multiplexer I2CMultiplexer(&Wire, MULTIPLEXER_ADDR);

// OLED setup on port 7 of multiplexer
const uint8_t OLED_PORT = 7;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// Heart rate sensor on analog pin A2
#define heartratePin A2
DFRobot_Heartrate heartrate(ANALOG_MODE);

// UI buttons
#define BUTTON_SELECT 3
#define BUTTON_MOVE   2

// UI screen states
enum ScreenID {
  SCREEN_MENU = 0,
  SCREEN_TRAINING_START,
  SCREEN_TRAINING_BASIC,
  SCREEN_TRAINING_HOLD,
  SCREEN_TESTING_CLASSIFY,
  SCREEN_TESTING_SQUAT,
  SCREEN_TESTING_RESULT,
  SCREEN_ABOUT,
  SCREEN_COUNT
};

// UI state variables
int currentScreen = SCREEN_MENU;
int menuSelection = 0;
int heartRate = 75;
String selectedTraining = "Basic";

// Python classification result
int squatLabel = 0;
float squatConfidence = 0.0;
int sampleSize = 0;
bool resultReady = false;

// For detecting button state changes
bool prevSelectState = HIGH;
bool prevMoveState   = HIGH;

// IMU sensor setup (we had to put 3rd position in 5th connector because of potential failure in MUX)
const uint8_t imuPorts[] = {0, 1, 5, 3, 4};
const uint8_t numIMUs = 5;

// One magnetometer and one accel/gyro per IMU port
AK09918 ak09918s[numIMUs];
ICM20600 icm20600s[numIMUs] = { ICM20600(true), ICM20600(true), ICM20600(true), ICM20600(true), ICM20600(true) };

int32_t x, y, z;
int16_t acc_x, acc_y, acc_z;
int32_t offset_x[numIMUs], offset_y[numIMUs], offset_z[numIMUs];
double roll, pitch;
double declination_london = -0.5; //we changed it from double declination_shenzhen = -2.2;

// Function declarations
void drawScreen(int screen);
void drawMenuScreen();
void drawTrainingStartScreen();
void drawTrainingBasicScreen();
void drawTrainingHoldScreen();
void drawTestingClassifyScreen();
void drawTestingSquatScreen();
void drawAboutScreen();
void captureIMUData();
void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  I2CMultiplexer.begin();

  // OLED Initialization
  I2CMultiplexer.selectPort(OLED_PORT);
  delay(100);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED allocation failed on Port 7");
    while (1);
  }
  display.clearDisplay();
  display.display();
  Serial.println("OLED initialized on Port 7");

  // Buttons with internal pull-ups
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(BUTTON_MOVE, INPUT_PULLUP);

  // Init each IMU on its assigned port
  for (uint8_t i = 0; i < numIMUs; i++) {
    I2CMultiplexer.selectPort(imuPorts[i]);

    if (ak09918s[i].initialize() != AK09918_ERR_OK) {
      Serial.print("Failed to initialize AK09918 on port ");
      Serial.println(imuPorts[i]);
    }

    icm20600s[i].initialize();
    ak09918s[i].switchMode(AK09918_POWER_DOWN);
    ak09918s[i].switchMode(AK09918_CONTINUOUS_100HZ);

    // Run calibration (collects magnetometer offset)
    Serial.print("Calibrating IMU on port ");
    Serial.println(imuPorts[i]);
    calibrate(10000, &offset_x[i], &offset_y[i], &offset_z[i]);
    Serial.println("Calibration complete.");
  }
}

void loop() {
  // Parse serial result from Python
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("RESULT,")) {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        squatLabel = line.substring(firstComma + 1, secondComma).toInt();
        squatConfidence = line.substring(secondComma + 1, thirdComma).toFloat();
        sampleSize = line.substring(thirdComma + 1).toInt();
        resultReady = true;
      }
    }
  }

  // Read buttons (active LOW)
  bool selectState = digitalRead(BUTTON_SELECT);
  bool moveState   = digitalRead(BUTTON_MOVE);

  // Navigate through menus with MOVE button
  if (prevMoveState == HIGH && moveState == LOW) {
    switch (currentScreen) {
      case SCREEN_MENU: menuSelection = (menuSelection + 1) % 3; break;
      case SCREEN_TRAINING_START: menuSelection = (menuSelection + 1) % 2; break;
      case SCREEN_TRAINING_BASIC: menuSelection = (menuSelection + 1) % 4; break;
      case SCREEN_TRAINING_HOLD: menuSelection = (menuSelection + 1) % 2; break;
      case SCREEN_TESTING_CLASSIFY: menuSelection = (menuSelection + 1) % 2; break;
      case SCREEN_TESTING_SQUAT: menuSelection = 0; break;
    }
  }

  // SELECT button confirms selection
  if (prevSelectState == HIGH && selectState == LOW) {
    if (currentScreen == SCREEN_MENU) {
      if (menuSelection == 0) currentScreen = SCREEN_TRAINING_START;
      else if (menuSelection == 1) currentScreen = SCREEN_TESTING_CLASSIFY;
      else currentScreen = SCREEN_ABOUT;
    } else if (currentScreen == SCREEN_TRAINING_START) {
      currentScreen = (menuSelection == 0) ? SCREEN_TRAINING_BASIC : SCREEN_MENU;
    } else if (currentScreen == SCREEN_TRAINING_BASIC) {
      if (menuSelection == 3) currentScreen = SCREEN_TRAINING_START;
      else {
        selectedTraining = (menuSelection == 0) ? "Basic" :
                           (menuSelection == 1) ? "Narrow" : "Plie";
        currentScreen = SCREEN_TRAINING_HOLD;
      }
    } else if (currentScreen == SCREEN_TRAINING_HOLD && menuSelection == 1) {
      currentScreen = SCREEN_TRAINING_BASIC;
    } else if (currentScreen == SCREEN_TESTING_CLASSIFY) {
      currentScreen = (menuSelection == 0) ? SCREEN_TESTING_SQUAT : SCREEN_MENU;
      resultReady = false;
    } else if (currentScreen == SCREEN_TESTING_SQUAT && menuSelection == 0) {
      currentScreen = SCREEN_TESTING_CLASSIFY;
    } else if (currentScreen == SCREEN_ABOUT) {
      currentScreen = SCREEN_MENU;
    }
    menuSelection = 0;
  }

  prevSelectState = selectState;
  prevMoveState   = moveState;

  // Update OLED
  I2CMultiplexer.selectPort(OLED_PORT);
  delay(10);
  display.clearDisplay();
  drawScreen(currentScreen);
  display.display();

  // Capture sensor data during testing
  if (currentScreen == SCREEN_TESTING_SQUAT) {
    captureIMUData();
  }

  delay(100);
}


// OLED Draw
void drawScreen(int screen) {
  I2CMultiplexer.selectPort(OLED_PORT);
  delay(10);
  display.clearDisplay();

  // Display heart rate reading in top-right corner
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(90, 2);
  display.print("\x03"); // Heart symbol
  display.print(heartRate);
  display.println("bpm");

  switch (screen) {
    case SCREEN_MENU:
      drawMenuScreen();
      break;
    case SCREEN_TRAINING_START:
      drawTrainingStartScreen();
      break;
    case SCREEN_TRAINING_BASIC:
      drawTrainingBasicScreen();
      break;
    case SCREEN_TRAINING_HOLD:
      drawTrainingHoldScreen();
      break;
    case SCREEN_TESTING_CLASSIFY:
      drawTestingClassifyScreen();
      break;
    case SCREEN_TESTING_SQUAT:
      drawTestingSquatScreen();
      break;
    case SCREEN_ABOUT:
      drawAboutScreen();
      break;
    default:
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Unknown Screen");
      break;
  }
}

void drawMenuScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Menu");

  int arrowX = 0, textX = 12;
  int yStart = 10, yGap = 8;
  // Option 0: Collect
  display.setCursor(arrowX, yStart);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, yStart);
  display.println("Collect");
  // Option 1: Testing
  display.setCursor(arrowX, yStart + yGap);
  if (menuSelection == 1) display.print(">");
  display.setCursor(textX, yStart + yGap);
  display.println("Testing");
  // Option 2: About
  display.setCursor(arrowX, yStart + 2 * yGap);
  if (menuSelection == 2) display.print(">");
  display.setCursor(textX, yStart + 2 * yGap);
  display.println("About");
}

void drawTrainingStartScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Collect");

  int arrowX = 0, textX = 12, yStart = 10, yGap = 8;
  display.setCursor(arrowX, yStart);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, yStart);
  display.println("Start");
  
  display.setCursor(arrowX, yStart + yGap);
  if (menuSelection == 1) display.print(">");
  display.setCursor(textX, yStart + yGap);
  display.println("Back");
}

void drawTrainingBasicScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Collect");

  int arrowX = 0, textX = 12;
  int yStart = 8, yGap = 6;
  display.setCursor(arrowX, yStart);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, yStart);
  display.println("Basic");
  
  display.setCursor(arrowX, yStart + yGap);
  if (menuSelection == 1) display.print(">");
  display.setCursor(textX, yStart + yGap);
  display.println("Narrow");
  
  display.setCursor(arrowX, yStart + 2 * yGap);
  if (menuSelection == 2) display.print(">");
  display.setCursor(textX, yStart + 2 * yGap);
  display.println("Plie");
  
  display.setCursor(arrowX, yStart + 3 * yGap);
  if (menuSelection == 3) display.print(">");
  display.setCursor(textX, yStart + 3 * yGap);
  display.println("Back");
}

void drawTrainingHoldScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Collect");

  int arrowX = 0, textX = 12, yStart = 10, yGap = 8;
  display.setCursor(arrowX, yStart);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, yStart);
  display.println(selectedTraining);
  
  display.setCursor(arrowX, yStart + yGap);
  if (menuSelection == 1) display.print(">");
  display.setCursor(textX, yStart + yGap);
  display.println("Back");
}

void drawTestingClassifyScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Testing");

  int arrowX = 0, textX = 12, yStart = 10, yGap = 8;
  display.setCursor(arrowX, yStart);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, yStart);
  display.println("Classify Squat");
  
  display.setCursor(arrowX, yStart + yGap);
  if (menuSelection == 1) display.print(">");
  display.setCursor(textX, yStart + yGap);
  display.println("Back");
}

void drawTestingSquatScreen() {
  int textX = 12;
  int row0 = 0, row1 = 8, row2 = 16, row3 = 24;
  
  // First Row: Display squat type (mapped)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(textX, row0);
  if (resultReady) {
    display.print("Squat: ");
    if (squatLabel == 1) {
      display.println("Simple");
    } else if (squatLabel == 2) {
      display.println("Narrow");
    } else if (squatLabel == 3) {
      display.println("Plie");
    } else {
      display.println("Unknown");
    }
  } else {
    display.println("Waiting...");
  }
  
  // Second Row: Display confidence as percentage
  display.setCursor(textX, row1);
  if (resultReady) {
    float confPercent = squatConfidence * 100.0;
    display.print("Conf.: ");
    display.print(confPercent, 0);
    display.println("%");
  }
  
  // Third Row: Display sample size
  display.setCursor(textX, row2);
  if (resultReady) {
    display.print("Sample size: ");
    display.println(sampleSize);
  }
  
  // Fourth Row: "Back" option
  display.setCursor(0, row3);
  if (menuSelection == 0) display.print(">");
  display.setCursor(textX, row3);
  display.println("Back");
}

void drawAboutScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("About");
  
  display.setCursor(0, 8);
  display.println("Avalos, S./Cui, E.");
  display.setCursor(0, 16);
  display.println("Yang, H. Imperial");
  display.setCursor(0, 24);
  display.println("MSc AML 24-25");
}

// Sensor Data Capture Function
void captureIMUData() {
  // Read buttons (BUTTON_MOVE==3, BUTTON_SELECT==2)
  int buttonState1 = digitalRead(BUTTON_MOVE);
  int buttonState2 = digitalRead(BUTTON_SELECT);

  // For each IMU, select its port, read sensor values, and output CSV-formatted data (based from library example for each IMU)
  for (uint8_t i = 0; i < numIMUs; i++) {
    I2CMultiplexer.selectPort(imuPorts[i]);
  
    // Get acceleration data
    acc_x = icm20600s[i].getAccelerationX();
    acc_y = icm20600s[i].getAccelerationY();
    acc_z = icm20600s[i].getAccelerationZ();

    // Get gyroscope data
    int16_t gyro_x = icm20600s[i].getGyroscopeX();
    int16_t gyro_y = icm20600s[i].getGyroscopeY();
    int16_t gyro_z = icm20600s[i].getGyroscopeZ();

    // Get magnetometer data and apply calibration offsets
    ak09918s[i].getData(&x, &y, &z);
    x -= offset_x[i];
    y -= offset_y[i];
    z -= offset_z[i];

    // Compute roll and pitch (in radians)
    roll = atan2((float)acc_y, (float)acc_z);
    pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

    // Compute heading (in degrees)
    double Xheading = x * cos(pitch) + y * sin(roll) * sin(pitch) + z * cos(roll) * sin(pitch);
    double Yheading = y * cos(roll) - z * sin(pitch);
    double heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination_shenzhen;

    // Print sensor data for CSV format
    // Format: DATA,IMU#  AccX  AccY  AccZ  GyroX  GyroY  GyroZ  MagX  MagY  MagZ  Heading  Button1  Button2
    Serial.print("DATA,IMU");
    Serial.print(i + 1);
    Serial.print("\t");
    Serial.print(acc_x); Serial.print("\t");
    Serial.print(acc_y); Serial.print("\t");
    Serial.print(acc_z); Serial.print("\t");
    Serial.print(gyro_x); Serial.print("\t");
    Serial.print(gyro_y); Serial.print("\t");
    Serial.print(gyro_z); Serial.print("\t");
    Serial.print(x); Serial.print("\t");
    Serial.print(y); Serial.print("\t");
    Serial.print(z); Serial.print("\t");
    Serial.print(heading); Serial.print("\t");
    Serial.print(buttonState1); Serial.print("\t");
    Serial.println(buttonState2);

    delay(20);  // Short delay for each IMU read
  }
}

// Calibration Function from original example code in Library
// Uses IMU on the selected port to calculate calibration offsets
void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
  int32_t value_x_min = 0, value_x_max = 0;
  int32_t value_y_min = 0, value_y_max = 0;
  int32_t value_z_min = 0, value_z_max = 0;
  uint32_t timeStart = 0;

  // Uses IMU on the selected port for calibration:
  ak09918s[0].getData(&x, &y, &z);
  value_x_min = x; value_x_max = x;
  value_y_min = y; value_y_max = y;
  value_z_min = z; value_z_max = z;
  delay(20);

  timeStart = millis();
  while ((millis() - timeStart) < timeout) {
    ak09918s[0].getData(&x, &y, &z);
    if (x < value_x_min) value_x_min = x;
    if (x > value_x_max) value_x_max = x;
    if (y < value_y_min) value_y_min = y;
    if (y > value_y_max) value_y_max = y;
    if (z < value_z_min) value_z_min = z;
    if (z > value_z_max) value_z_max = z;
    Serial.print(".");
    delay(20);
  }
  *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
  *offsety = value_y_min + (value_y_max - value_y_min) / 2;
  *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}
