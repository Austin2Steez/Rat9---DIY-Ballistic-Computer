//SUBFINAL
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <LSM303.h>
#include <Preferences.h>

// OLED display dimensions for 0.91" SSD1306 (128x32)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1  // Reset pin (not used, tie to reset line or use -1 if none)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C addresses (change if needed)
#define BME280_I2C_ADDR_PRIMARY   0x76
#define BME280_I2C_ADDR_SECONDARY 0x77

Adafruit_BME280 bme;
LSM303 compass;
Preferences prefs;

// Pin definitions for buttons (GPIO on ESP32)
#define PIN_BTN_UP      13
#define PIN_BTN_DOWN    12
#define PIN_BTN_LEFT    14
#define PIN_BTN_RIGHT   27
#define PIN_BTN_CENTER  26  // joystick center (select)
#define PIN_BTN_BACK    25  // back/cancel
#define PIN_BTN_MODE    33  // mode toggle (manual/continuous or hold for calibration)
#define PIN_BTN_TRIGGER 32  // external trigger button for rangefinder
#define DEG_TO_RAD (PI / 180.0)
#define DEBOUNCE_MS 50

struct ButtonState {
  bool current = false;
  bool last = false;
  bool justPressed = false;
  bool justReleased = false;
  bool longPressed = false;
  unsigned long pressStart = 0;
};


// UART for rangefinder
HardwareSerial RangeSerial(2);
#define PIN_RANGE_TX 17  // TX to rangefinder (if needed)
#define PIN_RANGE_RX 16  // RX from rangefinder

// Bullet data structure
struct Bullet {
  const char* code;      // short code/key
  const char* name;      // full name (if needed)
  double bc;
  double muzzle_velocity; // m/s
  double weight_gr;       // in grains
  double sight_height;    // in meters (sight height above bore)
  double diameter;        // in meters (bullet diameter)
  char drag_model;        // 'G1' or 'G7'
};

// Bullet profiles (all bullet types from original script)
static const Bullet bulletList[] = {
  { "308_M80",    ".308 M80 Ball",          0.400, 840.0, 147.0, 0.06985, 0.00782, '1' /* will interpret 'G' + number below */ },
  { "308_SUB",    ".308 Subsonic",          0.518, 325.0, 200.0, 0.06985, 0.00782, '1' },
  { "300BLK_SUP", ".300 Blackout Supersonic",0.305, 670.0, 125.0, 0.06985, 0.00782, '1' },
  { "300BLK_SUB", ".300 Blackout Subsonic", 0.660, 305.0, 220.0, 0.06985, 0.00782, '7' },
  { "556_M193",   "5.56 NATO 55gr M193",    0.243, 990.0,  55.0, 0.06985, 0.00570, '1' },
  { "68BLK_SUP",  "6.8 Blackout Supersonic",0.370, 780.0, 110.0, 0.06985, 0.00680, '7' },
  { "68BLK_SUB",  "6.8 Blackout Subsonic",  0.450, 330.0, 180.0, 0.06985, 0.00680, '1' }
};
static const int NUM_BULLETS = sizeof(bulletList) / sizeof(Bullet);

// Drag tables (ft/s, Cd) for G1 and G7 models
struct DragEntry { int vel_fps; double Cd; };
static const DragEntry G1_TABLE[] = {
  {137, 0.262}, {228, 0.230}, {274, 0.211}, {366, 0.203}, {457, 0.198},
  {549, 0.195}, {640, 0.192}, {732, 0.188}, {823, 0.185}, {914, 0.182},
  {1006,0.179}, {1097,0.176}, {1189,0.172}, {1280,0.169}, {1372,0.165},
  {1463,0.162}, {1555,0.159}, {1646,0.156}, {1738,0.152}, {1829,0.149},
  {1920,0.147}, {2012,0.144}, {2103,0.141}
};
static const DragEntry G7_TABLE[] = {
  {137, 0.120}, {228, 0.112}, {274, 0.107}, {366, 0.104}, {457, 0.102},
  {549, 0.100}, {640, 0.098}, {732, 0.095}, {823, 0.093}, {914, 0.091},
  {1006,0.089}, {1097,0.087}, {1189,0.085}, {1280,0.083}, {1372,0.081},
  {1463,0.079}, {1555,0.077}, {1646,0.075}, {1738,0.073}, {1829,0.071},
  {1920,0.069}, {2012,0.067}, {2103,0.065}
};
static const int G1_COUNT = sizeof(G1_TABLE)/sizeof(DragEntry);
static const int G7_COUNT = sizeof(G7_TABLE)/sizeof(DragEntry);

// Global state variables
int currentBulletIndex = 0;
bool useImperial = false;  // false = metric (default), true = imperial
bool continuousMode = false;
bool calibrating = false;
double windSpeed = 0.0;     // m/s
double windDirDeg = 0.0;    // wind coming from this compass direction (0-360)
double lastRange = 0.0;     // last measured range (m)
bool haveRange = false;
char lastSolutionStr[24] = "";  // last displayed solution string for comparison
bool editing = false;
bool inWindMenu = false;
bool inWindInputMode = false;
unsigned long windEditStartTime = 0;
bool windEditHeld = false;

int selectedField = 0;  // 1=Bullet, 2=Wind Speed, 3=Wind Direction (only active when editing)

// Button state tracking for debouncing/edge detect
unsigned long modePressStart = 0;
bool modeHoldHandled = false;

// Debounce timing

const unsigned long debounceDelay = 50; // milliseconds

bool laserOn = false; // Tracks whether the laser is currently on

const char spinnerChars[] = {'|', '/', '-', '\\'};
int spinnerIndex = 0;

// Magnetometer calibration values
LSM303::vector<int16_t> magMin, magMax;
bool magCalibrated = false;

// Utility functions for ballistic calculations
const double R_AIR = 287.05;  // specific gas constant for dry air (J/kg·K)
double speedOfSound(double tempC) {
  // Speed of sound in air (approx) = sqrt(gamma * R_air * T(K)) with gamma ~1.4
  return sqrt(1.4 * R_AIR * (tempC + 273.15));
}
double airDensity(double tempC, double pressure_hPa, double humidity) {
  double T = tempC + 273.15;
  double P = pressure_hPa * 100.0;  // convert hPa to Pa
  // saturation vapor pressure (hPa) over water at tempC
  double saturation = 6.1078 * pow(10.0, (7.5 * tempC) / (tempC + 237.3));
  double vaporPressure = (humidity / 100.0) * saturation * 100.0;  // convert to Pa
  double dryAirPressure = P - vaporPressure;
  return dryAirPressure / (R_AIR * T);
}
double interpolateCd(double velocity_mps, const DragEntry* table, int tableSize) {
  // velocity in m/s, table velocities in ft/s
  double v_fps = velocity_mps * 3.28084;
  for (int i = 0; i < tableSize - 1; ++i) {
    if (v_fps >= table[i].vel_fps && v_fps <= table[i+1].vel_fps) {
      double v1 = table[i].vel_fps;
      double v2 = table[i+1].vel_fps;
      double cd1 = table[i].Cd;
      double cd2 = table[i+1].Cd;
      // Linear interpolation between table points
      return cd1 + (cd2 - cd1) * ((v_fps - v1) / (v2 - v1));
    }
  }
  // if velocity beyond last table entry, return last Cd value
  return table[tableSize - 1].Cd;
}
// Send command to rangefinder
void sendRangefinderCommand(const char* cmd) {
  RangeSerial.print(cmd);
  RangeSerial.print('\r');  // REVARSI expects \r line ending
  if (strcmp(cmd, "O") == 0) {
    laserOn = true;
  } else if (strcmp(cmd, "C") == 0) {
    laserOn = false;
  }
}
double transonicCorrection(double cd, double mach) {
  // Transonic range typically between Mach 1.2 and 0.8
  if (mach >= 0.8 && mach <= 1.2) {
    // Linear blend from no correction (Mach 1.2) to +30% drag (Mach 0.8)
    float t = (1.2 - mach) / 0.4;
    t = constrain(t, 0.0, 1.0);
    return cd * (1.0 + 0.3 * t);  // up to +30% drag
  }
  return cd;
}

// Compute ballistic trajectory and return drop/drift in mils

// GPIO pins for buttons
#define PIN_BTN_UP      13
#define PIN_BTN_DOWN    12
#define PIN_BTN_LEFT    14
#define PIN_BTN_RIGHT   27
#define PIN_BTN_CENTER  26  // center (select)
#define PIN_BTN_BACK    25  // back
#define PIN_BTN_MODE    33  // mode toggle (manual/continuous, hold for calibrate)
#define PIN_BTN_TRIGGER 32  // external trigger for rangefinder

// UART for rangefinder (using Serial2 on ESP32)
HardwareSerial RangeSerial(2);
#define PIN_RANGE_TX 17  // TX pin (if needed to send commands)
#define PIN_RANGE_RX 16  // RX pin (to receive distance data)
#define RANGE_BAUD   9600  // baud rate for rangefinder (adjust if needed)

// Bullet profile structure
struct Bullet {
  const char* code;
  const char* name;
  double bc;
  double muzzle_velocity; // m/s
  double weight_gr;       // grains
  double sight_height;    // m
  double diameter;        // m
  char drag_model;        // '1' for G1, '7' for G7
};

// Bullet library (all bullets from original script)
static const Bullet bulletList[] = {
  { "308_M80",    ".308 M80 Ball",           0.400, 840.0, 147.0, 0.06985, 0.00782, '1' },
  { "308_SUB",    ".308 Subsonic",           0.518, 325.0, 200.0, 0.06985, 0.00782, '1' },
  { "300BLK_SUP", ".300 Blackout Supersonic",0.305, 670.0, 125.0, 0.06985, 0.00782, '7' },
  { "300BLK_SUB", ".300 Blackout Subsonic",  0.660, 305.0, 220.0, 0.06985, 0.00782, '1' },
  { "556_M193",   "5.56 NATO 55gr M193",     0.243, 990.0,  55.0, 0.06985, 0.00570, '1' },
  { "68BLK_SUP",  "6.8 Blackout Supersonic", 0.370, 780.0, 110.0, 0.06985, 0.00680, '7' },
  { "68BLK_SUB",  "6.8 Blackout Subsonic",   0.450, 330.0, 180.0, 0.06985, 0.00680, '1' }
};
const int NUM_BULLETS = sizeof(bulletList)/sizeof(Bullet);

// Drag tables (velocity in ft/s, Cd)
struct DragEntry { int vel_fps; double Cd; };
static const DragEntry G1_TABLE[] = {
  {137, 0.262}, {228, 0.230}, {274, 0.211}, {366, 0.203}, {457, 0.198},
  {549, 0.195}, {640, 0.192}, {732, 0.188}, {823, 0.185}, {914, 0.182},
  {1006, 0.179}, {1097, 0.176}, {1189, 0.172}, {1280, 0.169}, {1372, 0.165},
  {1463, 0.162}, {1555, 0.159}, {1646, 0.156}, {1738, 0.152}, {1829, 0.149},
  {1920, 0.147}, {2012, 0.144}, {2103, 0.141}
};
static const DragEntry G7_TABLE[] = {
  {137, 0.120}, {228, 0.112}, {274, 0.107}, {366, 0.104}, {457, 0.102},
  {549, 0.100}, {640, 0.098}, {732, 0.095}, {823, 0.093}, {914, 0.091},
  {1006,0.089}, {1097,0.087}, {1189,0.085}, {1280,0.083}, {1372,0.081},
  {1463,0.079}, {1555,0.077}, {1646,0.075}, {1738,0.073}, {1829,0.071},
  {1920,0.069}, {2012,0.067}, {2103,0.065}
};
const int G1_COUNT = sizeof(G1_TABLE)/sizeof(DragEntry);
const int G7_COUNT = sizeof(G7_TABLE)/sizeof(DragEntry);

// Global state
int currentBulletIndex = 0;
bool continuousMode = false;
double windSpeed = 0.0;    // wind speed (m/s)
double windDirDeg = 0.0;   // wind direction (degrees, coming from)
double lastRange = 0.0;    // last measured range (m)
bool haveRange = false;
char lastSolutionStr[24] = "";
bool editing = false;
bool inWindMenu = false;
int selectedField = 0;  // 1=Bullet, 2=Wind Speed, 3=Wind Dir

// Button state tracking (for edge detection and long press)
ButtonState btnUp;
ButtonState btnDown;
ButtonState btnLeft;
ButtonState btnRight;
ButtonState btnCenter;
ButtonState btnBack;
ButtonState btnMode;
ButtonState btnTrigger;

// Magnetometer calibration
LSM303::vector<int16_t> magMin, magMax;
bool magCalibrated = false;

// Environmental constants
const double R_AIR = 287.05;  // J/(kg·K), specific gas constant for dry air

// Utility: speed of sound (approx) at given temperature (°C)
double speedOfSound(double tempC) {
  return sqrt(1.4 * R_AIR * (tempC + 273.15));
}

// Utility: air density (kg/m^3) given temp (°C), pressure (hPa), humidity (%)
double airDensity(double tempC, double pressure_hPa, double humidity) {
  double T = tempC + 273.15;
  double P = pressure_hPa * 100.0;  // convert hPa to Pa
  // Saturation vapor pressure (hPa) at tempC (Tetens formula)
  double saturation_hPa = 6.1078 * pow(10.0, (7.5 * tempC) / (tempC + 237.3));
  double vaporPressure = (humidity / 100.0) * saturation_hPa * 100.0;  // Pa
  double dryAirPressure = P - vaporPressure;
  return dryAirPressure / (R_AIR * T);
}

// Utility: interpolate drag coefficient for given velocity (m/s) from drag table
double interpolateCd(double velocity_mps, const DragEntry* table, int tableSize) {
  double v_fps = velocity_mps * 3.28084;
  for (int i = 0; i < tableSize - 1; ++i) {
    if (v_fps >= table[i].vel_fps && v_fps <= table[i+1].vel_fps) {
      double v1 = table[i].vel_fps;
      double v2 = table[i+1].vel_fps;
      double cd1 = table[i].Cd;
      double cd2 = table[i+1].Cd;
      // linear interpolation
      return cd1 + (cd2 - cd1) * ((v_fps - v1) / (v2 - v1));
    }
  }
  // If velocity beyond last entry, return last Cd
  return table[tableSize - 1].Cd;
}

// Utility: apply transonic drag correction
double transonicCorrection(double cd, double mach) {
  // Transonic range typically between Mach 1.2 and 0.8
  if (mach >= 0.8 && mach <= 1.2) {
    // Linear blend from no correction (Mach 1.2) to +30% drag (Mach 0.8)
    float t = (1.2 - mach) / 0.4;
    t = constrain(t, 0.0, 1.0);
    return cd * (1.0 + 0.3 * t);  // up to +30% drag
  }
  return cd;
}

double calculatePitch(const LSM303::vector<int16_t>& accel) {
  return atan2(-accel.x, sqrt((float)accel.y * accel.y + (float)accel.z * accel.z));
}

double calculateHeading() {
  return compass.heading((LSM303::vector<int>){0, 0, 1});
}


// Compute ballistic drop and drift in mils for current state
void computeBallistic(double range_m, double pitchRad, double headingDeg,
                      double windSpeed_mps, double windDirDeg,
                      double& outDropMils, double& outDriftMils) {
  if (range_m <= 0) {
    outDropMils = 0.0;
    outDriftMils = 0.0;
    return;
  }
  // Determine horizontal distance and target elevation relative to shooter
  double horizontalRange = range_m * cos(pitchRad);
  double targetElevation = range_m * sin(pitchRad);
  // Use current bullet parameters
  const Bullet& bullet = bulletList[currentBulletIndex];
  // Environmental measurements
  if (!validateBME280()) {
    Wire.begin();  // Reset I2C
    bme.begin(BME280_I2C_ADDR_PRIMARY);
    return; // Skip this frame
  }

  double tempC = bme.readTemperature();
  double pressure_hPa = bme.readPressure() / 100.0;
  double humidity = bme.readHumidity();
  double rho = airDensity(tempC, pressure_hPa, humidity);
  double sos = speedOfSound(tempC);
  // Bullet parameters for simulation
  double g = 9.81;
  double dt = 0.01;
  double x = 0.0;
  double y = bullet.sight_height;
  double v = bullet.muzzle_velocity;
  // Convert bullet weight to kg
  double mass = bullet.weight_gr * 0.00006479891;
  // Cross-sectional area of bullet (m^2)
  double A = PI * pow(bullet.diameter / 2.0, 2);
  // Wind relative angle: compute angle between wind direction and shooter's heading
  // relativeWindDeg = headingDeg - windDirDeg (mod 360)
  double relativeWindDeg = fmod((headingDeg - windDirDeg + 360.0), 360.0);
  double windAngleRad = relativeWindDeg * DEG_TO_RAD;
  double drift = 0.0;
  // Select drag table based on G1 or G7
  const DragEntry* dragTable = (bullet.drag_model == '1') ? G1_TABLE : G7_TABLE;
  int dragSize = (bullet.drag_model == '1') ? G1_COUNT : G7_COUNT;
  // Integrate trajectory until horizontal distance reached or velocity drops too low
  while (x < horizontalRange && v > 100.0) {
    double mach = v / sos;
    double Cd = interpolateCd(v, dragTable, dragSize);
    Cd = transonicCorrection(Cd, mach);
    // Drag force Fd = 0.5 * rho * Cd * A * v^2
    // Dynamic pressure = 0.5 * rho * v^2
    // Fd = dynamic pressure * Cd * A
    double dynamicPressure = 0.5 * rho * v * v;
    double Fd = dynamicPressure * Cd * A;

    double a_drag = Fd / mass;
    // Update velocity and positions
    v -= a_drag * dt;
    if (v < 0.1) v = 0.1;  // prevent velocity from going negative or zero
    y -= g * dt;
    drift += windSpeed_mps * sin(windAngleRad) * dt;
    x += v * dt;
    // (time t advanced by dt if needed, but we don't use time output)
  }
  // Compute drop and drift
  double drop = bullet.sight_height - y - targetElevation;
  double mils = (horizontalRange > 0.0) ? (drop / horizontalRange) * 1000.0 : 0.0;
  double drift_mils = (horizontalRange > 0.0) ? (drift / horizontalRange) * 1000.0 : 0.0;
  outDropMils = mils;
  outDriftMils = drift_mils;
}

// Draw the current UI (ballistic solution and fields)
void drawDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Prepare strings for fields
  const Bullet& bullet = bulletList[currentBulletIndex];
  char bulletStr[20];
  snprintf(bulletStr, sizeof(bulletStr), "%s", bullet.code);  // bullet code or short name
  char rangeStr[10];
  if (haveRange) {
    if (useImperial) {
    snprintf(rangeStr, sizeof(rangeStr), "%.0fyd", lastRange * 1.09361);
  } else {
    snprintf(rangeStr, sizeof(rangeStr), "%.0fm", lastRange);
  }

  } else {
    snprintf(rangeStr, sizeof(rangeStr), "---m");
  }
  char windStr[20];
  if (useImperial) {
    snprintf(windStr, sizeof(windStr), "%.0fmph %.0fdeg", windSpeed * 2.23694, windDirDeg);
  } else {
    snprintf(windStr, sizeof(windStr), "%.0fm/s %.0fdeg", windSpeed, windDirDeg);
  }

  // We also have the solution string ready (lastSolutionStr updated after computeBallistic)

  // We will display:
  // Line1: Bullet code and range
  // Line2: Wind info
  // Line3: Solution (elevation and windage)
  // Line4: Mode indicator or empty (and used for highlighting if needed)
  // If editing, we highlight fields accordingly by inverting their text.

  // Line 1: Bullet and Range
  display.setCursor(0, 0);
  if (editing && selectedField == 1) {
    // Highlight bullet (invert bullet text)
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print(bulletStr);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.print(" ");
    display.print(rangeStr);
    display.setTextColor(SSD1306_WHITE);
  } else {
    // Not editing bullet, or not selected
    display.print(bulletStr);
    display.print(" ");
    display.print(rangeStr);
  }

  // Line 2: Wind
  display.setCursor(0, 8);
  display.print("Wind: ");
  // Determine which part of wind is highlighted if editing
  if (editing && (selectedField == 2 || selectedField == 3)) {
    // We will split wind into two parts: speed and direction
    // Find the position of space between speed and direction in windStr
    // windStr format: "<speed>m/s <dir>deg"
    char* dirPart = strstr(windStr, " ");
    if (dirPart) {
      *dirPart = '\0'; // temporarily split
      const char* speedPart = windStr;
      const char* directionPart = dirPart + 1;
      if (selectedField == 2) {
        // Highlight wind speed
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(speedPart);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.print(" ");
        display.print(directionPart);
        display.setTextColor(SSD1306_WHITE);
      } else if (selectedField == 3) {
        // Highlight wind direction
        display.print(speedPart);
        display.print(" ");
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(directionPart);
        display.setTextColor(SSD1306_WHITE);
      }
      *dirPart = ' '; // restore string
    } else {
      // safety: if format unexpected, just print normally
      display.print(windStr);
    }
  } else {
    // Not editing wind fields
    display.print(windStr);
  }

  // Line 3: Solution
  display.setCursor(0, 16);
  // Determine vertical and horizontal adjustments and format them
  // lastSolutionStr is maintained globally after last computeBallistic
  display.print(lastSolutionStr);

  // Line 4: Mode indicator (manual/continuous) if not editing (or calibrating)
  display.setCursor(0, 24);
  if (continuousMode) {
    display.print("Mode: Continuous");
  } else {
    display.print("Mode: Manual");
  }

// Show laser-on indicator (small dot in top-right corner)
if (laserOn) {
  display.fillCircle(123, 1, 2, SSD1306_WHITE);  // Adjust coordinates if needed
}
// Draw units indicator
display.setTextColor(SSD1306_WHITE);
display.setTextSize(1);
display.setCursor(110, 0);  // Top-right corner, tweak as needed
display.print(useImperial ? "yd" : "m");

// Show calibration status icon (bottom-right corner)
if (inWindInputMode) {
  display.setCursor(0, 24);
  display.print("Editing Wind");
}
if (magCalibrated) {
  // Draw checkmark or solid box (check style)
  display.drawPixel(124, 30, SSD1306_WHITE);
  display.drawPixel(125, 29, SSD1306_WHITE);
  display.drawPixel(126, 28, SSD1306_WHITE);
  display.drawPixel(125, 31, SSD1306_WHITE);
  display.drawPixel(126, 30, SSD1306_WHITE);
  display.drawPixel(127, 29, SSD1306_WHITE);
} else {
  // Draw "X" mark (x style) for uncalibrated
  display.drawLine(124, 28, 127, 31, SSD1306_WHITE);
  display.drawLine(127, 28, 124, 31, SSD1306_WHITE);
}

  display.display();
}
// Read rangefinder serial data
bool readRangefinder() {
  static String incoming = "";
  bool gotReading = false;

 if ((millis() - lastStart) > 200 && incoming.length() > 0) {
  // Timeout waiting for rest of message
  incoming = "";
  return false;
}



  while (RangeSerial.available()) {
    char c = (char)RangeSerial.read();
lastStart = millis();  // reset timeout timer when any character is read

    if (c == '\r' || c == '\n') {
      if (incoming.length() > 0) {
        // Remove whitespace or units like 'm' if present
        incoming.trim();

        // Strip trailing 'm' or other units if accidentally appended
        if (incoming.endsWith("m")) {
          incoming.remove(incoming.length() - 1);
        }

        // Attempt conversion
        double distance = incoming.toDouble();

        // Sanity check for realistic ranges
        if (distance > 0.0 && distance < 1000.0) {
          lastRange = distance;
          haveRange = true;
          gotReading = true;
        }

        // Optional debug print:
        // else {
        //   Serial.print("Bad range string: "); Serial.println(incoming);
        // }

      }
      incoming = "";
    } else {
      // Accept only numbers, dot, or 'm' (filter out junk characters)
      if ((c >= '0' && c <= '9') || c == '.' || c == 'm') {
        incoming += c;
      }

      // If input is suspiciously long, flush it
      if (incoming.length() > 20) {
        incoming = "";
      }
    }
  }

  return gotReading;
}


// Handle entering calibration mode
void startCalibration() {
  sendRangefinderCommand("O");  // Turn laser ON for calibration
  calibrating = true;
  // Reset min/max
  magMin = {+32767, +32767, +32767};
  magMax = {-32768, -32768, -32768};
  // Display instructions
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.println("Calibrating...");
  display.setCursor(0, 16);
  display.println("Rotate sensor");
  display.display();
}

// Handle the calibration loop (should be called frequently while calibrating)
void performCalibration() {
  // Read magnetometer (and accelerometer, though we only need magnetometer for calibration)
  compass.read();
  // Update min/max for magnetometer
  if (compass.m.x < magMin.x) magMin.x = compass.m.x;
  if (compass.m.y < magMin.y) magMin.y = compass.m.y;
  if (compass.m.z < magMin.z) magMin.z = compass.m.z;
  if (compass.m.x > magMax.x) magMax.x = compass.m.x;
  if (compass.m.y > magMax.y) magMax.y = compass.m.y;
  if (compass.m.z > magMax.z) magMax.z = compass.m.z;
  // Draw spinner frame in top-right
  display.drawRect(122, 0, 6, 6, SSD1306_WHITE);  // Small square
  display.setCursor(124, 1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.print(spinnerChars[spinnerIndex]);

  display.display();
  spinnerIndex = (spinnerIndex + 1) % 4;
}

// Finish calibration: save values and apply
void finishCalibration() {
  sendRangefinderCommand("C");  // Turn laser OFF when exiting calibration
  calibrating = false;
  // Save calibration to preferences
  prefs.putShort("magMinX", magMin.x);
  prefs.putShort("magMinY", magMin.y);
  prefs.putShort("magMinZ", magMin.z);
  prefs.putShort("magMaxX", magMax.x);
  prefs.putShort("magMaxY", magMax.y);
  prefs.putShort("magMaxZ", magMax.z);
  prefs.putBool("magCal", true);
  // Apply calibration to compass object
  compass.m_min = magMin;
  compass.m_max = magMax;
  magCalibrated = true;
  // Confirm done on display briefly
  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Calibration done");
  display.display();
  delay(1000);
  // Redraw normal display after calibration
  drawDisplay();
}

// Setup hardware
void setup() {
  // Initialize serial for debugging (optional)
  // Serial.begin(115200);
  // Serial.println("Starting Ballistica ESP32...");

  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // if display not found, hang
    // Serial.println("SSD1306 not found");
    while(true);
  }
  display.clearDisplay();
  display.display();

  // Show boot splash screen
  display.clearDisplay();
  display.setTextSize(2);  // Large text for splash
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);  // Adjust for centering as needed
  display.println("RAT 9");
  display.setCursor(10, 28);
  display.setTextSize(1);
  display.println("Ballistics Calculator");
  display.display();
  delay(2500);  // Show splash for 2.5 seconds


  // Initialize I2C sensors
  Wire.begin();
  bool bmeOK = bme.begin(BME280_ADDR_PRIMARY);
  if (!bmeOK) {
    bmeOK = bme.begin(BME280_ADDR_SECONDARY);
  }
  // It's okay if BME280 fails (should not, if connected properly)
  compass.init();
  compass.enableDefault();

  // Initialize rangefinder serial
  RangeSerial.begin(RANGE_BAUD, SERIAL_8N1, PIN_RANGE_RX, PIN_RANGE_TX);

  // Setup button inputs (use internal pull-ups)
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_CENTER, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_TRIGGER, INPUT_PULLUP);

  // Load saved preferences
  prefs.begin("Ballistica", false);
  currentBulletIndex = prefs.getInt("bulletIndex", 0);
  if (currentBulletIndex < 0 || currentBulletIndex >= NUM_BULLETS) currentBulletIndex = 0;
  windSpeed = prefs.getDouble("windSpeed", 0.0);
  windDirDeg = prefs.getDouble("windDir", 0.0);
  useImperial = prefs.getBool("unitsImperial", false);

  // Load magnetometer calibration if exists
  if (prefs.getBool("magCal", false)) {
    magMin.x = prefs.getShort("magMinX", 0);
    magMin.y = prefs.getShort("magMinY", 0);
    magMin.z = prefs.getShort("magMinZ", 0);
    magMax.x = prefs.getShort("magMaxX", 0);
    magMax.y = prefs.getShort("magMaxY", 0);
    magMax.z = prefs.getShort("magMaxZ", 0);
    compass.m_min = magMin;
    compass.m_max = magMax;
    magCalibrated = true;
  }

  // Initial draw of UI
  strcpy(lastSolutionStr, "----");  // no solution yet
  drawDisplay();
}

bool validateBME280() {
  float t = bme.readTemperature();
  float p = bme.readPressure();
  float h = bme.readHumidity();

  // Validate ranges
  if (isnan(t) || t < -40 || t > 85) return false;
  if (isnan(p) || p < 30000 || p > 110000) return false;
  if (isnan(h) || h < 0 || h > 100) return false;
  return true;
}

void updateButtonState(ButtonState& button, int pin, unsigned long debounceDelay = 50, unsigned long longPressDuration = 1000) {
  bool reading = (digitalRead(pin) == LOW); // Active LOW buttons

  if (reading != button.last) {
    // Reset debounce timer
    button.pressStart = millis();
  }

  // If stable for debounceDelay, update state
  if ((millis() - button.pressStart) > debounceDelay) {
    if (reading != button.current) {
      button.current = reading;
      if (button.current) {
        button.justPressed = true;
        button.longPressed = false;
        button.pressStart = millis(); // Start timing long press
      } else {
        button.justReleased = true;
        button.longPressed = false;
      }
    } else {
      button.justPressed = false;
      button.justReleased = false;
    }

    if (button.current && !button.longPressed && (millis() - button.pressStart >= longPressDuration)) {
      button.longPressed = true;
    }
  }

  button.last = reading;
}


// Poll buttons and handle UI logic
void loop() {
  // --- Update all button states ---
  updateButtonState(btnUp,      PIN_BTN_UP);
  updateButtonState(btnDown,    PIN_BTN_DOWN);
  updateButtonState(btnLeft,    PIN_BTN_LEFT);
  updateButtonState(btnRight,   PIN_BTN_RIGHT);
  updateButtonState(btnCenter,  PIN_BTN_CENTER);
  updateButtonState(btnBack,    PIN_BTN_BACK);
  updateButtonState(btnMode,    PIN_BTN_MODE);
  updateButtonState(btnTrigger, PIN_BTN_TRIGGER);

// Step 2: Handle entering wind input mode via long RIGHT press
if (btnRight.current && !windEditHeld && btnRight.pressStart > 0 &&
    (millis() - btnRight.pressStart >= 1000)) {
  inWindInputMode = true;
  windEditHeld = true;
  drawDisplay();  // Reflect change in UI
}

if (!btnRight.current) {
  windEditHeld = false;  // Reset for next long press
}

// If we're in wind input mode, override normal input logic
if (inWindInputMode) {
  // Adjust wind speed with LEFT/RIGHT
  if (btnLeft.justPressed) {
    windSpeed -= 0.5;
    if (windSpeed < 0) windSpeed = 0;
    drawDisplay();
  }
  if (btnRight.justPressed) {
    windSpeed += 0.5;
    drawDisplay();
  }

  // Adjust wind direction with UP/DOWN
  if (btnUp.justPressed) {
    windDirDeg += 5.0;
    if (windDirDeg >= 360) windDirDeg -= 360;
    drawDisplay();
  }
  if (btnDown.justPressed) {
    windDirDeg -= 5.0;
    if (windDirDeg < 0) windDirDeg += 360;
    drawDisplay();
  }

  // Exit wind input mode with BACK or CENTER
  if (btnBack.justPressed || btnCenter.justPressed) {
    prefs.putDouble("windSpeed", windSpeed);
    prefs.putDouble("windDir", windDirDeg);
    inWindInputMode = false;
    drawDisplay();
    delay(300);  // Basic debounce so you don’t immediately trigger other inputs
  }

  return;  // Skip rest of loop while in wind mode
}


  // If in calibration mode, handle calibration process
  if (calibrating) {
    // Check for back button to finish calibration
    bool backPressed = (digitalRead(PIN_BTN_BACK) == LOW);
    if (backPressed) {
      finishCalibration();
      // Consume this back press so it doesn't act in normal mode
      lastStateBack = backPressed;
      return;
    }
    // Continue calibration updates at a limited rate
    static unsigned long lastCalUpdate = 0;
    if (millis() - lastCalUpdate >= 100) {
      performCalibration();
      lastCalUpdate = millis();
    }
    // In calibration mode, do not update main display or ballistic calc
    return;
  }


  // BACK + CENTER combo to toggle units (metric ↔ imperial)
  static unsigned long comboStart = 0;
  if (stateBack && stateCenter) {
    if (comboStart == 0) comboStart = millis();
    else if (millis() - comboStart > 1000) {
      useImperial = !useImperial;
      prefs.putBool("unitsImperial", useImperial);
      drawDisplay(); // refresh screen with new units
      delay(500); // simple debounce
    }
  } else {
    comboStart = 0;
  }

  // Handle Mode button (toggle manual/continuous or long-press for calibration)
  if (btnMode.justPressed()) {
    // Mode button just pressed
    modePressStart = millis();
    modeHoldHandled = false;
  }
  if (stateMode && !modeHoldHandled && millis() - modePressStart > 3000) {
    // Long press detected - enter calibration
    modeHoldHandled = true;
    startCalibration();
    // Do not toggle continuous mode on release in this case
  }
  if (btnMode.justReleased()) {
    // Mode button released
    if (!modeHoldHandled) {
      // Short press (no calibrate triggered) - toggle mode
      continuousMode = !continuousMode;
      // Indicate mode change (e.g., we could flash or just update display)
      // Mark display for update
      drawDisplay();
    }
    modePressStart = 0;
    modeHoldHandled = false;
  }

  // Handle Trigger button (trigger rangefinder reading)
  if (btnTrigger.justPressed()) {
  if (!continuousMode) {
    sendRangefinderCommand("D");  // Manual mode triggers laser
  }
}
  bool newRange = false;
  // Continuously read from rangefinder serial if data available
  newRange = readRangefinder();
  if (newRange) {
    // If a new range was obtained, recalc solution if manual mode
    if (!continuousMode) {
      // Recalculate ballistic solution for new range immediately
      // Get current orientation (pitch from accelerometer, heading from magnetometer)
      if (!compass.read()) {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Compass read error!");
  display.display();
  delay(1000);
  return;
}
      // Apply calibration if available for heading calculation
      if (magCalibrated) {
        compass.m_min = magMin;
        compass.m_max = magMax;
      }
     double pitchRad = calculatePitch(compass.a);
     double headingDeg = calculateHeading();

      // Calculate ballistic solution
      double dropMils, driftMils;
      computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
      // Format solution string (e.g., "3.4U -5.8L" or "3.4U 5.8R")
      char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
      char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
      double vertAdj = fabs(dropMils);
      double horizAdj = fabs(driftMils);
      snprintf(lastSolutionStr, sizeof(lastSolutionStr), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
      // Update display
      drawDisplay();
    }
  }
  // Handle arrow buttons for navigation/adjustment
  if (stateCenter && !lastStateCenter) {
    // Center pressed: toggle editing mode or confirm selection
   if (!editing && !inWindMenu) {
  // First press: enter field selection mode
  editing = true;
  selectedField = 1;
} else if (editing && selectedField == 1) {
  // Press again while on bullet = enter Wind Menu
  editing = false;
  inWindMenu = true;
  selectedField = 2;  // Start at wind speed
} else if (inWindMenu) {
  // Press again in Wind Menu = exit Wind Menu
  inWindMenu = false;
  selectedField = 0;
  // Save wind settings
  prefs.putDouble("windSpeed", windSpeed);
  prefs.putDouble("windDir", windDirDeg);
} else {
  // Cycle fields normally
  selectedField++;
  if (selectedField > 3) {
    editing = false;
    selectedField = 0;
    // Save bullet and other settings
    prefs.putInt("bulletIndex", currentBulletIndex);
    prefs.putDouble("windSpeed", windSpeed);
    prefs.putDouble("windDir", windDirDeg);
    prefs.putBool("unitsImperial", useImperial);
  }
}
    drawDisplay();
  }

  if (stateBack && !lastStateBack) {
    // Back pressed: if editing, cancel/exit editing without saving (or simply exit)
    if (editing) {
      editing = false;
      selectedField = 0;
      // We might revert changes if we had a mechanism, but since we apply immediately on adjust, just exit.
      drawDisplay();
    }
    // If not editing, back could be used to do nothing (or reserved for calibrate exit which we handle separately).
  }

  if (editing) {
	if (inWindMenu) {
 	 // Handle up/down to toggle between wind speed and direction
 	 if (btnUp.justPressed()) {
  	  selectedField = (selectedField == 2) ? 3 : 2;
   	 drawDisplay();
	  }
	  if (btnDown.justPressed()) {
  	  selectedField = (selectedField == 3) ? 2 : 3;
   	 drawDisplay();
 	 }

  // Handle directional changes to value
  if (btnLeft.justPressed() || btnRight.justPressed() ||
      btnUp.justPressed() || btnDown.justPressed()) {

    int delta = 0;
    if (btnLeft.justPressed()) delta = -1;
    if (btnRight.justPressed()) delta = 1;
    if (btnUp.justPressed()) delta = 5;
    if (btnDown.justPressed()) delta = -5;

    if (selectedField == 2) {
      // Wind speed adjustment
      windSpeed += delta * 0.1;
      windSpeed = constrain(windSpeed, 0.0, 50.0); // Clamp between 0 and 50 m/s
    } else if (selectedField == 3) {
      // Wind direction adjustment
      windDirDeg += delta;
      windDirDeg = fmod((windDirDeg + 360.0), 360.0); // Keep between 0–359
    }

    // Recompute ballistic solution if range is available
    if (!continuousMode && haveRange) {
      compass.read();
      if (magCalibrated) {
        compass.m_min = magMin;
        compass.m_max = magMax;
      }
      double pitchRad = calculatePitch(compass.a);
      double headingDeg = calculateHeading();
      double dropMils, driftMils;
      computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
      char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
      char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
      double vertAdj = fabs(dropMils);
      double horizAdj = fabs(driftMils);
      snprintf(lastSolutionStr, sizeof(lastSolutionStr), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
    }

    drawDisplay();
  }

  return; // Skip normal editing logic if in Wind Menu
}

    // Navigation within editing mode
    if (btnUp.justPressed()) {
      // Move selection up (previous field)
      selectedField--;
      if (selectedField < 1) selectedField = 3;
      drawDisplay();
    }
    if (stateDown && !lastStateDown) {
      // Move selection down (next field)
      selectedField++;
      if (selectedField > 3) selectedField = 1;
      drawDisplay();
    }
    if ((stateLeft && !lastStateLeft) || (stateRight && !lastStateRight) ||
    (stateUp && !lastStateUp) || (stateDown && !lastStateDown)) {
  
  int xDirection = 0;
  int yDirection = 0;

  if (stateLeft && !lastStateLeft)  xDirection = -1;
  if (stateRight && !lastStateRight) xDirection = 1;
  if (stateUp && !lastStateUp)    yDirection = -1;
  if (stateDown && !lastStateDown)  yDirection = 1;

  if (selectedField == 1) {
    currentBulletIndex += yDirection * 2 + xDirection;
    if (currentBulletIndex < 0) currentBulletIndex = NUM_BULLETS - 1;
    if (currentBulletIndex >= NUM_BULLETS) currentBulletIndex = 0;
    } else if (selectedField == 2) {
      windSpeed += xDirection * 1.0 + yDirection * 5.0;
      windSpeed = constrain(windSpeed, 0.0, 50.0);  // Clamp wind speed (m/s) to 0–50

   } else if (selectedField == 3) {
    windDirDeg += xDirection * 5.0 + yDirection * 5.0;
    windDirDeg = fmod((windDirDeg + 360.0), 360.0);  // Always wraps between 0-359.99

  }

  if (!continuousMode && haveRange) {
    compass.read();
    if (magCalibrated) {
      compass.m_min = magMin;
      compass.m_max = magMax;
    }
    double pitchRad = calculatePitch(compass.a);
    double headingDeg = calculateHeading();
    double dropMils, driftMils;
    computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
    char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
    char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
    double vertAdj = fabs(dropMils);
    double horizAdj = fabs(driftMils);
    snprintf(lastSolutionStr, sizeof(lastSolutionStr), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
  }

  drawDisplay();
}

  // Continuous mode: update ballistic solution continuously
  static unsigned long lastContUpdate = 0;
  if (continuousMode && (millis() - lastContUpdate >= 100)) {
    lastContUpdate = millis();
    // Read orientation and environment continuously
    compass.read();
    if (magCalibrated) {
      compass.m_min = magMin;
      compass.m_max = magMax;
    }
    // Compute pitch and heading
    double pitchRad = atan2(-(double)compass.a.x, sqrt((double)compass.a.y * compass.a.y + (double)compass.a.z * compass.a.z));
    double headingDeg = compass.heading((LSM303::vector<int>){0, 0, 1});
    // If in continuous mode, we might not have an automatically changing range unless rangefinder provides streaming.
    // We will use lastRange (from last trigger press or measurement).
    static unsigned long lastRangePoll = 0;
    if (millis() - lastRangePoll >= 10000) {  // Every 10 seconds
      sendRangefinderCommand("D");  // Trigger laser for continuous mode
      lastRangePoll = millis();
    }

    if (!haveRange) {
      // If no range measured yet, skip until we have one
    } else {
      double dropMils, driftMils;
      computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
      char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
      char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
      double vertAdj = fabs(dropMils);
      double horizAdj = fabs(driftMils);
      char solutionBuf[16];
      snprintf(solutionBuf, sizeof(solutionBuf), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
      // Only update display if the solution or other displayed info has changed (to avoid flicker)
      if (strcmp(solutionBuf, lastSolutionStr) != 0 ||
          editing || !continuousMode) {
        strcpy(lastSolutionStr, solutionBuf);
        drawDisplay();
      }
    }
  }
}
