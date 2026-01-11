#include <M5Stack.h>
#include "X2driver.h"
#include "lidarcar.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#define FRONT_ANGLE 360
#define TFT_GREY 0x5AEB


enum Heading { NORTH, EAST, SOUTH, WEST };
Heading heading = NORTH;

// ---------- Grid params (tune) ----------
static const int MAP_W = 150;          // cells
static const int MAP_H = 150;          // cells
static const float RES_MM = 50.0f;     // mm per cell (50mm => 8m x 8m)

// Robot starts at center of grid
static const int ORIGIN_X = MAP_W / 2;
static const int ORIGIN_Y = MAP_H / 2;

// ---------- Log-odds fixed-point ----------
static const int LO_SCALE = 1000;
static const int16_t LO_MIN = -3000;
static const int16_t LO_MAX =  3000;

// Sensor model probabilities (tune)
static const float P_OCC  = 0.70f;     // endpoint occupied
static const float P_FREE = 0.30f;     // ray cells free

// Threshold later for A* (occupied if > this)
static const int16_t LO_OCC_THRESH = 800;

// Lidar validity (match your cleaning)
static const float LIDAR_MIN_MM = 80.0f;
static const float LIDAR_MAX_MM = 8000.0f;
static const float NO_HIT_MARGIN_MM = 80.0f;

// Downsample beams for speed: 2 => 1deg, 4 => 2deg
static const int BEAM_STEP = 2;

// The map: log-odds grid
static int16_t logOdds[MAP_H][MAP_W];

// Pose (mm) in world frame: +x east, +y north
static float robot_x_mm = 0.0f;
static float robot_y_mm = 0.0f;

// TODO
// Timestamp-based translation tuning (IMPORTANT to calibrate)
static float MM_PER_MS_FWD = 0.35f;  // forward mm per ms
static float MM_PER_MS_REV = 0.30f;  // backward mm per ms

// Track last commanded motion for integration
static int8_t lastCmdY = 0;
static unsigned long lastCmdTime = 0;

// Thread safety for lidar array snapshot (mazeTask reads while lidarTask writes)
static portMUX_TYPE lidarMux = portMUX_INITIALIZER_UNLOCKED;

// Precomputed log-odds deltas
static int16_t LO_OCC_DELTA;
static int16_t LO_FREE_DELTA;

// Heading enum -> radians (0=east, +90=north)
static inline float headingToThetaRad(Heading h) {
  switch (h) {
    case EAST:  return 0.0f;
    case NORTH: return PI * 0.5f;
    case WEST:  return PI;
    case SOUTH: return -PI * 0.5f;
  }
  return 0.0f;
}

// World mm -> grid cell
static inline bool worldToGrid(float x_mm, float y_mm, int &gx, int &gy) {
  gx = ORIGIN_X + (int)floorf(x_mm / RES_MM);
  gy = ORIGIN_Y - (int)floorf(y_mm / RES_MM); // y+ north; row index grows downward
  return (gx >= 0 && gx < MAP_W && gy >= 0 && gy < MAP_H);
}

// Clamp + add delta to a cell
static inline void addLogOdds(int gx, int gy, int16_t delta) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return;
  int v = (int)logOdds[gy][gx] + (int)delta;
  if (v > LO_MAX) v = LO_MAX;
  if (v < LO_MIN) v = LO_MIN;
  logOdds[gy][gx] = (int16_t)v;
}

// p -> log-odds fixed-point
static inline int16_t probToLogOddsDelta(float p) {
  float l = logf(p / (1.0f - p));
  return (int16_t)(l * (float)LO_SCALE);
}
// Bresenham ray: mark free along path, occ at endpoint if hit==true
static void updateRay(int x0, int y0, int x1, int y1, bool hit) {
  int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  bool first = true;

  while (true) {
    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }

    if (x0 < 0 || x0 >= MAP_W || y0 < 0 || y0 >= MAP_H) return;

    // skip robot cell itself
    if (!first) {
      if (x0 == x1 && y0 == y1) break; // endpoint handled below
      addLogOdds(x0, y0, LO_FREE_DELTA);
    }
    first = false;
  }

  if (hit) addLogOdds(x1, y1, LO_OCC_DELTA);
}
// Process one 720-beam scan -> update logOdds
static void updateMapFromScan(const float *scan720) {
  int rx, ry;
  if (!worldToGrid(robot_x_mm, robot_y_mm, rx, ry)) return;

  float baseTheta = headingToThetaRad(heading);

  for (int i = 0; i < 720; i += BEAM_STEP) {
    float r = scan720[i];
    if (r < LIDAR_MIN_MM || r > LIDAR_MAX_MM) continue;

    // your convention: index 360 is "front"
    float beamDeg = (float)(i - 360) * 0.5f;   // 0 deg = forward
    float ang = baseTheta + beamDeg * DEG_TO_RAD;

    bool hit = (r < (LIDAR_MAX_MM - NO_HIT_MARGIN_MM)); // max distance we define 

    float ex = robot_x_mm + r * cosf(ang);
    float ey = robot_y_mm + r * sinf(ang);

    int gx, gy;
    if (!worldToGrid(ex, ey, gx, gy)) {
      // If endpoint is out of map, clip it inward and treat as "no hit"
      float r2 = r;
      bool ok = false;
      for (int k = 0; k < 10; k++) {
        r2 *= 0.85f;
        ex = robot_x_mm + r2 * cosf(ang);
        ey = robot_y_mm + r2 * sinf(ang);
        if (worldToGrid(ex, ey, gx, gy)) { ok = true; break; }
      }
      if (!ok) continue;
      hit = false;
    }

    updateRay(rx, ry, gx, gy, hit);
  }
}

void updateHeading(int turn) {
    if (turn == 90)      heading = (Heading)((heading + 1) % 4);
    else if (turn == -90) heading = (Heading)((heading + 3) % 4);
    else if (turn == 180) heading = (Heading)((heading + 2) % 4);
}

struct LidarPoint {
    bool leftOpen;
    bool rightOpen;
    bool frontOpen;

    LidarPoint(bool left = false, bool right = false, bool front = false)
        : leftOpen(left), rightOpen(right), frontOpen(front) {}
};

LidarPoint p1(false, false, false);


bool mazeMode = false;
bool mazeSolved = false;

unsigned long uTurnCooldown = 0;

float cleanedMap[720];

X2 lidar;
LidarCar lidarcar;

const float threshold_front = 105.0;
const float threshhold_sides = 150.0;


bool isNewPath(bool openRight, bool openFront, bool openLeft) {
    
    if (millis() < uTurnCooldown) {
        openRight = false;
    }

    p1.rightOpen = openRight;
    p1.frontOpen = openFront;
    p1.leftOpen = openLeft;

    return true;
}


void sendControlCommand(int8_t X, int8_t Y, uint8_t A) {
    unsigned long now = millis();
    if (lastCmdTime == 0) lastCmdTime = now;

    // integrate translation using LAST commanded Y over elapsed time
    unsigned long dt = now - lastCmdTime;
    if (dt > 0 && lastCmdY != 0) {
        float mmPerMs = (lastCmdY > 0) ? MM_PER_MS_FWD : MM_PER_MS_REV;
        float dist = mmPerMs * (float)dt * (float)lastCmdY; // signed

        float theta = headingToThetaRad(heading);
        robot_x_mm += dist * cosf(theta);
        robot_y_mm += dist * sinf(theta);
    }

    lastCmdTime = now;
    lastCmdY = Y;

    // your original motor command
    lidarcar.ControlWheel(X, Y, A);
}


float getAvgDistRange(int startIdx, int endIdx) {
    float sum = 0;
    int count = 0;
    

    if (startIdx > endIdx) {
        for (int i = startIdx; i < 720; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                sum += d;
                count++;
            }
        }

        for (int i = 0; i <= endIdx; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                sum += d;
                count++;
            }
        }
    } else {

        for (int i = startIdx; i <= endIdx; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                sum += d;
                count++;
            }
        }
    }
    
    return (count == 0) ? 0 : (sum / count);
}


float getMinDistRange(int startIdx, int endIdx) {
    float minDist = 9999.0;
    bool foundValid = false;
    
    if (startIdx > endIdx) {
        for (int i = startIdx; i < 720; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                minDist = min(minDist, d);
                foundValid = true;
            }
        }
        for (int i = 0; i <= endIdx; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                minDist = min(minDist, d);
                foundValid = true;
            }
        }
    } else {
        for (int i = startIdx; i <= endIdx; i++) {
            float d = lidar.tmpmap.mapdata[i];
            if (d > 0) {
                minDist = min(minDist, d);
                foundValid = true;
            }
        }
    }
    
    return foundValid ? minDist : 0;
}


void printAllDistances() {
    Serial.println("=== LIDAR 360° DATA ===");
    for (int i = 0; i < 720; i++) {
        int angle = i / 2;
        Serial.printf("%3d° : %.1f mm\n", angle, lidar.tmpmap.mapdata[i]);
    }
    Serial.println("=================================\n");
}

void cleanLidarData(float *in, float *out) {
    const int n = 720;
    const int gap = 2;
    const float minValid = 80.0;
    const float maxValid = 8000.0;

    for (int i = 0; i < n; i++) out[i] = in[i];

    for (int i = 0; i < n; i++) {
        if (in[i] >= minValid && in[i] <= maxValid)
            continue;

        float left = -1, right = -1;

        for (int k = 1; k <= gap; k++) {
            int idx = (i - k + n) % n;
            if (in[idx] >= minValid && in[idx] <= maxValid) {
                left = in[idx];
                break;
            }
        }

        for (int k = 1; k <= gap; k++) {
            int idx = (i + k) % n;
            if (in[idx] >= minValid && in[idx] <= maxValid) {
                right = in[idx];
                break;
            }
        }

        if (left > 0 && right > 0) {
            out[i] = 0.5f * (left + right);
        }
    }
}


void detectMazeFeature() {
    
    const int FRONT_START = 310;
    const int FRONT_END = 390;
    const int LEFT_START = 140;
    const int LEFT_END = 233;
    const int RIGHT_START = 505;
    const int RIGHT_END = 580;
    
    float frontAvg = getAvgDistRange(FRONT_START, FRONT_END);
    float leftAvg = getAvgDistRange(LEFT_START, LEFT_END);
    float rightAvg = getAvgDistRange(RIGHT_START, RIGHT_END);
    
    float frontMin = getMinDistRange(FRONT_START, FRONT_END);
    float leftMin = getMinDistRange(LEFT_START, LEFT_END);
    float rightMin = getMinDistRange(RIGHT_START, RIGHT_END);

    bool openFront = (frontMin > threshold_front || frontMin == 0);
    bool openRight = (rightMin > threshhold_sides || rightMin == 0);
    bool openLeft  = (leftMin > threshhold_sides || leftMin == 0);

    p1.frontOpen = openFront;
    p1.rightOpen = openRight;
    p1.leftOpen = openLeft;

    Serial.printf("FRONT: avg=%.1f min=%.1f | LEFT: avg=%.1f min=%.1f | RIGHT: avg=%.1f min=%.1f\n", 
                  frontAvg, frontMin, leftAvg, leftMin, rightAvg, rightMin);


    if (!openFront && !openLeft && !openRight) { 
        Serial.println("➡ DEAD END - Need to turn around!"); 
        return; 
    }
    if (openFront && !openLeft && !openRight) { 
        Serial.println("➡ CORRIDOR (straight only)"); 
        return; 
    }
    if (!openFront && openLeft && !openRight) { 
        Serial.println("➡ LEFT TURN ONLY"); 
        return; 
    }
    if (!openFront && !openLeft && openRight) { 
        Serial.println("➡ RIGHT TURN ONLY"); 
        return; 
    }
    if (!openFront && openLeft && openRight) { 
        Serial.println("➡ T-JUNCTION (left + right available)"); 
        return; 
    }
    if (openFront && openLeft && openRight) { 
        Serial.println("➡ FULL JUNCTION (+ intersection)"); 
        return; 
    }
    if (openFront && openLeft && !openRight) { 
        Serial.println("➡ FRONT + LEFT available"); 
        return; 
    }
    if (openFront && openRight && !openLeft) { 
        Serial.println("➡ FRONT + RIGHT available"); 
        return; 
    }
}


void drawLidarMap() {
    const int centerX = 160;
    const int centerY = 120;
    const float scale = 0.1;

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.fillCircle(centerX, centerY, 3, RED);

    for (int i = 0; i < 720; i++) {
        float dist = lidar.tmpmap.mapdata[i];
        if (dist <= 0) continue;

        float relativeDeg = (i - FRONT_ANGLE) * 0.5f;
        float angleRad = (relativeDeg - 90.0f) * DEG_TO_RAD;

        int x = centerX + dist * scale * cos(angleRad);
        int y = centerY + dist * scale * sin(angleRad);

        if (x >= 0 && x < 320 && y >= 0 && y < 240)
            M5.Lcd.drawPixel(x, y, WHITE);
    }
}

// TODO tunning
int turnDuration90Right = 2200;
int turnDuration90Left = 2200;
int turnDuration180 = 4300;

void calibrateTurn() {
    Serial.println("\n=== TURN CALIBRATION MODE ===");
    Serial.println("Commands:");
    Serial.println("  1 = Test RIGHT 90° turn");
    Serial.println("  2 = Test LEFT 90° turn");
    Serial.println("  3 = Test 180° turn");
    Serial.println("  + = Increase duration by 100ms");
    Serial.println("  - = Decrease duration by 100ms");
    Serial.println("  s = Save and show current values");
    Serial.println("  q = Quit calibration");
    Serial.println("==============================\n");
    
    int currentDuration = turnDuration90Right;
    char lastTest = '1';
    
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            
            if (c == 'q' || c == 'Q') {
                Serial.println("Exiting calibration mode");
                break;
            }
            
            if (c == '1') {
                lastTest = '1';
                currentDuration = turnDuration90Right;
                Serial.printf("Testing RIGHT 90° turn (duration=%dms)\n", currentDuration);
                sendControlCommand(1, 0, 0);
                delay(currentDuration);
                sendControlCommand(0, 0, 0);
            }
            else if (c == '2') {
                lastTest = '2';
                currentDuration = turnDuration90Left;
                Serial.printf("Testing LEFT 90° turn (duration=%dms)\n", currentDuration);
                sendControlCommand(-1, 0, 0);
                delay(currentDuration);
                sendControlCommand(0, 0, 0);
            }
            else if (c == '3') {
                lastTest = '3';
                currentDuration = turnDuration180;
                Serial.printf("Testing 180° turn (duration=%dms)\n", currentDuration);
                sendControlCommand(1, 0, 0);
                delay(currentDuration);
                sendControlCommand(0, 0, 0);
            }
            else if (c == '+') {
                currentDuration += 100;
                Serial.printf("Increased to %dms\n", currentDuration);
                
                if (lastTest == '1') turnDuration90Right = currentDuration;
                else if (lastTest == '2') turnDuration90Left = currentDuration;
                else if (lastTest == '3') turnDuration180 = currentDuration;
            }
            else if (c == '-') {
                currentDuration -= 100;
                if (currentDuration < 0) currentDuration = 0;
                Serial.printf("Decreased to %dms\n", currentDuration);
                
                if (lastTest == '1') turnDuration90Right = currentDuration;
                else if (lastTest == '2') turnDuration90Left = currentDuration;
                else if (lastTest == '3') turnDuration180 = currentDuration;
            }
            else if (c == 's' || c == 'S') {
                Serial.println("\n=== CURRENT CALIBRATION VALUES ===");
                Serial.printf("Right 90°: %dms\n", turnDuration90Right);
                Serial.printf("Left 90°:  %dms\n", turnDuration90Left);
                Serial.printf("180°:      %dms\n", turnDuration180);
                Serial.println("===================================\n");
            }
        }
        delay(10);
    }
}

bool detectReflectiveExit() {
    const float REFLECTIVE_THRESHOLD = 4000.0;
    const int MIN_CONSECUTIVE = 15;
    
    int reflectiveCount = 0;
    
    for (int i = 310; i <= 402; i++) {
        if (lidar.tmpmap.mapdata[i] > REFLECTIVE_THRESHOLD) {
            reflectiveCount++;
        }
    }
    
    if (reflectiveCount >= MIN_CONSECUTIVE) {
        Serial.printf("✨ REFLECTIVE EXIT DETECTED! (%d readings > %.0fmm)\n", 
                      reflectiveCount, REFLECTIVE_THRESHOLD);
        return true;
    }
    return false;
}

bool checkForMazeExit() {
    static int consecutiveDetections = 0;
    const int REQUIRED_DETECTIONS = 2;
    
    bool exitDetected = detectReflectiveExit();
    
    if (exitDetected) {
        consecutiveDetections++;
        if (consecutiveDetections >= REQUIRED_DETECTIONS) {
            Serial.println("\nMAZE EXIT CONFIRMED! \n");
            return true;
        }
    } else {
        consecutiveDetections = 0;
    }
    
    return false;
}

void celebrateMazeSolved() {
    sendControlCommand(0, 0, 0);
    
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(30, 80);
    M5.Lcd.println("MAZE SOLVED!");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(20, 140);
    M5.Lcd.println("Press M to restart");
    
    for (int i = 0; i < 5; i++) {
        M5.Lcd.fillScreen(GREEN);
        delay(200);
        M5.Lcd.fillScreen(BLACK);
        delay(200);
    }
    
    M5.Lcd.fillScreen(GREEN);
    M5.Lcd.setTextColor(BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(30, 100);
    M5.Lcd.println("WINNER!");
}


void mazeDrive() {

    if (checkForMazeExit()) {
        mazeSolved = true;
        celebrateMazeSolved();
        mazeMode = false;
        return;
    }

    isNewPath(p1.rightOpen, p1.frontOpen, p1.leftOpen);

    const int speed = 1;

    if (p1.rightOpen) {
        Serial.println("Turning RIGHT");
        Serial.println("----------------------------------------------------");

        sendControlCommand(0, speed, 0);
        delay(700);
        sendControlCommand(speed, 0, 0);
        delay(turnDuration90Right);
        sendControlCommand(0, 0, 0);

        updateHeading(90);

        uTurnCooldown = millis() + 2000;

        return;
    }

    if (p1.frontOpen) {
        Serial.println("Going FORWARD");
        Serial.println("----------------------------------------------------");

        sendControlCommand(0, speed, 0);
        return;
    }

    if (p1.leftOpen) {
        Serial.println("Turning LEFT");
        Serial.println("----------------------------------------------------");

        sendControlCommand(0, speed, 0);
        delay(600);
        sendControlCommand(-speed, 0, 0);
        delay(turnDuration90Left);
        sendControlCommand(0, 0, 0);

        updateHeading(-90);
        return;
    }

    Serial.println("DEAD END → U-TURN");

    Serial.println("----------------------------------------------------");

    sendControlCommand(speed, 0, 0);
    delay(turnDuration180);
    sendControlCommand(0, 0, 0);

    updateHeading(180);

    uTurnCooldown = millis() + 2000;

    return;
}


static void mazeTask(void *arg) {
    static float scanSnap[720];

    while (1) {
        if (mazeMode && !mazeSolved) {

            // Snapshot scan safely
            portENTER_CRITICAL(&lidarMux);
            for (int i = 0; i < 720; i++) scanSnap[i] = lidar.tmpmap.mapdata[i];
            portEXIT_CRITICAL(&lidarMux);

            // Update occupancy grid map (log-odds)
            updateMapFromScan(scanSnap);

            // Your existing pipeline
            drawLidarMap();
            detectMazeFeature();
            mazeDrive();
        }
        delay(150);
    }
}


static void lidarTask(void *arg) {
    while (1) {
        while (Serial1.available()) {
            lidar.lidar_data_deal(Serial1.read());
        }

        cleanLidarData(lidar.tmpmap.mapdata, cleanedMap);

        portENTER_CRITICAL(&lidarMux);
        for (int i = 0; i < 720; i++) {
            lidar.tmpmap.mapdata[i] = cleanedMap[i];
        }
        portEXIT_CRITICAL(&lidarMux);

        delay(5);
    }
}


void debugPrintLidarAngles() {
    Serial.println("\n--- LIDAR SCAN ---");
    for (int i = 0; i < 720; i++) {
        //if (lidar.tmpmap.mapdata[i] > 0) {
            Serial.printf("Index %3d  Angle %6.2f°  Dist %.1f mm\n", i, i*0.5, lidar.tmpmap.mapdata[i]);
        //}
    }
    Serial.println("--- END SCAN ---\n");
}


void processKeyboardControl() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == 'm' || c == 'M') {
            mazeMode = !mazeMode;
            mazeSolved = false;
            Serial.println(mazeMode ? "Maze Mode ENABLED" : "Maze Mode DISABLED");

            M5.Lcd.clear();
            String msg = mazeMode ? "Maze Mode ENABLED" : "Maze Mode DISABLED";
            int16_t x = (M5.Lcd.width() - M5.Lcd.textWidth(msg)) / 2;
            int16_t y = (M5.Lcd.height() - M5.Lcd.fontHeight()) / 2;
            M5.Lcd.setCursor(x, y); M5.Lcd.print(msg);

            if (!mazeMode) sendControlCommand(0, 0, 0);
            return;
        }

        if (c == 'c' || c == 'C') {
            Serial.println("Entering calibration mode...");
            sendControlCommand(0, 0, 0);
            calibrateTurn();
            return;
        }

        if (c == 'p' || c == 'P') { printAllDistances(); return; }

        if (mazeMode) continue;

        int8_t X = 0, Y = 0; const int speed = 1;
        switch(c) {
            case 'w': case 'W': X=0; Y=speed; break;
            case 's': case 'S': X=0; Y=-speed; break;
            case 'a': case 'A': X=-speed; Y=0; break;
            case 'd': case 'D': X=speed; Y=0; break;
            case 'x': case 'X': X=0; Y=0; break;
            default: continue;
        }
        Serial.printf("KB cmd: %c -> X=%d Y=%d\n", c, X, Y);
        sendControlCommand(X, Y, 0);
    }
}


void setup() {
    Serial.begin(115200);
    M5.begin(true, false, true);

    Serial1.begin(115200, SERIAL_8N1, 16, 17);
    Serial2.begin(115200);

    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(3, 10);
    M5.Lcd.println("X2 LidarBot");
    M5.Lcd.println("Ready");
    LO_OCC_DELTA  = probToLogOddsDelta(P_OCC);
    LO_FREE_DELTA = probToLogOddsDelta(P_FREE);

    for (int y = 0; y < MAP_H; y++)
    for (int x = 0; x < MAP_W; x++)
        logOdds[y][x] = 0; // prior p=0.5 => l0=0

    robot_x_mm = 0.0f;
    robot_y_mm = 0.0f;
    lastCmdTime = 0;
    lastCmdY = 0;

    xTaskCreatePinnedToCore(lidarTask, "lidar_task", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(mazeTask, "maze_task", 8192, NULL, 1, NULL, 0);
}

unsigned long lastDebug = 0;
void loop() {
    M5.update();
    processKeyboardControl();

    //  if (millis() - lastDebug > 1000) {
    //      debugPrintLidarAngles();
    //       lastDebug = millis();
    //   }

    delay(10);
}