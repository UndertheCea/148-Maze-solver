#include <M5Stack.h>
#include "X2driver.h"
#include "lidarcar.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define FRONT_ANGLE 360
#define TFT_GREY 0x5AEB

bool mazeMode = false;

X2 lidar;
LidarCar lidarcar;

 const float threshold = 200.0;

void sendControlCommand(int8_t X, int8_t Y, uint8_t A) {
    lidarcar.ControlWheel(X, Y, A);
}


float getAvgDistIdx(int centerIdx, int spreadDeg = 10) {
    float sum = 0;
    int count = 0;
    for (int i = -spreadDeg * 2; i <= spreadDeg * 2; i++) {
        int idx = (centerIdx + i + 720) % 720;
        float d = lidar.tmpmap.mapdata[idx];
        if (d > 0) {
            sum += d;
            count++;
        }
    }
    return (count == 0) ? 0 : (sum / count);
}


float getAvgDistDir(int dirDeg, int spreadDeg = 10) {
    int centerIdx = (FRONT_ANGLE + dirDeg * 2) % 720;
    return getAvgDistIdx(centerIdx, spreadDeg);
}


void printAllDistances() {
    Serial.println("=== LIDAR 360° DATA ===");
    for (int i = 0; i < 720; i++) {
        int angle = i / 2;
        Serial.printf("%3d° : %.1f mm\n", angle, lidar.tmpmap.mapdata[i]);
    }
    Serial.println("=================================\n");
}


void detectMazeFeature() {
    float front = getAvgDistDir(0);
    float right = getAvgDistDir(90);
    float back  = getAvgDistDir(180);
    float left  = getAvgDistDir(270);


    bool openFront = (front > threshold || front == 0);
    bool openRight = (right > threshold || right == 0);
    bool openLeft  = (left  > threshold || left  == 0);
    bool openBack  = (back  > threshold || back  == 0);

    Serial.printf("F=%.1f  R=%.1f  L=%.1f  B=%.1f\n", front, right, left, back);

    // ---- Logic ----
    if (!openFront && !openLeft && !openRight) { Serial.println("➡ DEAD END"); return; }
    if (openFront && !openLeft && !openRight) { Serial.println("➡ CORRIDOR (straight road)"); return; }
    if (!openFront && openLeft && !openRight) { Serial.println("➡ LEFT TURN ONLY"); return; }
    if (!openFront && !openLeft && openRight) { Serial.println("➡ RIGHT TURN ONLY"); return; }
    if (!openFront && openLeft && openRight) { Serial.println("➡ T-JUNCTION (left + right)"); return; }
    if (openFront && openLeft && openRight) { Serial.println("➡ FULL JUNCTION (+ intersection)"); return; }
    if (openFront && openLeft && !openRight) { Serial.println("➡ FRONT + LEFT available"); return; }
    if (openFront && openRight && !openLeft) { Serial.println("➡ FRONT + RIGHT available"); return; }
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


void mazeDrive() {
    float frontDist = getAvgDistDir(0, 2);
    Serial.printf("FrontDist=%.1f\n", frontDist);

    int8_t X = 0, Y = 0;
    const int speed = 2;

    if (frontDist == 0 || frontDist > threshold) {
        Serial.println("Path is clear → forward");
        Y = speed;
    } else {
        Serial.println("Obstacle detected → turning right");
        X = speed;
    }

    sendControlCommand(X, Y, 0);
}


static void mazeTask(void *arg) {
    while (1) {
        if (mazeMode) {
            drawLidarMap();
            detectMazeFeature();
            mazeDrive();
        }
        delay(150);
    }
}


static void lidarTask(void *arg) {
    while (1) {
        while (Serial1.available()) lidar.lidar_data_deal(Serial1.read());
        delay(5);
    }
}


void debugPrintLidarAngles() {
    Serial.println("\n--- LIDAR SCAN ---");
    for (int i = 0; i < 720; i++) {
        if (lidar.tmpmap.mapdata[i] > 0) {
            Serial.printf("Index %3d  Angle %6.2f°  Dist %.1f mm\n", i, i*0.5, lidar.tmpmap.mapdata[i]);
        }
    }
    Serial.println("--- END SCAN ---\n");
}


void processKeyboardControl() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == 'm' || c == 'M') {
            mazeMode = !mazeMode;
            Serial.println(mazeMode ? "Maze Mode ENABLED" : "Maze Mode DISABLED");

            M5.Lcd.clear();
            String msg = mazeMode ? "Maze Mode ENABLED" : "Maze Mode DISABLED";
            int16_t x = (M5.Lcd.width() - M5.Lcd.textWidth(msg)) / 2;
            int16_t y = (M5.Lcd.height() - M5.Lcd.fontHeight()) / 2;
            M5.Lcd.setCursor(x, y); M5.Lcd.print(msg);

            if (!mazeMode) sendControlCommand(0, 0, 0);
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

    xTaskCreatePinnedToCore(lidarTask, "lidar_task", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(mazeTask, "maze_task", 8192, NULL, 1, NULL, 0);
}

unsigned long lastDebug = 0;
void loop() {
    M5.update();
    processKeyboardControl();

    // if (millis() - lastDebug > 1000) {
    //     debugPrintLidarAngles();
    //     lastDebug = millis();
    // }

    delay(10);
}
