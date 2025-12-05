#include <M5Stack.h>
#include "X2driver.h"
#include "lidarcar.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define TFT_GREY 0x5AEB
#define FRONT_ANGLE 360  // index for front of robot in lidar map (0–719)

bool mazeMode = false;

X2 lidar;
LidarCar lidarcar;

// ==================== Motor Helper ====================
void sendControlCommand(int8_t X, int8_t Y, uint8_t A) {
    lidarcar.ControlWheel(X, Y, A);
}

// ==================== Maze Driving ====================
void mazeDrive() {
    float frontDist = 0;
    int count = 0;

    for (int i = -2; i <= 2; i++) {
        int idx = (FRONT_ANGLE + i + 720) % 720;
        if (lidar.tmpmap.mapdata[idx] > 0) {
            frontDist += lidar.tmpmap.mapdata[idx];
            count++;
        }
    }

    if (count > 0) frontDist /= count;

    Serial.printf("FrontDist=%.1f, count=%d\n", frontDist, count);

    int8_t X = 0, Y = 0; uint8_t A = 0;
    const int speed = 2;  // small number, works with ControlWheel

    if (count == 0 || frontDist > 150) {
        Serial.println("Path is clear");
        Y = speed;
    } else {
        Serial.println("Obstacle detected");
        X = speed;
        Y = 0;
    }

    sendControlCommand(X, Y, 0);
}

// ==================== Tasks ====================
static void mazeTask(void *arg) {
    while (1) {
        if (mazeMode) {
            mazeDrive();
        }
        delay(50);
    }
}

static void lidarTask(void *arg) {
    while (1) {
        while (Serial1.available()) {
            lidar.lidar_data_deal(Serial1.read());
        }
        delay(5);
    }
}

// ==================== Setup ====================
void setup() {
    Serial.begin(115200);
    M5.begin(true, false, true);
    Serial1.begin(115200, SERIAL_8N1, 16, 17);
    Serial2.begin(115200);

    //lidarcar.begin();

    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(3, 10);
    M5.Lcd.println("X2 LidarBot");

    xTaskCreatePinnedToCore(lidarTask, "lidar_task", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(mazeTask, "maze_task", 8192, NULL, 1, NULL, 0);
}

// ==================== Keyboard ====================
void processKeyboardControl() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == 'm' || c == 'M') {
            mazeMode = !mazeMode;
            Serial.println(mazeMode ? "Maze Mode ENABLED" : "Maze Mode DISABLED");
            if (!mazeMode) sendControlCommand(0,0,0);
            return;
        }

        if (mazeMode) continue;

        int8_t X=0, Y=0; uint8_t A=0;
        const int speed=4;

        switch(c) {
            case 'w': case 'W': X=0; Y=speed; break;
            case 's': case 'S': X=0; Y=-speed; break;
            case 'a': case 'A': X=-speed; Y=0; break;
            case 'd': case 'D': X=speed; Y=0; break;
            case 'x': case 'X': X=0; Y=0; break;
            default: continue;
        }

        Serial.printf("KB cmd: %c -> X=%d Y=%d\n", c,X,Y);
        sendControlCommand(X,Y,A);
    }
}

// ==================== Loop ====================
void loop() {
    M5.update();
    processKeyboardControl();
    delay(10);
}
