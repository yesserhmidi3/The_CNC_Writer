#include <math.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <Arduino.h>
#include <ESP32Servo.h>

const int SERVO_R_PIN = 17;
const int SERVO_L_PIN = 18;
const int SERVO_PEN_PIN = 16;

const float LINK_L1 = 60.0f;
const float LINK_L2 = 85.0f;

int currentRPos = 90;
int currentLPos = 90;
const int SERVO_SPEED = 2;
const int SERVO_DELAY = 20;

Servo servoR;
Servo servoL;
Servo servoPen;

void moveServosSlowly(int targetR, int targetL, int SERVO_SPEED, int SERVO_DELAY)
{
    currentRPos = servoR.read();
    currentLPos = servoL.read();
    while (currentRPos != targetR || currentLPos != targetL)
    {
        if (currentRPos < targetR)
        {
            currentRPos = min(currentRPos + SERVO_SPEED, targetR);
        }
        else if (currentRPos > targetR)
        {
            currentRPos = max(currentRPos - SERVO_SPEED, targetR);
        }

        if (currentLPos < targetL)
        {
            currentLPos = min(currentLPos + SERVO_SPEED, targetL);
        }
        else if (currentLPos > targetL)
        {
            currentLPos = max(currentLPos - SERVO_SPEED, targetL);
        }
        servoR.write(currentRPos);
        servoL.write(currentLPos);
        delay(SERVO_DELAY);
    }
}

void inverseKinematics(float *angleR, float *angleL, float x, float y, float L1, float L2)
{
    float distR = sqrtf((x - 15) * (x - 15) + y * y);
    float distL = sqrtf((x + 15) * (x + 15) + y * y);
    float angle1r = acos((distR * distR + L1 * L1 - L2 * L2) / (2 * L1 * distR)) * (180 / M_PI);
    float angle2r = acos((distR * distR + 30 * 30 - distL * distL) / (2 * distR * 30)) * (180 / M_PI);
    *angleR = angle1r - (90 - angle2r);

    float angle1l = acos((distL * distL + L1 * L1 - L2 * L2) / (2 * L1 * distL)) * (180 / M_PI);
    float angle2l = acos((distL * distL + 30 * 30 - distR * distR) / (2 * distL * 30)) * (180 / M_PI);
    *angleL = (angle1l - (90 - angle2l));
}

void setup()
{
    Serial.begin(115200);
    servoR.attach(SERVO_R_PIN);
    servoL.attach(SERVO_L_PIN);
    servoPen.attach(SERVO_PEN_PIN);

    servoR.write(90);
    servoL.write(90);
    servoPen.write(90);

    Serial.println("Send: pen_angle x y");
}

void loop()
{
    if (Serial.available() > 0)
    {
        int penAngle = Serial.parseInt();
        float x = Serial.parseFloat();
        float y = Serial.parseFloat();

        Serial.readStringUntil('\n');

        penAngle = constrain(penAngle, 0, 180);
        servoPen.write(penAngle);

        delay(50);

        float angleR = 0.0f;
        float angleL = 0.0f;

        inverseKinematics(&angleR, &angleL, x, y, LINK_L1, LINK_L2);

        if (isnan(angleR) || isnan(angleL))
        {
            Serial.println("Unreachable point or invalid input");
            return;
        }

        int rCmd = 90 - (int)roundf(angleR);
        int lCmd = 90 + (int)roundf(angleL);

        rCmd = constrain(rCmd, 0, 180);
        lCmd = constrain(lCmd, 0, 180);

        moveServosSlowly(rCmd, lCmd, SERVO_SPEED, SERVO_DELAY);

        Serial.print("Pen: ");
        Serial.print(penAngle);
        Serial.print(" deg, x: ");
        Serial.print(x);
        Serial.print(" y: ");
        Serial.print(y);
        Serial.print(" -> R: ");
        Serial.print(rCmd);
        Serial.print(" deg, L: ");
        Serial.print(lCmd);
        Serial.println(" deg");
    }
}