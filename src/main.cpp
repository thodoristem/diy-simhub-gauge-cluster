#include <Arduino.h>
#include <AccelStepper.h>
#include <LedControl.h>
#include <LedMatrixGearMap.h>

#define MOTOR_STEPS 200
#define MOTOR_SPEED 400
#define CAL_MOTOR_SPEED 200
#define MOTOR_MAX_SPEED 1000
#define MOTOR_MICROSTEPS 32

#define MAX_RPM_RANGE 11000
#define MAX_SPEED_RANGE 320.0f

#define MSG_LENGTH 25
#define MSG_END_CHAR '|'
#define RPM_DELIMITER "R"
#define SPEED_DELIMITER "S"
#define GEAR_DELIMITER "G"
#define SHIFT_LIGHT_DELIMITER "L"
#define DIMMER_DELIMITER "D"

#define SHIFT_LIGHT_GREEN 6
#define SHIFT_LIGHT_YELLOW 7
#define SHIFT_LIGHT_RED 8

#define BACKLIGHT_PWM 9

#define GEAR_8x8_LED_DIN 12
#define GEAR_8x8_LED_CS 11
#define GEAR_8x8_LED_CLK 10

LedControl gearLC = LedControl(GEAR_8x8_LED_DIN, GEAR_8x8_LED_CLK, GEAR_8x8_LED_CS, 1);

const float STEPS_PER_DEG = (MOTOR_STEPS * MOTOR_MICROSTEPS) / 360.0f;

#define RPM_GAUGE_STOP A6
#define RPM_GAUGE_STEP 3
#define RPM_GAUGE_DIR 2
#define RPM_GAUGE_MIN_OFFSET -4 // Offset of the rpm zero mark from the min stop
#define RPM_GAUGE_MAX_OFFSET 6  // Offset of max speed mark from the max stop

AccelStepper rpmGaugeStepper(AccelStepper::DRIVER, RPM_GAUGE_STEP, RPM_GAUGE_DIR);

#define SPEED_GAUGE_STOP A7
#define SPEED_GAUGE_STEP 5
#define SPEED_GAUGE_DIR 4
#define SPEED_GAUGE_MIN_OFFSET -4 // Offset of the speed zero mark from the min stop
#define SPEED_GAUGE_MAX_OFFSET 6  // Offset of max speed mark from the max stop

AccelStepper speedGaugeStepper(AccelStepper::DRIVER, SPEED_GAUGE_STEP, SPEED_GAUGE_DIR);

long rpmValue = 0;
long rpmValuePrev = 0;
long rpmRotationSteps = 0;
long rpmMaxSteps = 360.0f * STEPS_PER_DEG;

void rpmGaugeLoop()
{
    if (rpmValue != rpmValuePrev)
    {
        rpmValuePrev = rpmValue;
        if (rpmValue > MAX_RPM_RANGE)
        {
            rpmValue = MAX_RPM_RANGE;
        }
        if (rpmValue < 0)
        {
            rpmValue = 0;
        }
        rpmRotationSteps = map(rpmValue, 0, MAX_RPM_RANGE, 0, rpmMaxSteps);
        rpmGaugeStepper.moveTo(rpmRotationSteps);
        rpmGaugeStepper.setSpeed(MOTOR_SPEED * MOTOR_MICROSTEPS);
    }
    rpmGaugeStepper.runSpeedToPosition();
}

float speedValue = 0.0f;
float speedValuePrev = 0.0f;
float speedRotationSteps = 0.0;
float speedMaxSteps = 360.0f * STEPS_PER_DEG;

void speedGaugeLoop()
{
    if (speedValue != speedValuePrev)
    {
        speedValuePrev = speedValue;
        if (speedValue > MAX_SPEED_RANGE)
        {
            speedValue = MAX_SPEED_RANGE;
        }
        if (speedValue < 0)
        {
            speedValue = 0.0f;
        }
        speedRotationSteps = map(speedValue * 100.0f, 0, MAX_SPEED_RANGE * 100.0f, 0, speedMaxSteps);
        speedGaugeStepper.moveTo(speedRotationSteps);
        speedGaugeStepper.setSpeed(MOTOR_SPEED * MOTOR_MICROSTEPS);
    }
    speedGaugeStepper.runSpeedToPosition();
}

short gearValue = 0;
short gearValuePrev = 0;

void ledMatrixLoop(bool init)
{
    if (init || (gearValue != gearValuePrev && (gearValue + 2) <= GEAR_MAX))
    {
        gearValuePrev = gearValue;
        for (int i = 7; i >= 0; i--)
        {
            gearLC.setColumn(0, 7 - i, GEARS_MAP[gearValue + 1][i]);
        }
    }
}

float shiftLightValue = 0;
float shiftLightValuePrev = 0;

void shifterLighLoop()
{
    if (shiftLightValue != shiftLightValuePrev)
    {
        shiftLightValuePrev = shiftLightValue;
        if (shiftLightValue >= 3)
        {
            digitalWrite(SHIFT_LIGHT_GREEN, HIGH);
            digitalWrite(SHIFT_LIGHT_YELLOW, HIGH);
            digitalWrite(SHIFT_LIGHT_RED, HIGH);
        }
        else if (shiftLightValue >= 2)
        {
            digitalWrite(SHIFT_LIGHT_GREEN, HIGH);
            digitalWrite(SHIFT_LIGHT_YELLOW, HIGH);
            digitalWrite(SHIFT_LIGHT_RED, LOW);
        }
        else if (shiftLightValue >= 1)
        {
            digitalWrite(SHIFT_LIGHT_GREEN, HIGH);
            digitalWrite(SHIFT_LIGHT_YELLOW, LOW);
            digitalWrite(SHIFT_LIGHT_RED, LOW);
        }
        else
        {
            digitalWrite(SHIFT_LIGHT_GREEN, LOW);
            digitalWrite(SHIFT_LIGHT_YELLOW, LOW);
            digitalWrite(SHIFT_LIGHT_RED, LOW);
        }
    }
}

short dimmerValue = 0;
short dimmerValuePrev = 0;

void dimmerLoop()
{
    if (dimmerValue != dimmerValuePrev)
    {
        dimmerValuePrev = dimmerValue;
        gearLC.setIntensity(0, map(dimmerValue, 0, 100, 0, 15));
        analogWrite(BACKLIGHT_PWM, map(dimmerValue, 0, 100, 0, 30));
    }
}

bool isLimitSetupDone = true;

void stepperSetup()
{
    rpmGaugeStepper.setMaxSpeed(MOTOR_MAX_SPEED * MOTOR_MICROSTEPS);
    speedGaugeStepper.setMaxSpeed(MOTOR_MAX_SPEED * MOTOR_MICROSTEPS);

    /* RPM GAUGE LIMIT SETUP BEGIN */
    while (analogRead(RPM_GAUGE_STOP) != LOW)
    {
        rpmGaugeStepper.moveTo(-400 * STEPS_PER_DEG);
        rpmGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        rpmGaugeStepper.runSpeedToPosition();
    }
    rpmGaugeStepper.stop();
    rpmGaugeStepper.setCurrentPosition(RPM_GAUGE_MIN_OFFSET * STEPS_PER_DEG);
    while (rpmGaugeStepper.currentPosition() < 0)
    {
        rpmGaugeStepper.moveTo(0);
        rpmGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        rpmGaugeStepper.runSpeedToPosition();
    }
    while (analogRead(RPM_GAUGE_STOP) != LOW)
    {
        rpmGaugeStepper.moveTo(400 * STEPS_PER_DEG);
        rpmGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        rpmGaugeStepper.runSpeedToPosition();
    }
    rpmGaugeStepper.stop();
    rpmMaxSteps = rpmGaugeStepper.currentPosition() - (RPM_GAUGE_MAX_OFFSET * STEPS_PER_DEG);
    while (rpmGaugeStepper.currentPosition() > 0)
    {
        rpmGaugeStepper.moveTo(0);
        rpmGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        rpmGaugeStepper.runSpeedToPosition();
    }
    /* RPM GAUGE LIMIT SETUP END */

    /* SPEED GAUGE LIMIT SETUP BEGIN */
    while (analogRead(SPEED_GAUGE_STOP) != LOW)
    {
        speedGaugeStepper.moveTo(-400 * STEPS_PER_DEG);
        speedGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        speedGaugeStepper.runSpeedToPosition();
    }
    speedGaugeStepper.stop();
    speedGaugeStepper.setCurrentPosition(SPEED_GAUGE_MIN_OFFSET * STEPS_PER_DEG);
    while (speedGaugeStepper.currentPosition() < 0)
    {
        speedGaugeStepper.moveTo(0);
        speedGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        speedGaugeStepper.runSpeedToPosition();
    }
    while (analogRead(SPEED_GAUGE_STOP) != LOW)
    {
        speedGaugeStepper.moveTo(400 * STEPS_PER_DEG);
        speedGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        speedGaugeStepper.runSpeedToPosition();
    }
    speedGaugeStepper.stop();
    speedMaxSteps = speedGaugeStepper.currentPosition() - (SPEED_GAUGE_MAX_OFFSET * STEPS_PER_DEG);
    while (speedGaugeStepper.currentPosition() > 0)
    {
        speedGaugeStepper.moveTo(0);
        speedGaugeStepper.setSpeed(CAL_MOTOR_SPEED * MOTOR_MICROSTEPS);
        speedGaugeStepper.runSpeedToPosition();
    }
    /* SPEED GAUGE LIMIT SETUP END */

    isLimitSetupDone = true;
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        delay(1000);
    }

    pinMode(RPM_GAUGE_STOP, INPUT);
    pinMode(SPEED_GAUGE_STOP, INPUT);

    pinMode(SHIFT_LIGHT_GREEN, OUTPUT);
    pinMode(SHIFT_LIGHT_YELLOW, OUTPUT);
    pinMode(SHIFT_LIGHT_RED, OUTPUT);
    pinMode(BACKLIGHT_PWM, OUTPUT);

    gearLC.shutdown(0, false);
    gearLC.setIntensity(0, 0);
    gearLC.clearDisplay(0);

    ledMatrixLoop(true);

    delay(1000);

    stepperSetup();

    delay(1000);
}

byte index = 0;
char msg[MSG_LENGTH];
bool newMsg = false;
char rChar;

void serialReadLoop()
{
    // Reading Serial Message
    if (Serial.available() > 0 && !newMsg)
    {
        rChar = Serial.read();
        if (rChar != MSG_END_CHAR)
        {
            msg[index] = rChar;
            index++;
            if (index >= MSG_LENGTH)
            {
                index = MSG_LENGTH - 1;
            }
        }
        else
        {
            msg[index] = '\0';
            index = 0;
            newMsg = true;
        }
    }

    // Executing Serial Message
    if (newMsg)
    {
        char *temp = msg;

        // Serial.println(msg);

        rpmValue = atoi(strtok(temp, RPM_DELIMITER));
        speedValue = atof(strtok(NULL, SPEED_DELIMITER));
        gearValue = atoi(strtok(NULL, GEAR_DELIMITER));
        shiftLightValue = atof(strtok(NULL, SHIFT_LIGHT_DELIMITER));
        dimmerValue = atoi(strtok(NULL, DIMMER_DELIMITER));

        newMsg = false;
    }
}

void loop()
{
    if (isLimitSetupDone)
    {
        serialReadLoop();

        rpmGaugeLoop();
        speedGaugeLoop();
        ledMatrixLoop(false);
        shifterLighLoop();
        dimmerLoop();
    }
}