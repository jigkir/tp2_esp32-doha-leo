#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DebouncedInput.h"

// =====================================================
// Pins
// =====================================================
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr int MOTOR_INPUT3_PIN = 14;
constexpr int MOTOR_INPUT4_PIN = 13;

constexpr int SERVO_PIN = 26;

constexpr int JOYSTICK_PUSH_BUTTON_PIN = 4;
constexpr int PASSIVE_BUZZER_PIN = 27;

constexpr int TURN_SIGNAL_SWITCH_PIN = 19;

constexpr int TURN_SIGNAL_LED_PIN = 32; // Yellow LED
constexpr int HEADLIGHTS_LED_PIN = 33;  // Red LED

/*
    Debounced inputs are created in setup(), so we only keep pointers here.

    - The DebouncedInput constructor calls pinMode() and attachInterrupt().
    A global object would be constructed before setup(), while the ESP32 system
    (GPIO interrupt service) is not ready yet: this caused a crash and reboot loop.

    - The class deletes its copy and move operations, so an object cannot be
    declared here and assigned later: a pointer lets us create it at the right time.

    - These objects live for the whole program, so they are never deleted.
*/
DebouncedInput *joystickPushButton = nullptr;
DebouncedInput *turnSignalSwitch = nullptr;

// =====================================================
// I2C devices
// =====================================================
constexpr uint8_t ADS7830_ADDRESS = 0x4B;
constexpr uint8_t LCD_ADDRESS = 0x27;

constexpr uint32_t I2C_CLOCK_HZ = 100000; // Standard mode: max speed of the LCD module

constexpr int ADC_MAX_READING = 255; // ADS7830: 8-bit readings (0..255)
constexpr int ADC_MID_READING = 128; // Default joystick center (middle of the range)
constexpr uint8_t ADS7830_CHANNEL_COUNT = 8;

constexpr uint8_t JOYSTICK_VRY_ADC_INPUT = 0;
constexpr uint8_t JOYSTICK_VRX_ADC_INPUT = 1;
constexpr uint8_t PHOTORESISTOR_ADC_INPUT = 4;

// =========================================================================
// Vehicle specification: speeds in units, accelerations in units per second
// =========================================================================
constexpr float MAX_BACKWARD_SPEED = -50.0f;
constexpr float MAX_FORWARD_SPEED = 100.0f;
constexpr float MAX_BACKWARD_ACCELERATION = 13.0f;
constexpr float MAX_FORWARD_ACCELERATION = 15.0f;
constexpr float NATURAL_DECELERATION = 1.0f; // always applied

// =====================================================
// Timing
// =====================================================
constexpr unsigned long SPEED_UPDATE_PERIOD_MS = 100; // 10 updates per second
constexpr unsigned long SERVO_UPDATE_PERIOD_MS = 250; // 4 updates per second
constexpr unsigned long HEADLIGHTS_UPDATE_PERIOD_MS = 200;
constexpr unsigned long LCD_REFRESH_PERIOD_MS = 200;
constexpr unsigned long DEBUG_PRINT_PERIOD_MS = 500;

constexpr unsigned long TURN_SIGNAL_OFF_MS = 250;
constexpr unsigned long TURN_SIGNAL_ON_MS = 500;

// =====================================================
// Servo
// =====================================================
constexpr int SERVO_PWM_FREQUENCY_HZ = 50; // Standard servo signal: one pulse every 20 ms
constexpr int SERVO_MIN_PULSE_US = 500;    // Pulse width for 0 degrees
constexpr int SERVO_MAX_PULSE_US = 2400;   // Pulse width for 180 degrees
constexpr int SERVO_MIN_ANGLE = 0;         // Joystick fully left
constexpr int SERVO_CENTER_ANGLE = 90;     // Wheels straight
constexpr int SERVO_MAX_ANGLE = 180;       // Joystick fully right

// =====================================================
// Calibration
// =====================================================
constexpr int JOYSTICK_DEAD_ZONE = 20;    // Ignore small movements around the center
constexpr bool INVERT_JOYSTICK_Y = true;  // Inverts the forward/backward axis
constexpr bool INVERT_JOYSTICK_X = false; // Inverts the steering axis

constexpr int HEADLIGHTS_ON_THRESHOLD = 200;
constexpr int HEADLIGHTS_OFF_THRESHOLD = 170;

constexpr int TURN_SIGNAL_SWITCH_LEVEL = HIGH;

constexpr int HORN_FREQUENCY_HZ = 440;

constexpr int MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr int MOTOR_MIN_DUTY_CYCLE = 70;  // Minimum duty (0..255) to keep the motor turning
constexpr float MOTOR_START_SPEED = 5.0f; // Below this speed the motor is off

constexpr int MOTOR_PWM_RESOLUTION_BITS = 8;
constexpr int MOTOR_MAX_DUTY_CYCLE = 255; // 8-bit PWM: 255 = 100%
constexpr int BUZZER_PWM_RESOLUTION_BITS = 10;

constexpr int DEBOUNCE_DELAY_MS = 20;            // Ignore button/switch bounces shorter than this
constexpr int JOYSTICK_CALIBRATION_SAMPLES = 16; // Readings averaged to find the joystick center

// =====================================================
// LCD custom characters
// =====================================================
constexpr uint8_t LCD_CHAR_ARROW_FORWARD = 0;
constexpr uint8_t LCD_CHAR_ARROW_BACKWARD = 1;
constexpr uint8_t LCD_CHAR_BULB = 2;

constexpr int LCD_COLUMNS = 16;
constexpr int LCD_ROWS = 2;
constexpr int LCD_ICON_COLUMN = LCD_COLUMNS - 1; // Last column: arrow and bulb icons

uint8_t arrowForwardIcon[8] = {
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000};

uint8_t arrowBackwardIcon[8] = {
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00000};

uint8_t bulbIcon[8] = {
    0b01110,
    0b10001,
    0b10001,
    0b10001,
    0b01010,
    0b01110,
    0b01110,
    0b00100};

// =====================================================
// Devices
// =====================================================
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
Servo servo;
ESP32PWM passiveBuzzerPwm;
ESP32PWM motorForwardPwm;
ESP32PWM motorBackwardPwm;

// ADS7830 single-ended channel commands
constexpr uint8_t ADS7830_COMMANDS[ADS7830_CHANNEL_COUNT] = {
    0x84, // Channel 0
    0xC4, // Channel 1
    0x94, // Channel 2
    0xD4, // Channel 3
    0xA4, // Channel 4
    0xE4, // Channel 5
    0xB4, // Channel 6
    0xF4  // Channel 7
};

// =====================================================
// State
// =====================================================
float vehicleSpeed = 0.0f; // -50 .. 100
int servoAngle = SERVO_CENTER_ANGLE;
int photoresistorReading = 0;
int joystickCenterX = ADC_MID_READING;
int joystickCenterY = ADC_MID_READING;
int lastJoystickRawY = ADC_MID_READING; // Last joystick Y reading (for debugging)

bool headlightsOn = false;
bool turnSignalOn = false;
bool hornOn = false;

unsigned long turnSignalStartMs = 0;
unsigned long lastSpeedUpdateMs = 0;
unsigned long lastServoUpdateMs = 0;
unsigned long lastHeadlightsUpdateMs = 0;
unsigned long lastLcdRefreshMs = 0;
unsigned long lastDebugPrintMs = 0;

// =====================================================
// ADS7830: returns 0..255, or -1 on error
// =====================================================
int readADC(uint8_t channel)
{
    if (channel >= ADS7830_CHANNEL_COUNT)
    {
      return -1;
    }

    Wire.beginTransmission(ADS7830_ADDRESS);
    Wire.write(ADS7830_COMMANDS[channel]);

    if (Wire.endTransmission() != 0)
    {
      return -1;
    }

    if (Wire.requestFrom(ADS7830_ADDRESS, (uint8_t)1) != 1 || !Wire.available())
    {
      return -1;
    }

    return Wire.read();
}

// Average several readings (used to find the joystick center at startup)
int readAverage(uint8_t channel, int samples)
{
    long sum = 0;
    int valid = 0;

    for (int i = 0; i < samples; i++)
    {
        int value = readADC(channel);
        if (value >= 0)
        {
          sum += value;
          valid++;
        }
        delay(5);
    }

    return (valid > 0) ? (int)(sum / valid) : ADC_MID_READING;
}

// Converts a raw joystick reading (0..255) into a value from -1.0 to 1.0
float normalizeJoystickAxis(int raw, int center)
{
    int delta = raw - center;

    // Inside the dead zone: joystick considered at rest
    if (abs(delta) <= JOYSTICK_DEAD_ZONE)
    {
      return 0.0f;
    }

    if (delta > 0)
    {
        float positiveTravel = (ADC_MAX_READING - center) - JOYSTICK_DEAD_ZONE;

        if (positiveTravel <= 0)
        {
          return 0.0f;
        }

        return constrain((delta - JOYSTICK_DEAD_ZONE) / positiveTravel, 0.0f, 1.0f);
    }

    float negativeTravel = center - JOYSTICK_DEAD_ZONE;

    if (negativeTravel <= 0)
    {
      return 0.0f;
    }

    return -constrain((-delta - JOYSTICK_DEAD_ZONE) / negativeTravel, 0.0f, 1.0f);
}

// =====================================================
// Motor (PWM on the active input, 0 on the other)
// =====================================================
void applyMotorSpeed(float speed)
{
    // Off below MOTOR_START_SPEED, then from MOTOR_MIN_DUTY_CYCLE up to 100% at 100 units
    float magnitude = fabsf(speed);
    int duty = 0;
    if (magnitude >= MOTOR_START_SPEED)
    {
      duty = MOTOR_MIN_DUTY_CYCLE + (int)((MOTOR_MAX_DUTY_CYCLE - MOTOR_MIN_DUTY_CYCLE) * magnitude / MAX_FORWARD_SPEED);
      duty = constrain(duty, 0, MOTOR_MAX_DUTY_CYCLE);
    }

    if (speed > 0.0f)
    {
      motorBackwardPwm.write(0);
      motorForwardPwm.write(duty);
    }
    else if (speed < 0.0f)
    {
      motorForwardPwm.write(0);
      motorBackwardPwm.write(duty);
    }
    else
    {
      motorForwardPwm.write(0);
      motorBackwardPwm.write(0);
    }
}

// =====================================================
// Speed: joystick Y controls the acceleration
// =====================================================
void updateSpeed(unsigned long now)
{
    float elapsedSeconds = (now - lastSpeedUpdateMs) / 1000.0f;
    lastSpeedUpdateMs = now;

    // On a read error, act as if the joystick were neutral
    int joystickRawY = readADC(JOYSTICK_VRY_ADC_INPUT);
    float y = 0.0f;

    if (joystickRawY >= 0)
    {
      lastJoystickRawY = joystickRawY;
      y = normalizeJoystickAxis(joystickRawY, joystickCenterY);
    }

    if (INVERT_JOYSTICK_Y)
    {
      y = -y;
    }

    // Acceleration proportional to the joystick position
    float acceleration = (y > 0.0f) ? y * MAX_FORWARD_ACCELERATION : y * MAX_BACKWARD_ACCELERATION;
    vehicleSpeed += acceleration * elapsedSeconds;

    // Natural deceleration toward 0, always applied
    float deceleration = NATURAL_DECELERATION * elapsedSeconds;

    if (vehicleSpeed > 0.0f)
    {
      vehicleSpeed = max(0.0f, vehicleSpeed - deceleration);
    }
    else if (vehicleSpeed < 0.0f)
    {
      vehicleSpeed = min(0.0f, vehicleSpeed + deceleration);
    }

    vehicleSpeed = constrain(vehicleSpeed, MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED);
    applyMotorSpeed(vehicleSpeed);
}

// =====================================================
// Steering: joystick X -> servo 0..180 (90 = neutral)
// =====================================================
void updateSteering()
{
    int joystickRawX = readADC(JOYSTICK_VRX_ADC_INPUT);

    if (joystickRawX < 0)
    {
      return; // Keep the last position on a read error
    }

    float x = normalizeJoystickAxis(joystickRawX, joystickCenterX);

    if (INVERT_JOYSTICK_X)
    {
      x = -x;
    }

    servoAngle = constrain((int)lroundf(SERVO_CENTER_ANGLE + x * (SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE)), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    servo.write(servoAngle);
}

// =====================================================
// Headlights: two thresholds to avoid useless toggling
// =====================================================
void updateHeadlights()
{
    int raw = readADC(PHOTORESISTOR_ADC_INPUT);

    if (raw < 0)
    {
      return;
    }

    photoresistorReading = raw;

    // Darker = higher reading
    if (!headlightsOn && photoresistorReading >= HEADLIGHTS_ON_THRESHOLD)
    {
      headlightsOn = true;
    }
    else if (headlightsOn && photoresistorReading <= HEADLIGHTS_OFF_THRESHOLD)
    {
      headlightsOn = false;
    }

    digitalWrite(HEADLIGHTS_LED_PIN, headlightsOn ? HIGH : LOW);
}

// =====================================================
// Turn signal: 250 ms OFF, then 500 ms ON, while the switch is on
// =====================================================
void updateTurnSignal(unsigned long now)
{
    bool enabled = (turnSignalSwitch->read() == TURN_SIGNAL_SWITCH_LEVEL);

    // Each activation starts at the beginning of the cycle (OFF phase)
    if (enabled && !turnSignalOn)
    {
      turnSignalStartMs = now;
    }

    turnSignalOn = enabled;

    if (!turnSignalOn)
    {
      digitalWrite(TURN_SIGNAL_LED_PIN, LOW);
      return;
    }

    /*
        Position in the blink cycle (0 to 749 ms):

        - (now - turnSignalStartMs): time elapsed since the turn signal was turned on.
        - % 750 (250 ms OFF + 500 ms ON): the modulo restarts at 0 every 750 ms, so the cycle repeats forever.
    */
    unsigned long phase = (now - turnSignalStartMs) % (TURN_SIGNAL_OFF_MS + TURN_SIGNAL_ON_MS);

    digitalWrite(TURN_SIGNAL_LED_PIN, (phase >= TURN_SIGNAL_OFF_MS) ? HIGH : LOW);
}

// =====================================================
// Horn: sounds while the joystick button is pressed
// =====================================================
void updateHorn()
{
    bool pressed = (joystickPushButton->read() == LOW); // LOW = pressed (pull-up)

    if (pressed && !hornOn)
    {
      passiveBuzzerPwm.writeTone(HORN_FREQUENCY_HZ);
      hornOn = true;
    }
    else if (!pressed && hornOn)
    {
      passiveBuzzerPwm.write(0);
      hornOn = false;
    }
}

// =====================================================================
// LCD
// Line 0: speed (always positive) + direction arrow (top right)
// Line 1: headlights state + bulb icon (bottom right) if turn signal on
// =====================================================================
void updateLCD()
{
    char line[LCD_COLUMNS + 1];

    snprintf(line, sizeof(line), "Speed: %5.1f   ", fabsf(vehicleSpeed));
    lcd.setCursor(0, 0);
    lcd.print(line);

    lcd.setCursor(LCD_ICON_COLUMN, 0);

    if (vehicleSpeed > 0.05f)
    {
      lcd.write(LCD_CHAR_ARROW_FORWARD);
    }
    else if (vehicleSpeed < -0.05f)
    {
      lcd.write(LCD_CHAR_ARROW_BACKWARD);
    }
    else
    {
      lcd.print(' ');
    }

    snprintf(line, sizeof(line), "Lights: %-7s", headlightsOn ? "ON" : "OFF");
    lcd.setCursor(0, 1);
    lcd.print(line);

    lcd.setCursor(LCD_ICON_COLUMN, 1);

    if (turnSignalOn)
    {
        lcd.write(LCD_CHAR_BULB);
    }
    else
    {
      lcd.print(' ');
    }
}

// =====================================================
// Serial debug output (useful for calibration)
// =====================================================
void printDebug()
{
    Serial.printf("JoystickY = %3d (center %3d) | Speed = %6.1f | Servo angle = %3d | Light = %3d | Headlights = %d | Turn signal = %d | Horn = %d\n",
                  lastJoystickRawY, joystickCenterY, vehicleSpeed, servoAngle, photoresistorReading,
                  headlightsOn, turnSignalOn, hornOn);
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("---> DRAGULA STARTING");

    // Debounced inputs (configure the pins and attach the interrupts)
    joystickPushButton = new DebouncedInput(JOYSTICK_PUSH_BUTTON_PIN, true, DEBOUNCE_DELAY_MS);
    turnSignalSwitch = new DebouncedInput(TURN_SIGNAL_SWITCH_PIN, false, DEBOUNCE_DELAY_MS);

    // I2C bus shared by the ADS7830 and the LCD
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    /*
        Let ESP32Servo manage all PWM timers (servo, buzzer and motor).
        Each output needs its own frequency (servo 50 Hz, buzzer 440 Hz, motor 20 kHz).
        Outputs sharing a timer would also share its frequency.
        The library assigns a separate timer to each frequency.
    */
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    /*
        Steering servo
        - 50 Hz: standard servo signal, one pulse every 20 ms.
        - Pulse width sets the angle: 500 us = 0 degrees, 2400 us = 180 degrees
        - Start centered (90 degrees = wheels straight).
    */
    servo.setPeriodHertz(SERVO_PWM_FREQUENCY_HZ);
    servo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    servo.write(servoAngle);

    // Buzzer silent
    passiveBuzzerPwm.attachPin(PASSIVE_BUZZER_PIN, HORN_FREQUENCY_HZ, BUZZER_PWM_RESOLUTION_BITS);
    passiveBuzzerPwm.write(0);

    // Motor stopped
    motorForwardPwm.attachPin(MOTOR_INPUT3_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    motorBackwardPwm.attachPin(MOTOR_INPUT4_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    applyMotorSpeed(0.0f);

    // LEDs
    pinMode(TURN_SIGNAL_LED_PIN, OUTPUT);
    pinMode(HEADLIGHTS_LED_PIN, OUTPUT);
    digitalWrite(TURN_SIGNAL_LED_PIN, LOW);
    digitalWrite(HEADLIGHTS_LED_PIN, LOW);

    // LCD and custom characters
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.createChar(LCD_CHAR_ARROW_FORWARD, arrowForwardIcon);
    lcd.createChar(LCD_CHAR_ARROW_BACKWARD, arrowBackwardIcon);
    lcd.createChar(LCD_CHAR_BULB, bulbIcon);
    /*
       The LCD has two memories:
       - display memory (DDRAM): the characters shown on the screen
       - custom-character memory (CGRAM): the shapes of our icons (arrows and bulb)

       createChar() switches the LCD to custom-character memory and stays there.
       home() switches back to display memory.
       Without it, the next print() would overwrite the icons instead of showing text on the screen.
   */
    lcd.home();

    lcd.print("Calibrating...");

    /*
        Joystick center (do not touch the joystick during startup).
        Each axis is read 16 times and averaged to get a stable rest position,
        used later to detect when the joystick is moved.
    */
    joystickCenterX = readAverage(JOYSTICK_VRX_ADC_INPUT, JOYSTICK_CALIBRATION_SAMPLES);
    joystickCenterY = readAverage(JOYSTICK_VRY_ADC_INPUT, JOYSTICK_CALIBRATION_SAMPLES);
    Serial.printf("Joystick center: X = %d Y = %d\n", joystickCenterX, joystickCenterY);

    lcd.clear();

    unsigned long now = millis();
    lastSpeedUpdateMs = now;
    lastServoUpdateMs = now;
    lastHeadlightsUpdateMs = now;
    lastLcdRefreshMs = now;
    lastDebugPrintMs = now;

    Serial.println("---> DRAGULA READY!");
}

void loop()
{
    unsigned long now = millis();

    // Checked on every loop for a fast response
    updateHorn();
    updateTurnSignal(now);

    if (now - lastSpeedUpdateMs >= SPEED_UPDATE_PERIOD_MS)
    {
      updateSpeed(now);
    }

    if (now - lastServoUpdateMs >= SERVO_UPDATE_PERIOD_MS)
    {
      lastServoUpdateMs = now;
      updateSteering();
    }

    if (now - lastHeadlightsUpdateMs >= HEADLIGHTS_UPDATE_PERIOD_MS)
    {
      lastHeadlightsUpdateMs = now;
      updateHeadlights();
    }

    if (now - lastLcdRefreshMs >= LCD_REFRESH_PERIOD_MS)
    {
      lastLcdRefreshMs = now;
      updateLCD();
    }

    if (now - lastDebugPrintMs >= DEBUG_PRINT_PERIOD_MS)
    {
      lastDebugPrintMs = now;
      printDebug();
    }
}