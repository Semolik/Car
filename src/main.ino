#include <Arduino.h>
#include <GyverMotor.h>
#include <Thread.h>
#include <PS2X_lib.h>
#include <GyverTM1637.h>
#include <FastLED.h>

// мин. сигнал, при котором мотор начинает вращение
#define MIN_DUTY 120

// пины драйвера
#define MOT_RA 3
#define MOT_RB 5
#define MOT_LA 9
#define MOT_LB 10

// пины ресивера ps2
#define PS2_DAT A0
#define PS2_CMD A1
#define PS2_SEL A2
#define PS2_CLK A3

// пины дисплея
#define CLK 12
#define DIO 11

// подсветка
#define NUM_LEDS 5
#define DATA_PIN 13
CRGB leds[NUM_LEDS];
volatile boolean LED_ON;
volatile int led_hue = 0;
// фары

#define HEADLIGHT_PIN 6  // led
#define TAIL_LIGHT_PIN 7 // ws2812b
#define TAIL_LIGHT_LED_COUNT 3
CRGB tail_light_leds[TAIL_LIGHT_LED_COUNT];
volatile boolean lights_on = false;

// батарея
#define BATTERY_PIN A4

// светодиод индикатор
#define LOW_POWER_LED 8
volatile boolean low_power;

unsigned long time;

// (тип, пин, ШИМ пин, уровень)
GMotor motorR(DRIVER2WIRE, MOT_RA, MOT_RB, HIGH);
GMotor motorL(DRIVER2WIRE, MOT_LA, MOT_LB, HIGH);

PS2X ps2x;

GyverTM1637 disp(CLK, DIO);

Thread mainThread = Thread();
Thread batteryThread = Thread();

void setup()
{
  Serial.begin(9600);
  disp.clear();
  disp.brightness(7);

  mainThread.onRun(car);
  mainThread.setInterval(20);

  batteryThread.onRun(buttory_check);
  batteryThread.setInterval(1000);

  motorR.setMode(AUTO);
  motorL.setMode(AUTO);
  motorR.setDirection(NORMAL);
  motorL.setDirection(NORMAL);
  motorR.setMinDuty(MIN_DUTY);
  motorL.setMinDuty(MIN_DUTY);
  motorR.setSmoothSpeed(1000);
  motorL.setSmoothSpeed(1000);

  // подрубаем геймпад
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

  low_power = battery_percentage() < 10;

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, TAIL_LIGHT_PIN>(tail_light_leds, TAIL_LIGHT_LED_COUNT);
}

void car()
{
  if ((millis() - time) > 10000 && battery_percentage() < 10)
  {
    low_power = true;
  }

  bool success = ps2x.read_gamepad(false, 0); // читаем
  ps2x.reconfig_gamepad();                    // костыль https://stackoverflow.com/questions/46493222/why-arduino-needs-to-be-restarted-after-ps2-controller-communication-in-arduino
  if (success)
  {
    if (ps2x.Button(PSB_L1))
    {
      if (led_hue < 240)
      {
        led_hue += 2;
      }
      else
      {
        led_hue = 0;
      }
    }
    if (ps2x.ButtonPressed(PSB_R1))
    {
      LED_ON = !LED_ON;
    }
    if (ps2x.ButtonPressed(PSB_CIRCLE))
    {
      lights_on = !lights_on;
    }

    if (LED_ON)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i].setHue(led_hue);
      }
      FastLED.show();
      if (!lights_on)
      {
        for (int i = 0; i < TAIL_LIGHT_LED_COUNT; i++)
        {
          tail_light_leds[i].setHue(led_hue);
        }
        FastLED.show();
      }
    }
    else
    {
      off_leds();
    }
    int LX = map(ps2x.Analog(PSS_RX), 255, 0, -255, 255);
    int LY = map(ps2x.Analog(PSS_LY), 255, 0, -255, 255);

    float dutyR = constrain(LX, -255, 255);
    float dutyL = constrain(LY, -255, 255);
    if (lights_on)
    {
      uint8_t brightness = 50;
      uint8_t brightness_tail = brightness;
      uint8_t brightness_headlight = brightness;

      if (dutyL < -1)
      {
        brightness_tail = map(dutyL, -1, -255, brightness, 255);
      }
      else if (dutyL > 1)
      {
        brightness_headlight = map(dutyL, 1, 255, brightness, 255);
      }
      for (int i = 0; i < TAIL_LIGHT_LED_COUNT; i++)
      {
        tail_light_leds[i].setRGB(brightness_tail, 0, 0);
      }
      FastLED.show();
      Serial.println(brightness_headlight);
      analogWrite(HEADLIGHT_PIN, brightness_headlight);
    }
    else
    {
      analogWrite(HEADLIGHT_PIN, 0);
    }

    motorR.smoothTick(dutyR == -1 ? 0 : dutyR);
    motorL.smoothTick(dutyL == -1 ? 0 : dutyL);

    if (dutyR != -1 || dutyL != -1)
    {
      time = millis();
    }
  }
  else
  {
    off_all();
  }
}
void off_all()
{
  motorR.setSpeed(0);
  motorL.setSpeed(0);
  off_leds();
}
void off_leds()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].setRGB(0, 0, 0);
  }
  FastLED.show();
  if (!lights_on)
  {
    for (int i = 0; i < TAIL_LIGHT_LED_COUNT; i++)
    {
      tail_light_leds[i].setRGB(0, 0, 0);
    }
    FastLED.show();
  }
}
void loop()
{
  if (!low_power && mainThread.shouldRun())
  {
    mainThread.run();
  }
  else if (low_power)
  {
    off_all();
  }
  digitalWrite(LOW_POWER_LED, low_power ? HIGH : LOW);

  if (batteryThread.shouldRun())
  {
    batteryThread.run();
  }
}

void buttory_check()
{
  if (low_power)
  {
    disp.displayInt(battery_percentage());
  }
  else
  {
    disp.clear();
  }
}
int battery_percentage()
{
  float voltage = ((analogRead(BATTERY_PIN) * 5.0) / 1023.0) + 0.2; // for default reference voltage
  voltage = voltage * 10;
  int bat_percent;
  if (voltage >= 42)
    bat_percent = 100;
  else if (voltage >= 41) // 90-100% range
    bat_percent = map(voltage, 41, 42, 90, 100);
  else if (voltage >= 37) // 10-90% range
    bat_percent = map(voltage, 37, 41, 10, 90);
  else if (voltage >= 36) // 0-10% range
    bat_percent = map(voltage, 36, 37, 0, 10);
  else
    bat_percent = 0;
  return bat_percent;
}