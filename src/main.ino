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

// параметры светодиодов
#define NUM_LEDS 5
#define DATA_PIN 13

volatile boolean LED_ON;
volatile boolean low_power;
int battery_pin = A4;
volatile boolean isStopped;
volatile int led_hue = 0;
unsigned long time;

// (тип, пин, ШИМ пин, уровень)
GMotor motorR(DRIVER2WIRE, MOT_RA, MOT_RB, HIGH);
GMotor motorL(DRIVER2WIRE, MOT_LA, MOT_LB, HIGH);

PS2X ps2x;

GyverTM1637 disp(CLK, DIO);

Thread mainThread = Thread();
Thread batteryThread = Thread();

CRGB leds[NUM_LEDS];
void setup()
{

  mainThread.onRun(car);
  mainThread.setInterval(20);

  batteryThread.onRun(buttory_check);
  batteryThread.setInterval(1000);

  disp.clear();
  disp.brightness(7);

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
}

void car()
{
  if ((millis() - time > 10000) && (battery_percentage() < 10))
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
        led_hue += 3;
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
    if (LED_ON)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i].setHue(led_hue);
      }
      FastLED.show();
    }
    else
    {
      off_leds();
    }
    int LX = map(ps2x.Analog(PSS_RX), 255, 0, -255, 255);
    int LY = map(ps2x.Analog(PSS_LY), 255, 0, -255, 255);

    float dutyR = constrain(LX, -255, 255);
    float dutyL = constrain(LY, -255, 255);

    motorR.smoothTick(dutyR == -1 ? 0 : dutyR);
    motorL.smoothTick(dutyL == -1 ? 0 : dutyL);

    if (dutyR == -1 && dutyL == -1)
    {
      time = millis();
    }
  }
  else
  {
    motorR.setSpeed(0);
    motorL.setSpeed(0);
    digitalWrite(LED_BUILTIN, LOW);
    off_leds();
  }
}
void off_leds()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].setRGB(0, 0, 0);
  }
  FastLED.show();
}
void loop()
{
  if (!low_power && mainThread.shouldRun())
  {
    mainThread.run();
  }

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

  int output = 0;                 // output value
  const float battery_max = 4.20; // maximum voltage of battery
  const float battery_min = 3.0;  // minimum voltage of battery before shutdown

  // calculate the voltage
  float voltage = (analogRead(battery_pin) * 5.0) / 1023.0; // for default reference voltage
  // round value by two precision
  voltage = roundf(voltage * 100) / 100;

  output = ((voltage - battery_min) / (battery_max - battery_min)) * 100;
  if (output < 100)
    return output;
  else
    return 100.0;
}