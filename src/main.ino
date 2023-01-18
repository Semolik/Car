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
#define CLK 12
#define DIO 11

volatile boolean LEDHdlts;
bool low_power = false;
int battery_pin = A4;

// ===========================

#include <GyverMotor.h>
#include <Thread.h>
// (тип, пин, ШИМ пин, уровень)
GMotor motorR(DRIVER2WIRE, MOT_RA, MOT_RB, HIGH);
GMotor motorL(DRIVER2WIRE, MOT_LA, MOT_LB, HIGH);

#include <PS2X_lib.h>
PS2X ps2x;

#include "GyverTM1637.h"
GyverTM1637 disp(CLK, DIO);
Thread mainThread = Thread(); // создаём поток управления светодиодом
Thread batteryThread = Thread();
void setup()
{
  Serial.begin(9600);
  mainThread.onRun(car); // назначаем потоку задачу
  mainThread.setInterval(20);

  batteryThread.onRun(buttory_check);
  batteryThread.setInterval(1000);
  disp.clear();
  disp.brightness(7);

  motorR.setMode(AUTO);
  motorL.setMode(AUTO);

  // НАПРАВЛЕНИЕ ГУСЕНИЦ (зависит от подключения)
  motorR.setDirection(NORMAL);
  motorL.setDirection(NORMAL);

  // мин. сигнал вращения
  motorR.setMinDuty(MIN_DUTY);
  motorL.setMinDuty(MIN_DUTY);

  // плавность скорости моторов
  motorR.setSmoothSpeed(1000);
  motorL.setSmoothSpeed(1000);

  // подрубаем геймпад
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

  pinMode(LED_BUILTIN, OUTPUT);
  if (battery_percentage() < 10)
  {
    low_power = true;
  }
}

void car()
{
  bool success = ps2x.read_gamepad(false, 0); // читаем
  ps2x.reconfig_gamepad();                    // костыль https://stackoverflow.com/questions/46493222/why-arduino-needs-to-be-restarted-after-ps2-controller-communication-in-arduino
  if (success)
  {
    if (ps2x.ButtonPressed(PSB_GREEN)) // Triangle pressed
    {
      LEDHdlts = !LEDHdlts; // Toggle the LED light flag
    }

    // преобразуем стики от 0..255 к -255, 255
    int LX = map(ps2x.Analog(PSS_RX), 255, 0, -255, 255);
    int LY = map(ps2x.Analog(PSS_LY), 255, 0, -255, 255);

    float dutyR = constrain(LX, -255, 255);
    float dutyL = constrain(LY, -255, 255);
    // задаём целевую скорость
    motorR.smoothTick(dutyR == -1 ? 0 : dutyR);
    motorL.smoothTick(dutyL == -1 ? 0 : dutyL);

    if (dutyR == -1 && dutyL == -1)
    {
      disp.displayInt(battery_percentage());
    }
    else
    {
      Serial.print("asdasd");
    }
    digitalWrite(LED_BUILTIN, LEDHdlts);
  }
  else
  {
    // проблема с геймпадом - остановка
    motorR.setSpeed(0);
    motorL.setSpeed(0);
  }
}
void loop()
{

  if (mainThread.shouldRun())
    mainThread.run();

  if (batteryThread.shouldRun())
    batteryThread.run();
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
  Serial.print("voltage: ");
  Serial.println(voltage, 2);
  output = ((voltage - battery_min) / (battery_max - battery_min)) * 100;
  if (output < 100)
    return output;
  else
    return 100.0;
}