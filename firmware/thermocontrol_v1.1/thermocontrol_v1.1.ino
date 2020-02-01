/*
  Скетч к проекту "Терморегулятор"
  - Страница проекта (схемы, описания): https://alexgyver.ru/thermocontrol/
  - Исходники на GitHub:
  Проблемы с загрузкой? Читай гайд для новичков: https://alexgyver.ru/arduino-first/
  Нравится, как написан код? Поддержи автора! https://alexgyver.ru/support_alex/
  Автор: AlexGyver, AlexGyver Technologies, 2020
  https://www.youtube.com/c/alexgyvershow
  https://github.com/AlexGyver
  https://AlexGyver.ru/
  alex@alexgyver.ru
*/

/*
  Терморегулятор с выходом ШИМ 20 кГц 0-100%
  Датчик температуры: термистор или DS18b20
  Управление кнопкой:
  - 1x клик: вкл/выкл вентиль
  - 2x клик: задать минимальную температуру
  - 3x клик: задать максимальную температуру
  - 4x клик - сброс на "стандартные" MIN_TEMP и MAX_TEMP
  Поддерживается ATmega328 (Arduino NANO) и ATtiny85 (Digispark)

  ШИМ пин:
  - D3 на Arduino (PD3 чип)
  - P1 на Digispark (PB1 чип)

  Пин термистора/далласа:
  - A0 на Arduino (PC0 чип)
  - A1 (P2) на Digispark (PB2 чип)

  Пин кнопки:
  - D2 на Arduino (PD2 чип)
  - P3 на Digispark (PB3 чип)

  Пин пищалки:
  - D4 на Arduino (PD2 чип)
  - P0 на Digispark (PB0 чип)
*/

// ========= ДАТЧИК ========
#define SENSOR_TYPE 0   // 0 - термистор, 1 - ds18b20

// для термистора!
#define T_RESIST 10000  // сопротивление термистора при 25 градусах
#define R_RESIST 10000  // сопротивление резистора
#define B_COEF 4300     // B коэффициент
// серый 4300
// проводной 3950

// ===== РЕЖИМ РАБОТЫ =====
#define MIN_TEMP 30     // мин. температура
#define MAX_TEMP 45     // макс. температура
#define MIN_SPEED 30    // (0-255) мин скорость
#define MAX_SPEED 255   // (0-255) макс скорость
#define ALARM_TEMP 50   // температура тревоги
#define BUZZER_TYPE 0   // тип пищалки: 0 пассивный, 1 активный

// ========= КНОПКА ========
#define BTN_CONTROL 1   // настройка кнопкой, 0 вкл, 1 выкл
// если выкл, режим работы будет такой, как задан выше
// если вкл, то настройки можно будет изменить кнопкой

// ========= СИГНАЛ ========

// ===== ДЛЯ РАЗРАБОТЧИКОВ =====
// отладка
#define DEBUG_ENABLE 0
#if (DEBUG_ENABLE == 1)
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

// выбор пинов
#if defined(__AVR_ATtiny85__)
#pragma message ("tiny85")
#define BTN_PIN PB3   // кнопка
#define SENS_PIN A1   // датчик
#define OUT_PIN PB1   // выход
#define BUZ_PIN PB0   // пищалка
#else
#define BTN_PIN 2     // кнопка
#define SENS_PIN A0   // датчик
#define OUT_PIN 3     // выход
#define BUZ_PIN 4     // пищалка
#endif

// либы
#include "timerMinim.h"
timerMinim fanTimer(60);
timerMinim sensorTimer(2000);

#if (SENSOR_TYPE == 0)
#include "thermistorMinim.h"
// (пин, R термистора, B термистора, базовая температура, R резистора)
thermistor therm(SENS_PIN, T_RESIST, B_COEF, 25, R_RESIST);

#elif (SENSOR_TYPE == 1)
#include <microDS18B20.h>;
MicroDS18B20 dallas(SENS_PIN);
#endif

#if (BTN_CONTROL == 1)
#include <avr/eeprom.h>
// макросы для епрома
#define EEPROM_UPD_INT(addr, val) eeprom_update_word((uint16_t*)(addr), (uint16_t)(val))
#define EEPROM_READ_INT(addr) eeprom_read_word((uint16_t*)(addr))

#include <GyverButton.h>
GButton btn(BTN_PIN);
#endif

// дата
int16_t minTemp = MIN_TEMP, maxTemp = MAX_TEMP;
bool systemState = true;
int newFanSpeed = 0;
int currentTemp = 0;
int fanSpeed = 0;
uint32_t thisTime;
bool buzFlag = false;
bool alarmFlag = false;

void setup() {
#if (DEBUG_ENABLE == 1)
  Serial.begin(9600);
#endif

  // пинаем далласа
#if (SENSOR_TYPE == 1)
  dallas.requestTemp();
#endif

  // восстанавливаем настройки (если с кнопкой)
#if (BTN_CONTROL == 1)
  if (eeprom_read_byte(0) != 50) { // первый запуск
    eeprom_write_byte(0, 50);
    EEPROM_UPD_INT(1, minTemp);
    EEPROM_UPD_INT(3, maxTemp);
  }
  minTemp = (int)EEPROM_READ_INT(1);
  maxTemp = (int)EEPROM_READ_INT(3);
#endif
  initPWM();
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
}

void loop() {
  tempToSpeed();
  fanTick();
  buttonTick();
}

// преобразуем температуру в скорость
void tempToSpeed() {
  if (sensorTimer.isReady()) {
#if (SENSOR_TYPE == 0)
    currentTemp = therm.getTempAverage();
#elif (SENSOR_TYPE == 1)
    currentTemp = dallas.getTemp(); // получить с далласа
    dallas.requestTemp();           // запросить измерение
#endif
    // преобразовать диапазон и ограничить значение
    if (!systemState) newFanSpeed = 0;          // если выкл - скорость 0
    else {
      newFanSpeed = map(currentTemp, minTemp, maxTemp, MIN_SPEED, MAX_SPEED);
      newFanSpeed = constrain(newFanSpeed, MIN_SPEED, MAX_SPEED);
    }

    initPWM();    // заново инициализируем шим, тон его ломает
    if (currentTemp > ALARM_TEMP) {
      alarmFlag = !alarmFlag;
#if (BUZZER_TYPE == 0)
      if (alarmFlag) tone(BUZ_PIN, 400, 900);
#else
      digitalWrite(BUZ_PIN, alarmFlag);
#endif
    } else {
      digitalWrite(BUZ_PIN, 0);
    }
    //DEBUG(currentTemp);
  }
}

// плавный контроль вентилятора
void fanTick() {
  if (fanTimer.isReady()) {
    if (newFanSpeed > fanSpeed) fanSpeed += 2;  // плавно управляем
    if (newFanSpeed < fanSpeed) fanSpeed -= 2;  // плавно управляем
    fanSpeed = constrain(fanSpeed, 0, 255);  // ограничить
#if defined(__AVR_ATtiny85__)
    tiny85_PWM20kHz(fanSpeed);    // ШИМ для тини
#else
    mega328_PWM20kHz_D3(fanSpeed);  // ШИМ для меги
#endif
    //DEBUG(fanSpeed);
  }
}

// отработка кнопки
void buttonTick() {
#if (BTN_CONTROL == 1)
  btn.tick();
  if (btn.hasClicks()) {
    switch (btn.getClicks()) {
      case 1:
        systemState = !systemState;
        buzzer(1);
        break;
      case 2:
        minTemp = currentTemp;
        EEPROM_UPD_INT(1, minTemp);
        buzzer(2);
        break;
      case 3:
        maxTemp = currentTemp;
        EEPROM_UPD_INT(3, maxTemp);
        buzzer(3);
        break;
      case 4:
        minTemp = MIN_TEMP;
        maxTemp = MAX_TEMP;
        EEPROM_UPD_INT(1, minTemp);
        EEPROM_UPD_INT(3, maxTemp);
        buzzer(4);
        break;
    }
  }
#endif
}

// пищатель
void buzzer(byte count) {
  for (byte i = 0; i < count; i++) {
#if (BUZZER_TYPE == 0)
    tone(BUZ_PIN, 400, 200);
    delay(200);
#else
    digitalWrite(BUZ_PIN, 1);
    delay(200);
    digitalWrite(BUZ_PIN, 0);
#endif
    delay(70);
  }
  initPWM();  // заново инициализируем шим, тон его ломает
}

void initPWM() {
#if defined(__AVR_ATtiny85__)
  // инициализация ШИМ 20 кГц для tiny85
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(WGM02) | 0x02;
  OCR0A = 99;
#else
  // инициализация ШИМ 20 кГц для mega328
  TCCR2A = 0b10100011;
  TCCR2B = 0b00001010;
  OCR2A = 99;
#endif
}

// ШИМ 20 кГц
#if defined(__AVR_ATtiny85__)
void tiny85_PWM20kHz(uint8_t duty) {
  OCR0B = (float)duty * 0.39;
}
#else
void mega328_PWM20kHz_D3(uint8_t duty) {
  if (duty == 0) {
    bitClear(TCCR2A, COM2B1);
    bitClear(PORTD, 3);
  } else {
    bitSet(TCCR2A, COM2B1);
    OCR2B = map(duty, 0, 255, 0, 99);
  }
}
#endif
