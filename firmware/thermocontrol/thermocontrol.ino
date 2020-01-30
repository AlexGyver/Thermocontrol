/*
  Скетч к проекту "Терморегулятор"
  - Страница проекта (схемы, описания):
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
  - P2 на Digispark (PB2 чип)

  Пин кнопки:
  - D2 на Arduino (PD2 чип)
  - P3 на Digispark (PB3 чип)
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
#define MIN_TEMP 25     // мин. температура
#define MAX_TEMP 35     // макс. температура
#define MIN_SPEED 30    // (0-255) мин скорость
#define MAX_SPEED 255   // (0-255) макс скорость
#define ALARM_TEMP 60   // температура тревоги
#define BUZZER_TYPE 0   // тип пищалки: 0 пассивный, 1 активный

// ========= КНОПКА ========
#define BTN_CONTROL 1   // настройка кнопкой, 0 вкл, 1 выкл
// если выкл, режим работы будет такой, как задан выше
// если вкл, то настройки можно будет изменить кнопкой

// ========= СИГНАЛ ========

// ===== ДЛЯ РАЗРАБОТЧИКОВ =====
// отладка
#define DEBUG_ENABLE 1
#if (DEBUG_ENABLE == 1)
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

// выбор пинов
#if defined(__AVR_ATtiny85__)
#define BTN_PIN PB3   // кнопка
#define SENS_PIN PB2  // датчик
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
timerMinim fanTimer(50);
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
uint32_t thisTime;
bool buzFlag = false;
bool alarmFlag = false;

void setup() {
#if (DEBUG_ENABLE == 1)
  Serial.begin(9600);
#endif

#if defined(__AVR_ATtiny85__)
  // инициализация ШИМ 20 кГц для tiny85
  PLLCSR = (1 << PLLE);          // PLL enable
  while (!PLLCSR & (1 << PLOCK)); // PLL locked
  PLLCSR |= (1 << PCKE);         // Timer1 clock - 64MHz from PLL

  TCCR1 = (1 << PWM1A) | (1 << COM1A1) | 0x05; // PWM A enable , pin connected to timer, prescaler - /16
  OCR1C = 199;
  pinMode(OUT_PIN, OUTPUT);
#else
  // инициализация ШИМ 20 кГц для mega328
  TCCR2A = 0b10100011;
  TCCR2B = 0b00001010;
  OCR2A = 99;
  pinMode(OUT_PIN, OUTPUT);
#endif

  // пинаем далласа
#if (SENSOR_TYPE == 1)
  dallas.requestTemp();
#endif

  // восстанавливаем настройки (если с кнопкой)
#if (BTN_CONTROL == 1)
  if (eeprom_read_byte(0) != 25) { // первый запуск
    eeprom_write_byte(0, 25);
    EEPROM_UPD_INT(1, minTemp);
    EEPROM_UPD_INT(3, maxTemp);
  }
  minTemp = (int)EEPROM_READ_INT(1);
  maxTemp = (int)EEPROM_READ_INT(3);
#endif
  pinMode(BUZ_PIN, OUTPUT);
}

void loop() {
  tempToSpeed();
  fanTick();
  buttonTick();
  buzzTick();
}

void buzzTick() {
#if (BUZZER_TYPE == 0)
  if (alarmFlag && micros() - thisTime >= 2500) {
    thisTime = micros();
    digitalWrite(BUZ_PIN, buzFlag);
    buzFlag = !buzFlag;
  }
}
#endif

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
    newFanSpeed = map(currentTemp, minTemp, maxTemp, MIN_SPEED, MAX_SPEED);
    newFanSpeed = constrain(newFanSpeed, MIN_SPEED, MAX_SPEED);    
    if (currentTemp > ALARM_TEMP) {
      alarmFlag = !alarmFlag;
      digitalWrite(BUZ_PIN, alarmFlag);
    } else {
      alarmFlag = false;
      digitalWrite(BUZ_PIN, 0);
    }
    //DEBUG(currentTemp);
  }
}

// плавный контроль вентилятора
void fanTick() {
  if (fanTimer.isReady()) {
    static int fanSpeed = 0;
    if (!systemState) newFanSpeed = 0;          // если выкл - скорость 0
    if (newFanSpeed > fanSpeed) fanSpeed += 1;  // плавно управляем
    if (newFanSpeed < fanSpeed) fanSpeed -= 1;  // плавно управляем
    if (systemState) fanSpeed = constrain(fanSpeed, 0, 255);  // ограничить
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
    uint32_t thisTime = millis();
    uint32_t buzTimer = micros();
    digitalWrite(BUZ_PIN, 1);
    while (millis() - thisTime < 200) {
#if (BUZZER_TYPE == 0)
      if (micros() - buzTimer >= 1000) {
        buzTimer = micros();
        digitalWrite(BUZ_PIN, buzFlag);
        buzFlag = !buzFlag;
      }
#endif
    }
    digitalWrite(BUZ_PIN, 0);
    delay(70);
  }
}

// ШИМ 20 кГц
#if defined(__AVR_ATtiny85__)
void tiny85_PWM20kHz(uint8_t duty) {
  OCR1A = (float)duty * 0.78;
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
