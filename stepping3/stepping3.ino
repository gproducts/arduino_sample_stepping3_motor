/*
MIT License

Copyright (c) 2025 G.Products

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ArduinoOTA.h>
#include <WiFi.h>

// If ESP-IDF environment can be used,
// you can use the native functions for ADC to improve the accuracy.
// #define ESPIDF

#ifdef ESPIDF
#include "driver/adc.h"
#include "esp_adc_cal.h"
#endif

// Port
#define PIN_LED_STATUS (GPIO_NUM_13)
#define PIN_LED_VFREAD (GPIO_NUM_14)

#define INA1 (GPIO_NUM_27)  // IC3
#define INA2 (GPIO_NUM_26)  // IC3
#define INB1 (GPIO_NUM_25)  // IC2
#define INB2 (GPIO_NUM_33)  // IC2
#define INC1 (GPIO_NUM_17)  // IC1
#define INC2 (GPIO_NUM_16)  // IC1

#define INA1_CH (1)  // IC3
#define INA2_CH (2)  // IC3
#define INB1_CH (3)  // IC2
#define INB2_CH (4)  // IC2
#define INC1_CH (5)  // IC1
#define INC2_CH (6)  // IC1

// PWM
#define LEDC_MAX_DUTY (256)
#define LEDC_PWMFREQ (200000)
#define LEDC_TIMER_BIT (8)

// ADC
#define VSENA_PORT (GPIO_NUM_34)

#ifdef ESPIDF
esp_adc_cal_characteristics_t adcChar;
#define ADC_CH (ADC_CHANNEL_6)
#define ADC1_CH (ADC1_CHANNEL_6)
#define ADC_WIDTH (ADC_WIDTH_BIT_12)
#define ADC_ATTEN (ADC_ATTEN_DB_0)  // 100mV ~ 950mV
#define ADC_UNIT (ADC_UNIT_1)
#endif

// WiFi Setting for OTA
const char *ssid = "OTA_AP";
const char *pass = "password";
const IPAddress ip(192, 168, 10, 1);
const IPAddress subnet(255, 255, 255, 0);

void setup() {
  delay(1000);
  Serial.begin(115200);

  // -------------------------------------------------
  // MUST KEEP OTA CODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(ssid, pass);
  IPAddress address = WiFi.softAPIP();
  Serial.println(address);
  ArduinoOTA.onStart([]() {})
    .onEnd([]() {})
    .onProgress([](unsigned int progress, unsigned int total) {})
    .onError([](ota_error_t error) {});
  ArduinoOTA.begin();
  // -------------------------------------------------

  // GPIO initialize for LED
  pinMode(PIN_LED_VFREAD, INPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // GPIO initialize for Motor
  // ledcSetup(INA1_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INA1, INA1_CH);
  ledcAttachChannel(INA1, LEDC_PWMFREQ, LEDC_TIMER_BIT, INA1_CH);
  ledcWrite(INA1_CH, 0);

  // ledcSetup(INA2_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INA2, INA2_CH);
  ledcAttachChannel(INA2, LEDC_PWMFREQ, LEDC_TIMER_BIT, INA2_CH);
  ledcWrite(INA2_CH, 0);

  // ledcSetup(INB1_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INB1, INB1_CH);
  ledcAttachChannel(INB1, LEDC_PWMFREQ, LEDC_TIMER_BIT, INB1_CH);
  ledcWrite(INB1_CH, 0);

  // ledcSetup(INB2_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INB2, INB2_CH);
  ledcAttachChannel(INB2, LEDC_PWMFREQ, LEDC_TIMER_BIT, INB2_CH);
  ledcWrite(INB2_CH, 0);

  // ledcSetup(INC1_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INC1, INC1_CH);
  ledcAttachChannel(INC1, LEDC_PWMFREQ, LEDC_TIMER_BIT, INC1_CH);
  ledcWrite(INC1_CH, 0);

  // ledcSetup(INC2_CH, LEDC_PWMFREQ, LEDC_TIMER_BIT);
  // ledcAttachPin(INC2, INC2_CH);
  ledcAttachChannel(INC2, LEDC_PWMFREQ, LEDC_TIMER_BIT, INC2_CH);
  ledcWrite(INC2_CH, 0);

  // ADC initialize
  pinMode(VSENA_PORT, INPUT);
#ifdef ESPIDF
  adc_gpio_init(ADC_UNIT, ADC_CH);
  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC1_CH, ADC_ATTEN);  // 100~950mV
  esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, 1100, &adcChar);
#else
  analogSetPinAttenuation(VSENA_PORT, ADC_0db);
#endif
}

uint32_t adc_meas_current(void) {
  uint32_t adc, voltage;
#ifdef ESPIDF
  esp_adc_cal_get_voltage(ADC_CH, &adcChar, &voltage);
#else
  adc = analogReadMilliVolts(VSENA_PORT);
  voltage = adc;
#endif
  return voltage;
}

void pwm_set_duty(uint8_t port, uint8_t duty)  // duty : 0~100%
{
  // uint8_t ch = 0;
  // switch (port) {
  //   case INA1:
  //     ch = INA1_CH;
  //     break;
  //   case INA2:
  //     ch = INA2_CH;
  //     break;
  //   case INB1:
  //     ch = INB1_CH;
  //     break;
  //   case INB2:
  //     ch = INB2_CH;
  //     break;
  //   case INC1:
  //     ch = INC1_CH;
  //     break;
  //   case INC2:
  //     ch = INC2_CH;
  //     break;
  //   default:
  //     return;
  //     break;
  // }
  // ledcWrite(ch, (duty * LEDC_MAX_DUTY / 100));
  ledcWrite(port, (duty * LEDC_MAX_DUTY / 100));
}

void sm_3phase(uint8_t mstate, uint8_t pwm) {
  switch (mstate) {
    case 0:                     // for initializing
      pwm_set_duty(INA1, 100);  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);  //*OUT2 L

      pwm_set_duty(INB1, 100);  //*OUT3 L
      pwm_set_duty(INB2, 100);  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);  //*OUT5 L
      pwm_set_duty(INC2, 100);  // OUT6 L (not use)
      break;

    case 1:
      pwm_set_duty(INA1, (100 - pwm));  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);          //*OUT2 PWM (H)

      pwm_set_duty(INB1, 100);  //*OUT3 L
      pwm_set_duty(INB2, 100);  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);  //*OUT5 L
      pwm_set_duty(INC2, 100);  // OUT6 L (not use)
      break;

    case 2:
      pwm_set_duty(INA1, (100 - pwm));  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);          //*OUT2 PWM (H)

      pwm_set_duty(INB1, 100);          //*OUT3 PWM (H)
      pwm_set_duty(INB2, (100 - pwm));  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);  //*OUT5 L
      pwm_set_duty(INC2, 100);  // OUT6 L (not use)
      break;

    case 3:
      pwm_set_duty(INA1, 100);  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);  //*OUT2 L

      pwm_set_duty(INB1, 100);          //*OUT3 PWM (H)
      pwm_set_duty(INB2, (100 - pwm));  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);  //*OUT5 L
      pwm_set_duty(INC2, 100);  // OUT6 L (not use)
      break;

    case 4:
      pwm_set_duty(INA1, 100);  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);  //*OUT2 L

      pwm_set_duty(INB1, 100);          //*OUT3 PWM (H)
      pwm_set_duty(INB2, (100 - pwm));  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);          //*OUT5 PWM (H)
      pwm_set_duty(INC2, (100 - pwm));  // OUT6 L (not use)
      break;

    case 5:
      pwm_set_duty(INA1, 100);  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);  //*OUT2 L

      pwm_set_duty(INB1, 100);  //*OUT3 L
      pwm_set_duty(INB2, 100);  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);          //*OUT5 PWM (H)
      pwm_set_duty(INC2, (100 - pwm));  // OUT6 L (not use)
      break;

    case 6:
      pwm_set_duty(INA1, (100 - pwm));  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);          //*OUT2 PWM (H)

      pwm_set_duty(INB1, 100);  //*OUT3 L
      pwm_set_duty(INB2, 100);  // OUT4 L (not use)

      pwm_set_duty(INC1, 100);          //*OUT5 PWM (H)
      pwm_set_duty(INC2, (100 - pwm));  // OUT6 L (not use)
      break;

    case 100:                   // for detection
      pwm_set_duty(INA1, 100);  // OUT1 L (not use)
      pwm_set_duty(INA2, 100);  //*OUT2 L

      pwm_set_duty(INC1, 100);          //*OUT5 PWM (H)
      pwm_set_duty(INC2, (100 - pwm));  // OUT6 L (not use)

      pwm_set_duty(INB1, 0);  //*OUT3 OFF
      pwm_set_duty(INB2, 0);  // OUT4 OFF (not use)
      break;

    case 200:
    default:
      pwm_set_duty(INA1, 0);
      pwm_set_duty(INA2, 0);
      pwm_set_duty(INB1, 0);
      pwm_set_duty(INB2, 0);
      pwm_set_duty(INC1, 0);
      pwm_set_duty(INC2, 0);
      break;
  }
}


uint8_t loop_count = 0;
uint8_t mstate = 0;
uint8_t step = 0;

uint32_t voltage, current;
void loop() {
  // -------------------------------------------------
  // MUST KEEP OTA CODE
  ArduinoOTA.handle();
  // -------------------------------------------------

  if (++loop_count > 100) {
    digitalWrite(PIN_LED_STATUS, HIGH);
    loop_count = 0;

    // motor, step=50, pwm=40%
    for (step = 0; step < 50; step++) {
      if (++mstate > 6) {
        mstate = 1;
      }
      sm_3phase(mstate, 40);
      delay(2);  // speed
      voltage += adc_meas_current();
      delay(2);  // speed
    }
    sm_3phase(mstate, 1);

    // Current measurement
    voltage = voltage / step;
    current = 100 * voltage / 47;  // voltage / 0.47ohm

    digitalWrite(PIN_LED_STATUS, LOW);

    // Log
    char line[100];
    sprintf(line, "%d[mV], %d[mA]", voltage, current);
    Serial.println(line);
  }
  delay(5);
}
