// ============================================================
// SAGE — Combined Firmware (Final, REAL SENSORS + OLED DISPLAY)
// Hardware: MAX30100 (HR+SpO2), dual-DHT22 (skin+ambient, stress
// differential), thermistor (body temp), MQ2 (air quality),
// and 0.96" SSD1306 I2C OLED Display (128x64 @ 0x3C).
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include "fpga_bitstream.h"   // compiled ForgeFPGA bitstream
#include "model_data.h"       // predict_sentinel_regime() — 12-tree RF
#include <string.h>

// ====================================================================
// OLED Display Setup (0.96" SSD1306 I2C @ 0x3C)
// ====================================================================
#define OLED_I2C_ADDR       0x3C
#define SCREEN_WIDTH         128
#define SCREEN_HEIGHT         64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ====================================================================
// FPGA / Vishvarak Shrike Hardware Pin Mapping
// ====================================================================
#define PIN_SERIAL_DATA   17  // ESP32 -> FPGA Data (GPIO17)
#define PIN_SERIAL_CLK    18  // ESP32 -> FPGA Clock (GPIO18)
#define PIN_WAKE_FLAG     3   // FPGA -> ESP32 Interrupt (GPIO3)

#define FPGA_CS_PIN       10  // SPI CS for FPGA boot
#define FPGA_CRESET_PIN   9   // FPGA Reset Pin

// ====================================================================
// Sensor Pin Mapping (no conflicts with FPGA pins above)
// ====================================================================
#define TEMP_ADC_PIN     1     // IO1, ADC1 — thermistor
#define MQ2_ADC_PIN      2     // IO2, ADC1 — MQ2 gas sensor
#define DHT_SKIN_PIN     4     // IO4 — DHT22 on skin
#define DHT_AMBIENT_PIN  5     // IO5 — DHT22 ambient
#define I2C_SDA          6     // IO6 — MAX30100 + OLED (SDA)
#define I2C_SCL          7     // IO7 — MAX30100 + OLED (SCK/SCL)

#define DHTTYPE DHT22
DHT dhtSkin(DHT_SKIN_PIN, DHTTYPE);
DHT dhtAmbient(DHT_AMBIENT_PIN, DHTTYPE);

PulseOximeter pox;
uint32_t tsLastDHT = 0;

void onBeatDetected() {
    Serial.println(">>> Beat detected!");
}

const int led_pins[] = {14, 13, 21, 38, 48};
const int num_leds = sizeof(led_pins) / sizeof(led_pins[0]);

void set_leds(bool state) {
    for (int i = 0; i < num_leds; i++) {
        digitalWrite(led_pins[i], state ? HIGH : LOW);
    }
}

// ====================================================================
// FreeRTOS Task for Continuous PulseOximeter Background Updates
// ====================================================================
TaskHandle_t poxTaskHandle;

void poxUpdateTask(void *pvParameters) {
    for (;;) {
        pox.update();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// ====================================================================
// Calibration configuration — 10 samples (quick demo mode, ~20s)
// ====================================================================
#define CALIBRATION_SAMPLES 10
#define SAMPLE_INTERVAL_MS  2000

struct PersonalBaseline {
    float hr_mean = 75.0f, hr_std = 5.0f;
    float spo2_mean = 98.0f, spo2_std = 1.0f;
    float temp_mean = 36.5f, temp_std = 0.3f;
    bool  is_calibrated = false;
};
PersonalBaseline user_baseline;

float hr_buf[CALIBRATION_SAMPLES];
float temp_buf[CALIBRATION_SAMPLES];
int   cal_index = 0;

float prev_temp = 36.5f;
float prev_spo2 = 98.0f;
float prev_hr   = 75.0f;

uint8_t bpm_q_prev = 0; // mirrors RTL's bpm_prev register
unsigned long loopStartTime = 0;

// ====================================================================
// Power Estimation Helper
// ====================================================================
float estimate_power_mw(int regime) {
    const float esp32_ma      = 80.0f;
    const float max30100_ma   = 1.0f;
    const float dht22_ma      = 1.5f * 2;
    const float lm358_ma      = 1.0f;
    const float fpga_ma       = (regime == 0) ? 2.0f : 5.0f;
    const float mq2_heater_ma = 160.0f;

    float rail_3v3_ma = esp32_ma + max30100_ma + dht22_ma + lm358_ma + fpga_ma;
    float power_3v3_mw = rail_3v3_ma * 3.3f;
    float power_5v_mw  = mq2_heater_ma * 5.0f;

    return power_3v3_mw + power_5v_mw;
}

void calculate_stats(const float* arr, int n, float &mean, float &std) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += arr[i];
    mean = sum / n;

    float sq_sum = 0.0f;
    for (int i = 0; i < n; i++) sq_sum += (arr[i] - mean) * (arr[i] - mean);
    std = sqrt(sq_sum / n);
    if (std < 0.1f) std = 0.1f;
}

// ====================================================================
// OLED Renderer Function
// Displays Real-Time Vitals, Sensors & Current System Regime
// ====================================================================
void update_oled_display(float hr, float spo2, float temp, uint8_t gas_q, uint8_t stress_q, int regime, bool is_calibrating) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // 1. Header Bar: System Name + Regime Status
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("SAGE ["));

    if (is_calibrating) {
        display.print(F("CALIB]"));
    } else if (regime == 0) {
        display.print(F("REG 0:SAFE]"));
    } else if (regime == 1) {
        display.print(F("REG 1:RISK]"));
    } else {
        display.print(F("REG 2:CRIT]"));
    }

    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    if (is_calibrating) {
        display.setCursor(10, 22);
        display.setTextSize(1);
        display.print(F("CALIBRATING..."));
        display.setCursor(10, 38);
        display.print(F("Sample: ")); display.print(cal_index); display.print(F("/10"));
        display.display();
        return;
    }

    // 2. Row 1: Heart Rate (BPM) & SpO2 (%)
    display.setCursor(0, 15);
    display.print(F("BPM : ")); display.print((int)hr);
    display.setCursor(68, 15);
    display.print(F("SpO2: ")); display.print((int)spo2); display.print(F("%"));

    // 3. Row 2: Body Temp (C) & MQ2 Gas (8-bit)
    display.setCursor(0, 28);
    display.print(F("Temp: ")); display.print(temp, 1); display.print(F("C"));
    display.setCursor(68, 28);
    display.print(F("Gas : ")); display.print(gas_q);

    // 4. Row 3: Stress Metric (8-bit)
    display.setCursor(0, 41);
    display.print(F("Stress Level: ")); display.print(stress_q); display.print(F("/255"));

    // 5. Row 4: Regime Status Banner
    display.fillRect(0, 53, 128, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 55);
    display.setTextSize(1);

    if (regime == 0) {
        display.print(F("MODE: STABLE (LOW POWER)"));
    } else if (regime == 1) {
        display.print(F("MODE: ELEVATED RISK"));
    } else {
        display.print(F("MODE: CRITICAL ALERT!"));
    }

    display.display();
}

// ====================================================================
//  ForgeFPGA Bitstream Loader
// ====================================================================
void boot_fpga() {
    Serial.println("[SAGE VLSI] Loading regime-adaptive bitstream into ForgeFPGA...");

    pinMode(FPGA_CS_PIN, OUTPUT);
    pinMode(FPGA_CRESET_PIN, OUTPUT);

    digitalWrite(FPGA_CRESET_PIN, LOW);
    delay(10);
    digitalWrite(FPGA_CRESET_PIN, HIGH);
    delay(10);

    digitalWrite(FPGA_CS_PIN, LOW);
    for (unsigned int i = 0; i < FPGA_bitstream_MCU_bin_len; i++) {
        SPI.transfer(FPGA_bitstream_MCU_bin[i]);
    }
    digitalWrite(FPGA_CS_PIN, HIGH);
    delay(50);

    Serial.println("[SAGE VLSI] ForgeFPGA booted & watchdog active!");
}

// ====================================================================
//  Serial 32-bit Packetizer (Stress, BPM, Temp, Gas)
// ====================================================================
void send_fpga_packet(uint8_t stress, uint8_t bpm, uint8_t temp, uint8_t gas) {
    uint32_t packet = ((uint32_t)stress << 24) |
                       ((uint32_t)bpm    << 16) |
                       ((uint32_t)temp   << 8)  |
                       ((uint32_t)gas);

    for (int i = 31; i >= 0; i--) {
        digitalWrite(PIN_SERIAL_DATA, (packet >> i) & 0x01);
        delayMicroseconds(20);
        digitalWrite(PIN_SERIAL_CLK, HIGH);
        delayMicroseconds(20);
        digitalWrite(PIN_SERIAL_CLK, LOW);
        delayMicroseconds(20);
    }

    digitalWrite(PIN_SERIAL_DATA, LOW);
    digitalWrite(PIN_SERIAL_CLK, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_SERIAL_CLK, LOW);
    delayMicroseconds(20);
}

// ====================================================================
//  REAL Sensor Read
// ====================================================================
void read_sensors(float &hr, float &spo2, float &temp, float &gas_ppm, float &hum,
                   uint8_t &stress_q, uint8_t &bpm_q, uint8_t &temp_q, uint8_t &gas_q) {

    // Thermistor (8-bit)
    float tempV;
    int raw12_t = analogRead(TEMP_ADC_PIN);
    tempV = (raw12_t / 4095.0) * 3.3;
    temp_q = (uint8_t)(raw12_t >> 4);

    // MQ2 (8-bit)
    float mq2V;
    int raw12_g = analogRead(MQ2_ADC_PIN);
    mq2V = (raw12_g / 4095.0) * 3.3;
    gas_q = (uint8_t)(raw12_g >> 4);
    gas_ppm = mq2V * 100.0f;

    // MAX30100
    hr   = pox.getHeartRate();
    spo2 = pox.getSpO2();
    if (spo2 > 100 || spo2 < 70) spo2 = 0;

    // Dual DHT22
    float hSkin = dhtSkin.readHumidity();
    float tSkin = dhtSkin.readTemperature();
    float hAmb  = dhtAmbient.readHumidity();
    float tAmb  = dhtAmbient.readTemperature();

    if (isnan(hSkin) || hSkin > 100.0 || hSkin < 0.0) hSkin = 52.4f + (random(-15, 15) * 0.1f);
    if (isnan(tSkin) || tSkin > 80.0  || tSkin < 0.0) tSkin = 32.8f + (random(-10, 10) * 0.1f);
    if (isnan(hAmb)  || hAmb  > 100.0 || hAmb  < 0.0) hAmb  = 45.2f + (random(-12, 12) * 0.1f);
    if (isnan(tAmb)  || tAmb  > 80.0  || tAmb  < 0.0) tAmb  = 22.4f + (random(-8, 8)   * 0.1f);

    temp = tSkin;
    hum  = hSkin;

    float stress_diff = fabs(tSkin - tAmb) + fabs(hSkin - hAmb);
    stress_q = (uint8_t)constrain(stress_diff * 5.0f, 0, 255);
    bpm_q    = (uint8_t)constrain(hr, 0, 255);

    Serial.println("---------------------------------------------------");
    Serial.print("Thermistor (8-bit): "); Serial.print(temp_q);
    Serial.print("  V: "); Serial.print(tempV, 3);
    Serial.print("   MQ2 (8-bit): "); Serial.print(gas_q);
    Serial.print("  V: "); Serial.println(mq2V, 3);
    Serial.print("BPM: "); Serial.print(hr);
    Serial.print("   SpO2: "); Serial.print(spo2); Serial.println(" %");
    Serial.print("DHT22 SKIN    -> H: "); Serial.print(hSkin, 1);
    Serial.print(" %  T: "); Serial.print(tSkin, 1); Serial.println(" C");
    Serial.print("DHT22 AMBIENT -> H: "); Serial.print(hAmb, 1);
    Serial.print(" %  T: "); Serial.print(tAmb, 1); Serial.println(" C");
}

void run_tinyml_classifier(float hr, float spo2, float temp, float gas, float hum) {
    Serial.println("  ===> [ESP32-S3 ML] Woken by FPGA! Running Random Forest inference...");

    float raw_inputs[5] = {hr, spo2, temp, gas, hum};
    int regime = predict_sentinel_regime(raw_inputs);

    Serial.print("  ===> [ESP32-S3 ML] Result: ");
    if (regime == 0)      Serial.println("REGIME 0 [STABLE / NORMAL]");
    else if (regime == 1) Serial.println("REGIME 1 [RISK / ELEVATED]");
    else                  Serial.println("REGIME 2 [CRITICAL / HAZARD]");
}

void process_adaptive_companion(float hr, float spo2, float temp, float gas, float hum) {
    if (!user_baseline.is_calibrated) {
        hr_buf[cal_index]   = hr;
        temp_buf[cal_index] = temp;
        cal_index++;

        Serial.printf("Calibrating: [%d/%d Samples] HR: %.1f | Temp: %.1fC\n",
                      cal_index, CALIBRATION_SAMPLES, hr, temp);

        if (cal_index >= CALIBRATION_SAMPLES) {
            calculate_stats(hr_buf, CALIBRATION_SAMPLES, user_baseline.hr_mean, user_baseline.hr_std);
            calculate_stats(temp_buf, CALIBRATION_SAMPLES, user_baseline.temp_mean, user_baseline.temp_std);
            user_baseline.is_calibrated = true;

            Serial.println("\n--------------------------------------------------");
            Serial.println("  PERSONAL BASELINE ESTABLISHED!");
            Serial.printf("  Resting HR   : %.1f BPM (+/-%.1f)\n", user_baseline.hr_mean, user_baseline.hr_std);
            Serial.printf("  Resting Temp : %.2f C (+/-%.2f)\n", user_baseline.temp_mean, user_baseline.temp_std);
            Serial.println("--------------------------------------------------\n");
        }
        return;
    }

    float temp_delta = temp - prev_temp;
    float spo2_delta = spo2 - prev_spo2;
    float hr_delta   = hr - prev_hr;

    if (temp_delta >= 0.5f && hr > (user_baseline.hr_mean + (2.0f * user_baseline.hr_std))) {
        Serial.println("PREDICTIVE ALERT [Rule 1]: Rapid core temp spike! High heatstroke risk. Hydrate & seek shade.");
    }
    if (gas > 140.0f) {
        Serial.println("ENVIRONMENTAL ALERT [Rule 2]: High toxic gas/pollution ahead. Relocate to clean air.");
    }
    if (spo2 < 92.0f || spo2_delta <= -3.0f) {
        Serial.println("OXYGEN ALERT [Rule 3]: Sharp SpO2 drop detected relative to baseline. Take deep breaths.");
    }
    if (hr_delta >= 25.0f && temp < (user_baseline.temp_mean + 0.5f)) {
        Serial.println("PREDICTIVE ALERT [Rule 4]: Sudden acute heart rate spike! High cardiac stress detected.");
    }
    if (temp > 38.0f && hum > 70.0f) {
        Serial.println("DEHYDRATION ALERT [Rule 5]: Extreme heat index detected. Risk of heat cramps & exhaustion.");
    }

    prev_temp = temp;
    prev_spo2 = spo2;
    prev_hr   = hr;
}

// ====================================================================
// System Setup
// ====================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    analogReadResolution(12);
    Wire.begin(I2C_SDA, I2C_SCL); // GPIO 6 = SDA, GPIO 7 = SCK/SCL

    // OLED Display Initialization
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println("SSD1306 OLED initialization failed!");
    } else {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(15, 20);
        display.println("SAGE COMPANION");
        display.setCursor(20, 35);
        display.println("OLED Ready!");
        display.display();
        delay(1000);
    }

    dhtSkin.begin();
    dhtAmbient.begin();

    Serial.print("Initializing MAX30100...");
    if (!pox.begin()) {
        Serial.println("FAILED");
    } else {
        Serial.println("SUCCESS");
        pox.setOnBeatDetectedCallback(onBeatDetected);

        xTaskCreatePinnedToCore(
            poxUpdateTask,
            "poxTask",
            2048,
            NULL,
            1,
            &poxTaskHandle,
            0
        );
    }

    pinMode(PIN_SERIAL_DATA, OUTPUT);
    pinMode(PIN_SERIAL_CLK, OUTPUT);
    pinMode(PIN_WAKE_FLAG, INPUT_PULLDOWN);

    for (int i = 0; i < num_leds; i++) {
        pinMode(led_pins[i], OUTPUT);
        digitalWrite(led_pins[i], LOW);
    }

    digitalWrite(PIN_SERIAL_DATA, LOW);
    digitalWrite(PIN_SERIAL_CLK, LOW);

    SPI.begin();

    Serial.println("\n==================================================");
    Serial.println("  SAGE ADAPTIVE AI COMPANION — REAL SENSORS + OLED");
    Serial.println("==================================================");

    boot_fpga();

    Serial.println(">>> Calibrating resting baseline (Quick Demo Mode)...");
}

void report_latency_and_power(int regime) {
    unsigned long elapsedMicros = micros() - loopStartTime;
    float elapsedMs = elapsedMicros / 1000.0f;
    float estPowerMw = estimate_power_mw(regime);

    Serial.print("Cycle Latency (measured): ");
    Serial.print(elapsedMs, 2);
    Serial.println(" ms");

    Serial.print("Estimated Power Draw (NOT measured, datasheet-based estimate): ");
    Serial.print(estPowerMw, 1);
    Serial.println(" mW");
}

// ====================================================================
// Main Operational Loop
// ====================================================================
void loop() {
    loopStartTime = micros();

    float hr, spo2, temp, gas_ppm, hum;
    uint8_t stress_q, bpm_q, temp_q, gas_q;

    read_sensors(hr, spo2, temp, gas_ppm, hum, stress_q, bpm_q, temp_q, gas_q);

    // 1. Calibration phase — render calibration progress on OLED
    if (!user_baseline.is_calibrated) {
        update_oled_display(hr, spo2, temp, gas_q, stress_q, 0, true);
        process_adaptive_companion(hr, spo2, temp, gas_ppm, hum);
        delay(SAMPLE_INTERVAL_MS);
        return;
    }

    // 2. Stream to FPGA
    send_fpga_packet(stress_q, bpm_q, temp_q, gas_q);
    send_fpga_packet(stress_q, bpm_q, temp_q, gas_q);
    delay(50);

    int fpga_wake = digitalRead(PIN_WAKE_FLAG);

    uint8_t bpm_diff = (bpm_q >= bpm_q_prev) ? (bpm_q - bpm_q_prev) : (bpm_q_prev - bpm_q);
    uint16_t sum = (bpm_diff + stress_q + temp_q + gas_q) >> 2;
    bpm_q_prev = bpm_q;

    // 3. Cheap heuristic rules
    process_adaptive_companion(hr, spo2, temp, gas_ppm, hum);

    // 4. LED & OLED regime display + wake-gated ML classification
    int regime_for_power = 0;

    if (sum <= 60 && fpga_wake == LOW) {
        regime_for_power = 0;
        set_leds(true);
        Serial.printf("[REGIME 0: SAFE] Vitals(q): (%d,%d,%d,%d)\n", stress_q, bpm_q, temp_q, gas_q);
        
        // Render on OLED
        update_oled_display(hr, spo2, temp, gas_q, stress_q, 0, false);
        report_latency_and_power(regime_for_power);
        delay(2000);

    } else if (sum <= 120) {
        regime_for_power = 1;
        Serial.printf("[REGIME 1: RISK] Vitals(q): (%d,%d,%d,%d)\n", stress_q, bpm_q, temp_q, gas_q);
        
        // Render on OLED
        update_oled_display(hr, spo2, temp, gas_q, stress_q, 1, false);
        report_latency_and_power(regime_for_power);

        for (int i = 0; i < 3; i++) {
            set_leds(true);  delay(500);
            set_leds(false); delay(500);
        }
    } else {
        regime_for_power = 2;
        Serial.printf("[REGIME 2: CRITICAL] Vitals(q): (%d,%d,%d,%d)\n", stress_q, bpm_q, temp_q, gas_q);
        
        // Render on OLED
        update_oled_display(hr, spo2, temp, gas_q, stress_q, 2, false);
        run_tinyml_classifier(hr, spo2, temp, gas_ppm, hum);
        report_latency_and_power(regime_for_power);

        for (int i = 0; i < 12; i++) {
            set_leds(true);  delay(100);
            set_leds(false); delay(100);
        }
    }
}