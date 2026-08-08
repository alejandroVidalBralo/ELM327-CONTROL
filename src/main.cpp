/**
 * ESP32 ELM327 Control Center & Ambient Footwell LED System (4 Canales Bajo Asientos)
 * 
 * Iluminación Ambiental de Suelo / Bajo Asientos:
 * - Toda la tira (332 LEDs en 4 canales GPIO 33, 32, 25, 26) se ilumina completa.
 * - Transición suavizada 60 FPS de COLOR e INTENSIDAD/BRILLO en función de las RPM del Corsa Diésel.
 */

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include "config.h"

// ==========================================
// VARIABLES DE ESTADO Y SINCRONIZACIÓN MUTEX
// ==========================================
static SemaphoreHandle_t g_dataMutex = NULL;

static uint16_t g_targetRPM = 0;
static LedMode  g_currentMode = MODE_WELCOME;
static CRGB     g_customColor = CRGB(0, 150, 255);
static uint8_t  g_brightness = DEFAULT_BRIGHTNESS;
static bool     g_elmConnected = false;
static bool     g_webConnected = false;

// Buffers para los 4 canales de salida independientes (83 LEDs por tira)
static CRGB g_leds33[NUM_LEDS];
static CRGB g_leds32[NUM_LEDS];
static CRGB g_leds25[NUM_LEDS];
static CRGB g_leds26[NUM_LEDS];

// Helper para pintar toda la iluminación bajo los asientos en los 4 canales
inline void setAll4StripsColor(CRGB color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        g_leds33[i] = color;
        g_leds32[i] = color;
        g_leds25[i] = color;
        g_leds26[i] = color;
    }
}

inline void clear4Strips() {
    setAll4StripsColor(CRGB::Black);
}

// UUIDs BLE del ELM327
static const char* ELM_SERVICES[] = {
    "0000fff0-0000-1000-8000-00805f9b34fb",
    "0000ffe0-0000-1000-8000-00805f9b34fb",
    "000018f0-0000-1000-8000-00805f9b34fb"
};

static NimBLEClient* pElmClient = nullptr;
static NimBLERemoteCharacteristic* pElmTxChar = nullptr;
static NimBLERemoteCharacteristic* pElmRxChar = nullptr;

static NimBLEServer* pWebPointer = nullptr;
static NimBLECharacteristic* pCharRPM = nullptr;

enum ElmBleState {
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING,
    BLE_STATE_INIT_ELM,
    BLE_STATE_POLLING,
    BLE_STATE_RETRY_WAIT
};

static ElmBleState g_elmState = BLE_STATE_SCANNING;
static uint32_t g_lastElmDataTime = 0;
static NimBLEAdvertisedDevice* g_targetElmDevice = nullptr;

// Declaraciones
void taskBLE(void* pvParameters);
void taskLED(void* pvParameters);

void renderWelcomeAnimation(uint8_t step);
void renderFootwellRpmAmbient(float rpm, uint8_t baseBrightness);
void renderStaticColor(CRGB color);
void renderRainbow();
void renderBreathing(CRGB color);
void renderStrobe();

void parseOBDResponse(const char* data);
bool connectToElmServer(NimBLEAdvertisedDevice* advertisedDevice);

// ==========================================
// CALLBACKS BLE CLIENTE (ELM327)
// ==========================================
class ElmClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        Serial.println("[BLE Client] Conectado físicamente al adaptador ELM327.");
    }

    void onDisconnect(NimBLEClient* pClient) override {
        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        g_elmConnected = false;
        xSemaphoreGive(g_dataMutex);

        g_elmState = BLE_STATE_RETRY_WAIT;
        Serial.println("[BLE Client] Desconectado del ELM327. Reintentando...");
    }
};

void elmNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length == 0) return;
    
    char buffer[128];
    size_t copyLen = length < 127 ? length : 127;
    memcpy(buffer, pData, copyLen);
    buffer[copyLen] = '\0';

    g_lastElmDataTime = millis();
    parseOBDResponse(buffer);
}

class ElmScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
        if (g_targetElmDevice != nullptr) return;

        for (const char* uuidStr : ELM_SERVICES) {
            if (advertisedDevice->isAdvertisingService(NimBLEUUID(uuidStr))) {
                Serial.printf("[BLE Scan] Encontrado ELM327: %s [%s]\n", 
                              advertisedDevice->getName().c_str(), 
                              advertisedDevice->getAddress().toString().c_str());
                g_targetElmDevice = advertisedDevice;
                NimBLEDevice::getScan()->stop();
                return;
            }
        }

        std::string name = advertisedDevice->getName();
        if (name.find("OBD") != std::string::npos || name.find("ELM") != std::string::npos || 
            name.find("V-LINK") != std::string::npos || name.find("Viecar") != std::string::npos) {
            Serial.printf("[BLE Scan] Encontrado por Nombre: %s [%s]\n", 
                          name.c_str(), advertisedDevice->getAddress().toString().c_str());
            g_targetElmDevice = advertisedDevice;
            NimBLEDevice::getScan()->stop();
        }
    }
};

// ==========================================
// CALLBACKS BLE SERVIDOR (WEB CONTROL)
// ==========================================
class WebServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        g_webConnected = true;
        xSemaphoreGive(g_dataMutex);
        Serial.println("[BLE Server] Navegador Web conectado.");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        g_webConnected = false;
        xSemaphoreGive(g_dataMutex);
        NimBLEDevice::startAdvertising();
    }
};

class WebModeCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.length() >= 1) {
            uint8_t modeVal = val[0];
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            if (modeVal <= MODE_OFF) {
                g_currentMode = (LedMode)modeVal;
                Serial.printf("[Web Command] Cambio de Modo: %d\n", g_currentMode);
            }
            xSemaphoreGive(g_dataMutex);
        }
    }
};

class WebColorCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.length() >= 3) {
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            g_customColor = CRGB(val[0], val[1], val[2]);
            g_currentMode = MODE_STATIC_COLOR;
            xSemaphoreGive(g_dataMutex);
            Serial.printf("[Web Command] Color RGB: (%d, %d, %d)\n", val[0], val[1], val[2]);
        }
    }
};

class WebBrightCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.length() >= 1) {
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            g_brightness = val[0];
            xSemaphoreGive(g_dataMutex);
            Serial.printf("[Web Command] Brillo: %d\n", g_brightness);
        }
    }
};

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n=======================================================");
    Serial.println("  ESP32 ILUMINACIÓN AMBIENTAL DE SUELO (BAJO ASIENTOS)");
    Serial.println("  4 Canales (GPIO 33, 32, 25, 26) - 83 LEDs c/u");
    Serial.println("  Transición de Color e Intensidad por RPM (Corsa Diésel)");
    Serial.println("=======================================================\n");

    g_dataMutex = xSemaphoreCreateMutex();

    FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(g_leds33, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(g_leds32, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_3, COLOR_ORDER>(g_leds25, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_4, COLOR_ORDER>(g_leds26, NUM_LEDS).setCorrection(TypicalLEDStrip);

    FastLED.setBrightness(DEFAULT_BRIGHTNESS);
    clear4Strips();
    FastLED.show();

    NimBLEDevice::init("ESP32_OBD_ShiftLight");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new WebServerCallbacks());
    NimBLEService* pService = pServer->createService(ESP32_SERVICE_UUID);

    NimBLECharacteristic* pCharMode = pService->createCharacteristic(ESP32_CHAR_MODE, NIMBLE_PROPERTY::WRITE);
    pCharMode->setCallbacks(new WebModeCallback());

    NimBLECharacteristic* pCharColor = pService->createCharacteristic(ESP32_CHAR_COLOR, NIMBLE_PROPERTY::WRITE);
    pCharColor->setCallbacks(new WebColorCallback());

    NimBLECharacteristic* pCharBright = pService->createCharacteristic(ESP32_CHAR_BRIGHT, NIMBLE_PROPERTY::WRITE);
    pCharBright->setCallbacks(new WebBrightCallback());

    pCharRPM = pService->createCharacteristic(ESP32_CHAR_RPM, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    pService->start();
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(ESP32_SERVICE_UUID);
    pAdvertising->start();

    Serial.println("[System] Servidor BLE activo para control de luces.");

    pElmClient = NimBLEDevice::createClient();
    pElmClient->setClientCallbacks(new ElmClientCallbacks(), false);

    xTaskCreatePinnedToCore(taskBLE, "TaskBLE", 8192, NULL, 2, NULL, CORE_BLE);
    xTaskCreatePinnedToCore(taskLED, "TaskLED", 6144, NULL, 3, NULL, CORE_LED);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==========================================
// TAREA CORE 0: GESTOR BLE CLIENTE & COMUNICACIÓN
// ==========================================
void taskBLE(void* pvParameters) {
    uint32_t lastPollTime = 0;
    uint32_t stateTimer = 0;

    for (;;) {
        uint32_t now = millis();

        switch (g_elmState) {
            case BLE_STATE_SCANNING: {
                if (g_targetElmDevice == nullptr) {
                    NimBLEScan* pScan = NimBLEDevice::getScan();
                    pScan->setAdvertisedDeviceCallbacks(new ElmScanCallbacks());
                    pScan->setInterval(45);
                    pScan->setWindow(15);
                    pScan->setActiveScan(true);
                    pScan->start(4, false);
                } else {
                    g_elmState = BLE_STATE_CONNECTING;
                }
                break;
            }

            case BLE_STATE_CONNECTING: {
                if (g_targetElmDevice != nullptr) {
                    if (connectToElmServer(g_targetElmDevice)) {
                        g_elmState = BLE_STATE_INIT_ELM;
                        stateTimer = now;
                    } else {
                        g_targetElmDevice = nullptr;
                        g_elmState = BLE_STATE_RETRY_WAIT;
                        stateTimer = now;
                    }
                }
                break;
            }

            case BLE_STATE_INIT_ELM: {
                if (now - stateTimer > 300) {
                    if (pElmTxChar) {
                        pElmTxChar->writeValue("AT Z\r", false);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        pElmTxChar->writeValue("AT E0\r", false);
                        vTaskDelay(pdMS_TO_TICKS(150));
                        pElmTxChar->writeValue("AT SP 0\r", false);
                        vTaskDelay(pdMS_TO_TICKS(150));
                    }
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_elmConnected = true;
                    xSemaphoreGive(g_dataMutex);

                    g_elmState = BLE_STATE_POLLING;
                    g_lastElmDataTime = now;
                }
                break;
            }

            case BLE_STATE_POLLING: {
                if (now - lastPollTime >= 70) {
                    lastPollTime = now;
                    if (pElmTxChar && pElmClient && pElmClient->isConnected()) {
                        pElmTxChar->writeValue("01 0C\r", false);
                    }
                }

                if (now - g_lastElmDataTime > 4000) {
                    if (pElmClient && pElmClient->isConnected()) {
                        pElmClient->disconnect();
                    }
                    g_elmState = BLE_STATE_RETRY_WAIT;
                    stateTimer = now;
                }
                break;
            }

            case BLE_STATE_RETRY_WAIT: {
                if (now - stateTimer >= 2000) {
                    g_targetElmDevice = nullptr;
                    g_elmState = BLE_STATE_SCANNING;
                }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

bool connectToElmServer(NimBLEAdvertisedDevice* advertisedDevice) {
    if (!pElmClient->connect(advertisedDevice)) return false;

    for (const char* uuidStr : ELM_SERVICES) {
        NimBLERemoteService* pService = pElmClient->getService(uuidStr);
        if (pService != nullptr) {
            pElmRxChar = pService->getCharacteristic(NimBLEUUID("0000fff1-0000-1000-8000-00805f9b34fb"));
            if (!pElmRxChar) pElmRxChar = pService->getCharacteristic(NimBLEUUID("0000ffe1-0000-1000-8000-00805f9b34fb"));
            if (!pElmRxChar) pElmRxChar = pService->getCharacteristic(NimBLEUUID("00002af0-0000-1000-8000-00805f9b34fb"));

            pElmTxChar = pService->getCharacteristic(NimBLEUUID("0000fff2-0000-1000-8000-00805f9b34fb"));
            if (!pElmTxChar) pElmTxChar = pElmRxChar;

            if (pElmRxChar && pElmRxChar->canNotify()) {
                pElmRxChar->subscribe(true, elmNotifyCallback);
            }
            return true;
        }
    }
    return false;
}

void parseOBDResponse(const char* data) {
    if (strstr(data, "41 0C") != nullptr || strstr(data, "410C") != nullptr) {
        const char* p = strstr(data, "0C");
        if (!p) p = strstr(data, "0c");
        if (p) {
            int a = 0, b = 0;
            if (sscanf(p, "%*s %x %x", &a, &b) >= 2) {
                uint16_t rpm = (uint16_t)(((a * 256) + b) / 4);
                
                xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                g_targetRPM = rpm;
                
                if (g_webConnected && pCharRPM) {
                    uint8_t rpmBytes[2] = { (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF) };
                    pCharRPM->setValue(rpmBytes, 2);
                    pCharRPM->notify();
                }
                xSemaphoreGive(g_dataMutex);
            }
        }
    }
}

// ==========================================
// TAREA CORE 1: RENDERIZADOR ILUMINACIÓN AMBIENTAL (60 FPS)
// ==========================================
void taskLED(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(16); // ~60 FPS

    float smoothedRPM = 0.0f;
    uint8_t welcomeStep = 0;

    for (;;) {
        uint16_t targetRPM;
        LedMode mode;
        CRGB customColor;
        uint8_t brightness;

        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        targetRPM = g_targetRPM;
        mode = g_currentMode;
        customColor = g_customColor;
        brightness = g_brightness;
        xSemaphoreGive(g_dataMutex);

        // Filtro suave para transiciones ambientales de fluido
        smoothedRPM += ((float)targetRPM - smoothedRPM) * 0.10f;

        switch (mode) {
            case MODE_WELCOME: {
                FastLED.setBrightness(brightness);
                renderWelcomeAnimation(welcomeStep);
                welcomeStep++;
                if (welcomeStep >= (NUM_LEDS / 2) + 1) {
                    welcomeStep = 0;
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_currentMode = MODE_RPM_SHIFTLIGHT; // Entrar a modo ambiental RPM
                    xSemaphoreGive(g_dataMutex);
                }
                break;
            }

            case MODE_RPM_SHIFTLIGHT:
                renderFootwellRpmAmbient(smoothedRPM, brightness);
                break;

            case MODE_STATIC_COLOR:
                FastLED.setBrightness(brightness);
                renderStaticColor(customColor);
                break;

            case MODE_RAINBOW:
                FastLED.setBrightness(brightness);
                renderRainbow();
                break;

            case MODE_BREATHING:
                FastLED.setBrightness(brightness);
                renderBreathing(customColor);
                break;

            case MODE_STROBE:
                FastLED.setBrightness(brightness);
                renderStrobe();
                break;

            case MODE_OFF:
            default:
                clear4Strips();
                break;
        }

        FastLED.show();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// RENDERIZADOR AMBIENTAL DE SUELO (BAJO ASIENTOS)
// ==========================================

// ANIMACIÓN DE BIENVENIDA: Llenado desde los bordes hacia el centro (Edge-to-Center)
void renderWelcomeAnimation(uint8_t step) {
    clear4Strips();
    int half = NUM_LEDS / 2;
    CRGB welcomeColor = CRGB(0, 220, 255);

    for (int i = 0; i <= step && i <= half; i++) {
        int leftIdx = i;
        int rightIdx = NUM_LEDS - 1 - i;

        for (int j = 0; j < NUM_LEDS; j++) {
            if (j == leftIdx || j == rightIdx) {
                g_leds33[j] = welcomeColor;
                g_leds32[j] = welcomeColor;
                g_leds25[j] = welcomeColor;
                g_leds26[j] = welcomeColor;
            }
        }
    }
}

// MODO AMBIENTAL BAJO ASIENTOS: Toda la tira encendida, cambiando COLOR e INTENSIDAD según RPM
void renderFootwellRpmAmbient(float rpm, uint8_t baseBrightness) {
    // 1. Estado Reposo / Motor Apagado / Ralentí (< 800 RPM)
    if (rpm < RPM_IDLE) {
        FastLED.setBrightness((uint8_t)(baseBrightness * 0.40f)); // Brillo tenue de ambiente
        setAll4StripsColor(CRGB(0, 80, 200)); // Azul / Cian suave de cortesía
        return;
    }

    // 2. Estado Aviso Corte (> 4300 RPM)
    if (rpm >= RPM_REDLINE) {
        FastLED.setBrightness(baseBrightness);
        static bool flashToggle = false;
        flashToggle = !flashToggle;
        CRGB alertColor = flashToggle ? CRGB::Red : CRGB::White;
        setAll4StripsColor(alertColor);
        return;
    }

    // 3. Porcentaje de Aceleración / RPM (800 -> 4300 RPM)
    float pct = (rpm - (float)RPM_IDLE) / ((float)RPM_REDLINE - (float)RPM_IDLE);
    pct = constrain(pct, 0.0f, 1.0f);

    // 4. Dinámica de Intensidad: El brillo aumenta gradualmente del 35% al 100% conforme aceleras
    float dynamicBrightPct = 0.35f + (pct * 0.65f);
    uint8_t currentBrightness = (uint8_t)(baseBrightness * dynamicBrightPct);
    FastLED.setBrightness(currentBrightness);

    // 5. Dinámica de Color: Toda la tira cambia de color fluidamente
    CRGB ambientColor;

    if (pct < 0.35f) {
        // Zona 1: Ralentí a Cruce (800 - 2000 RPM) -> Azul Cian ➔ Verde Esmeralda
        float subPct = pct / 0.35f;
        ambientColor = blend(CRGB(0, 180, 255), CRGB(0, 255, 60), (uint8_t)(subPct * 255.0f));
    } else if (pct < 0.75f) {
        // Zona 2: Zona de Par / Aceleración (2000 - 3500 RPM) -> Verde ➔ Naranja / Dorado
        float subPct = (pct - 0.35f) / 0.40f;
        ambientColor = blend(CRGB(0, 255, 60), CRGB(255, 120, 0), (uint8_t)(subPct * 255.0f));
    } else {
        // Zona 3: Zona Alta Potencia (3500 - 4300 RPM) -> Naranja ➔ Rojo Deporte
        float subPct = (pct - 0.75f) / 0.25f;
        ambientColor = blend(CRGB(255, 120, 0), CRGB(255, 0, 0), (uint8_t)(subPct * 255.0f));
    }

    // Aplicar el color a TODOS los LEDs de las 4 tiras
    setAll4StripsColor(ambientColor);
}

void renderStaticColor(CRGB color) {
    setAll4StripsColor(color);
}

void renderRainbow() {
    static uint8_t hue = 0;
    hue += 1;
    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB col = CHSV(hue + (i * 3), 255, 255);
        g_leds33[i] = col; g_leds32[i] = col;
        g_leds25[i] = col; g_leds26[i] = col;
    }
}

void renderBreathing(CRGB color) {
    uint8_t val = beatsin8(14, 30, 255);
    CRGB col = color;
    col.nscale8_video(val);
    setAll4StripsColor(col);
}

void renderStrobe() {
    static uint32_t lastStrobe = 0;
    static bool toggle = false;
    if (millis() - lastStrobe >= 70) {
        lastStrobe = millis();
        toggle = !toggle;
    }
    CRGB col = toggle ? CRGB::White : CRGB::Black;
    setAll4StripsColor(col);
}
