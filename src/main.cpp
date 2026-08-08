/**
 * ESP32 ELM327 Control Center & Ultra-Fluid LED Shift Light System (4 Canales Simultáneos)
 * 
 * Arquitectura Dual-Core FreeRTOS:
 * - Core 0: Tarea BLE (NimBLE Client para ELM327 + NimBLE Server para Web Bluetooth)
 * - Core 1: Tarea FastLED (Renderizado 4 Canales GPIO 33, 32, 25, 26 a 60 FPS)
 */

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include "config.h"

// ==========================================
// VARIABLES DE ESTADO Y SINCRONIZACIÓN MUTEX
// ==========================================
static SemaphoreHandle_t g_dataMutex = NULL;

// Variables compartidas protegidas por Mutex
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

// Helper para replicar color en los 4 canales simultáneos
inline void setPixel4Strips(int index, CRGB color) {
    if (index >= 0 && index < NUM_LEDS) {
        g_leds33[index] = color;
        g_leds32[index] = color;
        g_leds25[index] = color;
        g_leds26[index] = color;
    }
}

inline void clear4Strips() {
    for (int i = 0; i < NUM_LEDS; i++) {
        setPixel4Strips(i, CRGB::Black);
    }
}

// UUIDs BLE del ELM327
static const char* ELM_SERVICES[] = {
    "0000fff0-0000-1000-8000-00805f9b34fb",
    "0000ffe0-0000-1000-8000-00805f9b34fb",
    "000018f0-0000-1000-8000-00805f9b34fb"
};

// Punteros BLE Cliente (ELM327)
static NimBLEClient* pElmClient = nullptr;
static NimBLERemoteCharacteristic* pElmTxChar = nullptr;
static NimBLERemoteCharacteristic* pElmRxChar = nullptr;

// Punteros BLE Servidor (Web App)
static NimBLEServer* pWebPointer = nullptr;
static NimBLECharacteristic* pCharRPM = nullptr;

// Máquina de estados BLE Cliente
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

// Declaración de Funciones
void taskBLE(void* pvParameters);
void taskLED(void* pvParameters);

void renderWelcomeAnimation(uint8_t step);
void renderRpmShiftLight(float rpm);
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

// Callback de notificación de datos del ELM327
void elmNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length == 0) return;
    
    char buffer[128];
    size_t copyLen = length < 127 ? length : 127;
    memcpy(buffer, pData, copyLen);
    buffer[copyLen] = '\0';

    g_lastElmDataTime = millis();
    parseOBDResponse(buffer);
}

// Callbacks de escaneo BLE
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
        Serial.println("[BLE Server] Navegador Web conectado a la app.");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        g_webConnected = false;
        xSemaphoreGive(g_dataMutex);
        Serial.println("[BLE Server] Navegador Web desconectado. Reiniciando publicidad...");
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
    Serial.println("  ESP32 DUAL-CORE ELM327 DIESEL CONTROL CENTER");
    Serial.println("  4 Canales Simultáneos (GPIO 33, 32, 25, 26) - 83 LEDs c/u");
    Serial.println("=======================================================\n");

    g_dataMutex = xSemaphoreCreateMutex();

    // Registrar los 4 canales independientes de 83 LEDs cada uno
    FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(g_leds33, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(g_leds32, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_3, COLOR_ORDER>(g_leds25, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN_4, COLOR_ORDER>(g_leds26, NUM_LEDS).setCorrection(TypicalLEDStrip);

    FastLED.setBrightness(DEFAULT_BRIGHTNESS);
    clear4Strips();
    FastLED.show();

    // Inicializar Motor NimBLE Dual
    NimBLEDevice::init("ESP32_OBD_ShiftLight");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    // Servidor BLE
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

    Serial.println("[System] Servidor BLE activo en 4 salidas de tiras LED.");

    // Cliente BLE (ELM327)
    pElmClient = NimBLEDevice::createClient();
    pElmClient->setClientCallbacks(new ElmClientCallbacks(), false);

    // Tareas Dual-Core
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
                    Serial.println("[BLE Task] Escaneando adaptadores ELM327 BLE...");
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
                    Serial.println("[BLE Task] Conectando a ELM327...");
                    if (connectToElmServer(g_targetElmDevice)) {
                        g_elmState = BLE_STATE_INIT_ELM;
                        stateTimer = now;
                    } else {
                        Serial.println("[BLE Task] Conexión fallida. Reintentando escaneo...");
                        g_targetElmDevice = nullptr;
                        g_elmState = BLE_STATE_RETRY_WAIT;
                        stateTimer = now;
                    }
                }
                break;
            }

            case BLE_STATE_INIT_ELM: {
                if (now - stateTimer > 300) {
                    Serial.println("[BLE Task] Inicializando AT ELM327...");
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
                    Serial.println("[BLE Task] Timeout ELM327. Reiniciando...");
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
// TAREA CORE 1: RENDERIZADOR FASTLED (60 FPS EN 4 CANALES)
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

        smoothedRPM += ((float)targetRPM - smoothedRPM) * 0.12f;

        FastLED.setBrightness(brightness);

        switch (mode) {
            case MODE_WELCOME: {
                renderWelcomeAnimation(welcomeStep);
                welcomeStep++;
                if (welcomeStep >= (NUM_LEDS / 2) + 1) {
                    welcomeStep = 0;
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_currentMode = MODE_RPM_SHIFTLIGHT;
                    xSemaphoreGive(g_dataMutex);
                }
                break;
            }

            case MODE_RPM_SHIFTLIGHT:
                renderRpmShiftLight(smoothedRPM);
                break;

            case MODE_STATIC_COLOR:
                renderStaticColor(customColor);
                break;

            case MODE_RAINBOW:
                renderRainbow();
                break;

            case MODE_BREATHING:
                renderBreathing(customColor);
                break;

            case MODE_STROBE:
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
// RENDERIZADORES REPLICADOS EN 4 CANALES
// ==========================================

// ANIMACIÓN DE BIENVENIDA: Edge-to-Center en 4 tiras
void renderWelcomeAnimation(uint8_t step) {
    clear4Strips();
    int half = NUM_LEDS / 2;
    CRGB welcomeColor = CRGB(0, 220, 255);

    for (int i = 0; i <= step && i <= half; i++) {
        int leftIdx = i;
        int rightIdx = NUM_LEDS - 1 - i;

        setPixel4Strips(leftIdx, welcomeColor);
        setPixel4Strips(rightIdx, welcomeColor);
    }
}

// TACÓMETRO RPM OPEL CORSA DIESEL (Sub-pixel Liquid en 4 tiras)
void renderRpmShiftLight(float rpm) {
    clear4Strips();

    if (rpm < RPM_IDLE) {
        setPixel4Strips(0, CRGB(0, 40, 80));
        return;
    }

    if (rpm >= RPM_REDLINE) {
        static bool flashToggle = false;
        flashToggle = !flashToggle;
        CRGB alertColor = flashToggle ? CRGB::Red : CRGB::White;
        for (int i = 0; i < NUM_LEDS; i++) setPixel4Strips(i, alertColor);
        return;
    }

    float pct = (rpm - (float)RPM_IDLE) / ((float)RPM_REDLINE - (float)RPM_IDLE);
    pct = constrain(pct, 0.0f, 1.0f);

    float numLedsToLit = pct * (float)NUM_LEDS;
    int fullLeds = (int)numLedsToLit;
    float fractionalPart = numLedsToLit - fullLeds;

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < fullLeds || (i == fullLeds && fractionalPart > 0.05f)) {
            float ledPct = (float)i / (float)NUM_LEDS;
            CRGB color;

            if (ledPct < 0.35f) {
                color = blend(CRGB(0, 180, 255), CRGB(0, 255, 60), (uint8_t)(ledPct * (255.0f / 0.35f)));
            } else if (ledPct < 0.75f) {
                color = blend(CRGB(0, 255, 60), CRGB(255, 140, 0), (uint8_t)((ledPct - 0.35f) * (255.0f / 0.40f)));
            } else {
                color = blend(CRGB(255, 140, 0), CRGB(255, 0, 0), (uint8_t)((ledPct - 0.75f) * (255.0f / 0.25f)));
            }

            if (i == fullLeds) {
                color.nscale8_video((uint8_t)(fractionalPart * 255.0f));
            }

            setPixel4Strips(i, color);
        }
    }
}

void renderStaticColor(CRGB color) {
    for (int i = 0; i < NUM_LEDS; i++) setPixel4Strips(i, color);
}

void renderRainbow() {
    static uint8_t hue = 0;
    hue += 1;
    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB col = CHSV(hue + (i * 3), 255, 255);
        setPixel4Strips(i, col);
    }
}

void renderBreathing(CRGB color) {
    uint8_t val = beatsin8(14, 30, 255);
    CRGB col = color;
    col.nscale8_video(val);
    for (int i = 0; i < NUM_LEDS; i++) setPixel4Strips(i, col);
}

void renderStrobe() {
    static uint32_t lastStrobe = 0;
    static bool toggle = false;
    if (millis() - lastStrobe >= 70) {
        lastStrobe = millis();
        toggle = !toggle;
    }
    CRGB col = toggle ? CRGB::White : CRGB::Black;
    for (int i = 0; i < NUM_LEDS; i++) setPixel4Strips(i, col);
}
