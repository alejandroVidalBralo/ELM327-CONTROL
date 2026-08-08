/**
 * ESP32 ELM327 Control Center & Ambient Footwell LED System (4 Canales Bajo Asientos)
 * 
 * Corrección de Estabilidad BLE & Control de Flujo:
 * 1. Eliminado 'AT Z' del bucle de reconexión (evita el reinicio del microprocesador del dongle ELM327).
 * 2. Control de flujo por respuesta (no se satura el buffer de notificaciones).
 * 3. Reseteo automático de g_targetRPM a 0 en caso de desconexión o timeout para evitar colores "atascados".
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

// Acumulador de fragmentos BLE
static String   g_bleRxAccumulator = "";
static bool     g_waitingResponse = false;

// Buffers para los 4 canales de salida independientes (83 LEDs por tira)
static CRGB g_leds33[NUM_LEDS];
static CRGB g_leds32[NUM_LEDS];
static CRGB g_leds25[NUM_LEDS];
static CRGB g_leds26[NUM_LEDS];

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
void renderFootwellRpmAmbient(float rpm, uint8_t baseBrightness, uint16_t targetRPM);
void renderStaticColor(CRGB color);
void renderRainbow();
void renderBreathing(CRGB color);
void renderStrobe();

void parseOBDResponse(const char* rawData);
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
        g_targetRPM = 0; // Reset a 0 para que no quede atascada la luz si cae la señal
        xSemaphoreGive(g_dataMutex);

        g_elmState = BLE_STATE_RETRY_WAIT;
        g_waitingResponse = false;
        Serial.println("[BLE Client] Desconectado del ELM327. Reseteando RPM a 0 y reintentando...");
    }
};

// Callback con acumulador de fragmentos BLE y liberación de flujo
void elmNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length == 0) return;

    g_lastElmDataTime = millis();

    for (size_t i = 0; i < length; i++) {
        char c = (char)pData[i];
        if (c == '>' || c == '\r' || c == '\n') {
            if (g_bleRxAccumulator.length() > 0) {
                g_bleRxAccumulator.trim();
                if (g_bleRxAccumulator.length() > 0) {
                    parseOBDResponse(g_bleRxAccumulator.c_str());
                }
                g_bleRxAccumulator = "";
            }
            g_waitingResponse = false; // Liberar bloqueo para enviar la siguiente consulta
        } else {
            g_bleRxAccumulator += c;
        }
    }
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
        Serial.println("[BLE Server] Navegador Web conectado a la app.");
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
    Serial.println("  SISTEMA OPTIMIZADO: SIN AT Z + CONTROL DE FLUJO BLE");
    Serial.println("  4 Canales (GPIO 33, 32, 25, 26) - 83 LEDs c/u");
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
                if (now - stateTimer > 200) {
                    if (pElmTxChar) {
                        // NO enviamos AT Z para evitar reiniciar el dongle ELM327
                        Serial.println("[BLE Task] Configurando AT E0 (Echo Off)...");
                        pElmTxChar->writeValue("AT E0\r", false);
                        vTaskDelay(pdMS_TO_TICKS(120));

                        Serial.println("[BLE Task] Configurando AT SP 0 (Auto Protocol)...");
                        pElmTxChar->writeValue("AT SP 0\r", false);
                        vTaskDelay(pdMS_TO_TICKS(120));
                    }
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_elmConnected = true;
                    xSemaphoreGive(g_dataMutex);

                    g_elmState = BLE_STATE_POLLING;
                    g_lastElmDataTime = now;
                    g_waitingResponse = false;
                }
                break;
            }

            case BLE_STATE_POLLING: {
                // Control de flujo: solo consulta si no hay respuesta pendiente o si han pasado >250ms
                if (!g_waitingResponse || (now - lastPollTime >= 250)) {
                    lastPollTime = now;
                    g_waitingResponse = true;
                    if (pElmTxChar && pElmClient && pElmClient->isConnected()) {
                        pElmTxChar->writeValue("01 0C\r", false);
                    }
                }

                // Detección de Timeout (4 segundos sin ninguna respuesta)
                if (now - g_lastElmDataTime > 4000) {
                    Serial.println("[BLE Task WARNING] Timeout de datos ELM327 (4s sin responder). Desconectando...");
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_elmConnected = false;
                    g_targetRPM = 0; // Decaer a 0 RPM
                    xSemaphoreGive(g_dataMutex);

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

// PARSEADOR OBD-II CON DEBUG SERIE DETALLADO
void parseOBDResponse(const char* rawData) {
    String data = String(rawData);
    data.toUpperCase();
    String cleanData = data;
    cleanData.replace(" ", "");

    Serial.printf("[OBD RX RAW] \"%s\"\n", rawData);

    int idx = cleanData.indexOf("410C");
    if (idx != -1 && (idx + 8 <= cleanData.length())) {
        String hexA = cleanData.substring(idx + 4, idx + 6);
        String hexB = cleanData.substring(idx + 6, idx + 8);

        char* endA;
        char* endB;
        long a = strtol(hexA.c_str(), &endA, 16);
        long b = strtol(hexB.c_str(), &endB, 16);

        if (*endA == '\0' && *endB == '\0') {
            uint16_t rpm = (uint16_t)(((a * 256) + b) / 4);

            if (rpm <= 6500) {
                xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                g_targetRPM = rpm;

                if (g_webConnected && pCharRPM) {
                    uint8_t rpmBytes[2] = { (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF) };
                    pCharRPM->setValue(rpmBytes, 2);
                    pCharRPM->notify();
                }
                xSemaphoreGive(g_dataMutex);

                Serial.printf("  --> [OBD PARSE OK] Hex: %s %s | RPM Extraídas: %u RPM\n", 
                              hexA.c_str(), hexB.c_str(), rpm);
            } else {
                Serial.printf("  --> [OBD PARSE WARN] RPM fuera de rango (>6500): %u RPM (Ignorado).\n", rpm);
            }
        } else {
            Serial.printf("  --> [OBD PARSE ERR] Fallo conversión hex '%s' '%s'\n", hexA.c_str(), hexB.c_str());
        }
    } else {
        if (cleanData.indexOf("SEARCHING") != -1) {
            Serial.println("  --> [OBD BUS] Buscando protocolo CAN...");
        } else if (cleanData.indexOf("NODATA") != -1) {
            Serial.println("  --> [OBD BUS] NO DATA recibido de la ECU.");
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

        // Decaimiento/Suavizado exponencial
        smoothedRPM += ((float)targetRPM - smoothedRPM) * 0.10f;

        switch (mode) {
            case MODE_WELCOME: {
                FastLED.setBrightness(brightness);
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
                renderFootwellRpmAmbient(smoothedRPM, brightness, targetRPM);
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

void renderWelcomeAnimation(uint8_t step) {
    clear4Strips();
    int half = NUM_LEDS / 2;
    // Color idéntico al inicio del modo tacómetro en ralentí para cero saltos de color
    CRGB welcomeColor = CRGB(0, 180, 255);

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

void renderFootwellRpmAmbient(float rpm, uint8_t baseBrightness, uint16_t targetRPM) {
    float pct = 0.0f;
    uint8_t currentBrightness = baseBrightness; // Brillo siempre al tope (255)
    CRGB ambientColor = CRGB::Black;

    // 1. Reposo / Ralentí (< 800 RPM) -> Mismo color exacto que el final de la bienvenida
    if (rpm < RPM_IDLE) {
        ambientColor = CRGB(0, 180, 255); // Cian / Azul de inicio del tacómetro
    }
    // 2. Corte (> 4300 RPM)
    else if (rpm >= RPM_REDLINE) {
        FastLED.setBrightness(baseBrightness);
        static bool flashToggle = false;
        flashToggle = !flashToggle;
        ambientColor = flashToggle ? CRGB::Red : CRGB::White;
        setAll4StripsColor(ambientColor);
        return;
    }
    // 3. Rango Normal RPM (800 -> 4300 RPM)
    else {
        pct = (rpm - (float)RPM_IDLE) / ((float)RPM_REDLINE - (float)RPM_IDLE);
        pct = constrain(pct, 0.0f, 1.0f);

        if (pct < 0.35f) {
            float subPct = pct / 0.35f;
            ambientColor = blend(CRGB(0, 180, 255), CRGB(0, 255, 60), (uint8_t)(subPct * 255.0f));
        } else if (pct < 0.75f) {
            float subPct = (pct - 0.35f) / 0.40f;
            ambientColor = blend(CRGB(0, 255, 60), CRGB(255, 120, 0), (uint8_t)(subPct * 255.0f));
        } else {
            float subPct = (pct - 0.75f) / 0.25f;
            ambientColor = blend(CRGB(255, 120, 0), CRGB(255, 0, 0), (uint8_t)(subPct * 255.0f));
        }
    }

    FastLED.setBrightness(currentBrightness);
    setAll4StripsColor(ambientColor);

    static uint32_t lastDbgTime = 0;
    if (millis() - lastDbgTime >= 500) {
        lastDbgTime = millis();
        Serial.printf("[LED FOOTWELL DBG] Target: %u RPM | Smoothed: %.1f RPM | Pct: %.2f | Brightness: %u | Color: RGB(%u,%u,%u)\n",
                      targetRPM, rpm, pct, currentBrightness, ambientColor.r, ambientColor.g, ambientColor.b);
    }
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
