/**
 * ESP32 ELM327 Control Center & Ultra-Fluid LED Shift Light System
 * 
 * Arquitectura Dual-Core FreeRTOS:
 * - Core 0: Tarea BLE (NimBLE Client para ELM327 + NimBLE Server para Web Bluetooth)
 * - Core 1: Tarea FastLED (Renderizado a 60 FPS con suavizado sub-LED y filtro exponencial)
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

// Buffers de LEDs (83 LEDs en GPIO 33)
static CRGB g_leds[NUM_LEDS];

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
class ElmScanCallbacks : public NimBLEScanCallbacks {
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

        // Búsqueda alternativa por nombre conocido
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
    Serial.println("  Tira de LEDs: 83 LEDs en GPIO 33 | Suavizado 60 FPS");
    Serial.println("=======================================================\n");

    // Crear Mutex FreeRTOS para sincronización limpia entre núcleos
    g_dataMutex = xSemaphoreCreateMutex();

    // Inicializar FastLED en el sistema principal
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(g_leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(DEFAULT_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();

    // Inicializar Motor NimBLE Dual (Cliente + Servidor)
    NimBLEDevice::init("ESP32_OBD_ShiftLight");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Máxima potencia de transmisión BLE

    // Configurar Servidor BLE para la Web App
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

    Serial.println("[System] Servidor BLE 'ESP32_OBD_ShiftLight' activo para la Web.");

    // Lector / Cliente BLE (ELM327)
    pElmClient = NimBLEDevice::createClient();
    pElmClient->setClientCallbacks(new ElmClientCallbacks(), false);

    // Lanzar Tareas Dual-Core
    // Tarea BLE en Core 0
    xTaskCreatePinnedToCore(
        taskBLE,
        "TaskBLE",
        8192,
        NULL,
        2,
        NULL,
        CORE_BLE
    );

    // Tarea LED en Core 1 (Renderizado prioritario 60FPS)
    xTaskCreatePinnedToCore(
        taskLED,
        "TaskLED",
        6144,
        NULL,
        3,
        NULL,
        CORE_LED
    );
}

void loop() {
    // El bucle principal se delega totalmente a las tareas de FreeRTOS
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
                    Serial.println("[BLE Task] Iniciando escaneo de adaptadores ELM327 BLE...");
                    NimBLEScan* pScan = NimBLEDevice::getScan();
                    pScan->setScanCallbacks(new ElmScanCallbacks());
                    pScan->setInterval(45);
                    pScan->setWindow(15);
                    pScan->setActiveScan(true);
                    pScan->start(4, false); // Escaneo de 4 segundos
                } else {
                    g_elmState = BLE_STATE_CONNECTING;
                }
                break;
            }

            case BLE_STATE_CONNECTING: {
                if (g_targetElmDevice != nullptr) {
                    Serial.println("[BLE Task] Intentando conectar con ELM327...");
                    if (connectToElmServer(g_targetElmDevice)) {
                        g_elmState = BLE_STATE_INIT_ELM;
                        stateTimer = now;
                    } else {
                        Serial.println("[BLE Task] Fallo al conectar. Reintentando escaneo...");
                        g_targetElmDevice = nullptr;
                        g_elmState = BLE_STATE_RETRY_WAIT;
                        stateTimer = now;
                    }
                }
                break;
            }

            case BLE_STATE_INIT_ELM: {
                if (now - stateTimer > 300) { // Enviar órdenes iniciales AT
                    Serial.println("[BLE Task] Inicializando comandos AT ELM327...");
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
                // Sondeo continuo de RPM (01 0C\r) cada 70ms
                if (now - lastPollTime >= 70) {
                    lastPollTime = now;
                    if (pElmTxChar && pElmClient && pElmClient->isConnected()) {
                        pElmTxChar->writeValue("01 0C\r", false);
                    }
                }

                // Detección de desconexión por Timeout (4 segundos sin respuesta)
                if (now - g_lastElmDataTime > 4000) {
                    Serial.println("[BLE Task] Timeout de datos ELM327. Reiniciando conexión...");
                    if (pElmClient && pElmClient->isConnected()) {
                        pElmClient->disconnect();
                    }
                    g_elmState = BLE_STATE_RETRY_WAIT;
                    stateTimer = now;
                }
                break;
            }

            case BLE_STATE_RETRY_WAIT: {
                if (now - stateTimer >= 2000) { // Esperar 2s antes de reesccanear
                    g_targetElmDevice = nullptr;
                    g_elmState = BLE_STATE_SCANNING;
                }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Ceder tiempo de CPU en Core 0
    }
}

// Conectar con Servidor BLE ELM327
bool connectToElmServer(NimBLEAdvertisedDevice* advertisedDevice) {
    if (!pElmClient->connect(advertisedDevice)) {
        return false;
    }

    // Buscar Servicios
    for (const char* uuidStr : ELM_SERVICES) {
        NimBLERecordService* pService = pElmClient->getService(uuidStr);
        if (pService != nullptr) {
            // Resolver características de Notificación y Escritura
            pElmRxChar = pService->getCharacteristic(NimBLEUUID("0000fff1-0000-1000-8000-00805f9b34fb"));
            if (!pElmRxChar) pElmRxChar = pService->getCharacteristic(NimBLEUUID("0000ffe1-0000-1000-8000-00805f9b34fb"));
            if (!pElmRxChar) pElmRxChar = pService->getCharacteristic(NimBLEUUID("00002af0-0000-1000-8000-00805f9b34fb"));

            pElmTxChar = pService->getCharacteristic(NimBLEUUID("0000fff2-0000-1000-8000-00805f9b34fb"));
            if (!pElmTxChar) pElmTxChar = pElmRxChar; // Fallback si es característica bidireccional

            if (pElmRxChar && pElmRxChar->canNotify()) {
                pElmRxChar->subscribe(true, elmNotifyCallback);
            }
            return true;
        }
    }
    return false;
}

// Parseador de respuestas OBD-II (01 0C)
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
                
                // Notificar valor de RPM a la Web si está conectada
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
// TAREA CORE 1: RENDERIZADOR FASTLED ULTRA-FLUIDO (60 FPS)
// ==========================================
void taskLED(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(16); // ~60 FPS (16.6ms)

    float smoothedRPM = 0.0f;
    uint8_t welcomeStep = 0;

    for (;;) {
        // Leer estado compartido bajo Mutex
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

        // Aplicar filtro exponencial suavizado para fluidez extrema
        smoothedRPM += ((float)targetRPM - smoothedRPM) * 0.12f;

        FastLED.setBrightness(brightness);

        // Renderizar según Modo
        switch (mode) {
            case MODE_WELCOME: {
                renderWelcomeAnimation(welcomeStep);
                welcomeStep++;
                if (welcomeStep >= (NUM_LEDS / 2) + 1) {
                    welcomeStep = 0;
                    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
                    g_currentMode = MODE_RPM_SHIFTLIGHT; // Transición automática al tacómetro
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
                FastLED.clear();
                break;
        }

        FastLED.show();
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Mantener 60 FPS exactos
    }
}

// ==========================================
// ANIMACIONES Y RENDERIZADORES DE ILUMINACIÓN
// ==========================================

// ANIMACIÓN DE BIENVENIDA: Llenado desde los dos bordes hacia el centro (Edge-to-Center)
void renderWelcomeAnimation(uint8_t step) {
    FastLED.clear();
    int half = NUM_LEDS / 2;
    CRGB welcomeColor = CRGB(0, 220, 255); // Cian brillante de bienvenida

    for (int i = 0; i <= step && i <= half; i++) {
        int leftIdx = i;
        int rightIdx = NUM_LEDS - 1 - i;

        if (leftIdx >= 0 && leftIdx < NUM_LEDS) g_leds[leftIdx] = welcomeColor;
        if (rightIdx >= 0 && rightIdx < NUM_LEDS) g_leds[rightIdx] = welcomeColor;
    }
}

// RENDIMIENTO TACÓMETRO RPM OPEL CORSA DIESEL (83 LEDs Sub-pixel Liquid)
void renderRpmShiftLight(float rpm) {
    FastLED.clear();

    // 1. Estado Reposo / Motor Apagado (< 750 RPM)
    if (rpm < RPM_IDLE) {
        g_leds[0] = CRGB(0, 40, 80); // Luz guía mínima en reposo
        return;
    }

    // 2. Estado Aviso de Corte / Shift Light Warning (> 4300 RPM)
    if (rpm >= RPM_REDLINE) {
        static bool flashToggle = false;
        flashToggle = !flashToggle;
        CRGB alertColor = flashToggle ? CRGB::Red : CRGB::White;
        for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = alertColor;
        return;
    }

    // 3. Mapeo Progresivo Fluido (800 -> 4300 RPM en 83 LEDs)
    float pct = (rpm - (float)RPM_IDLE) / ((float)RPM_REDLINE - (float)RPM_IDLE);
    pct = constrain(pct, 0.0f, 1.0f);

    float numLedsToLit = pct * (float)NUM_LEDS;
    int fullLeds = (int)numLedsToLit;
    float fractionalPart = numLedsToLit - fullLeds;

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < fullLeds || (i == fullLeds && fractionalPart > 0.05f)) {
            // Calcular color dinámico según la posición relativa del LED
            float ledPct = (float)i / (float)NUM_LEDS;
            CRGB color;

            if (ledPct < 0.35f) {
                // Zona 1: Eco / Cruceros (Cian -> Verde)
                color = blend(CRGB(0, 180, 255), CRGB(0, 255, 60), (uint8_t)(ledPct * (255.0f / 0.35f)));
            } else if (ledPct < 0.75f) {
                // Zona 2: Par Motor / Aceleración (Verde -> Amarillo / Naranja)
                color = blend(CRGB(0, 255, 60), CRGB(255, 140, 0), (uint8_t)((ledPct - 0.35f) * (255.0f / 0.40f)));
            } else {
                // Zona 3: Máxima Entrega (Naranja -> Rojo Intenso)
                color = blend(CRGB(255, 140, 0), CRGB(255, 0, 0), (uint8_t)((ledPct - 0.75f) * (255.0f / 0.25f)));
            }

            // Aplicar brillo fraccional al último LED para lograr movimiento líquido sub-pixel
            if (i == fullLeds) {
                color.nscale8_video((uint8_t)(fractionalPart * 255.0f));
            }

            g_leds[i] = color;
        }
    }
}

// COLOR FIJO
void renderStaticColor(CRGB color) {
    for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = color;
}

// ARCOÍRIS FLUIDO
void renderRainbow() {
    static uint8_t hue = 0;
    hue += 1;
    fill_rainbow(g_leds, NUM_LEDS, hue, 3);
}

// RESPIRACIÓN / PULSACIÓN
void renderBreathing(CRGB color) {
    uint8_t val = beatsin8(14, 30, 255);
    CRGB col = color;
    col.nscale8_video(val);
    for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = col;
}

// ESTROBOSCÓPICO
void renderStrobe() {
    static uint32_t lastStrobe = 0;
    static bool toggle = false;
    if (millis() - lastStrobe >= 70) {
        lastStrobe = millis();
        toggle = !toggle;
    }
    CRGB col = toggle ? CRGB::White : CRGB::Black;
    for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = col;
}
