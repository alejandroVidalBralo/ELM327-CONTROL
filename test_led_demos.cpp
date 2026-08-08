/**
 * CorollaLights - Demostrador de Animaciones Multicanal Seguro (Versión Avanzada)
 * 
 * PROPÓSITO:
 * Probar simultáneamente los pines de salida GPIO 33, 32, 25 y 26 con 83 LEDs cada uno.
 * Añade animaciones especiales de prueba controlables por Bluetooth y puerto serie.
 * 
 * ANIMACIONES SOPORTADAS:
 * - Ambientales: ECO (Azul), NORMAL (Blanco), SPORT (Rojo).
 * - Transiciones: Bienvenida (Avance 0-82), Despedida (Vaciado 82-0 en color activo).
 * - Giro: Intermitente expansivo desde el centro (Ámbar suave).
 * - Avanzadas:
 *   1. Barrido de Arcoíris (Rainbow).
 *   2. Destello Policial (Strobe Red/Blue).
 *   3. Respiración Multicolor (Fade Rojo -> Verde -> Azul -> Amarillo).
 */

#include <Arduino.h>
#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define NUM_LEDS        83       // 83 LEDs por tira

// UUIDs del servicio de demostración BLE
#define DEMO_SERVICE_UUID   "a21b3690-d98e-11ec-9d64-0242ac120002"
#define DEMO_CHAR_UUID      "a21b3a32-d98e-11ec-9d64-0242ac120002"

// Buffers para los 4 canales simultáneos
static CRGB leds33[NUM_LEDS];
static CRGB leds32[NUM_LEDS];
static CRGB leds25[NUM_LEDS];
static CRGB leds26[NUM_LEDS];

// Colores de los Modos
struct ModeColors {
    CRGB eco;
    CRGB normal;
    CRGB sport;
    uint8_t brightness;
};

static ModeColors colors = {
    CRGB(0, 70, 255),    // ECO: Azul
    CRGB(200, 200, 200), // NORMAL: Blanco
    CRGB(255, 0, 0),     // SPORT: Rojo
    40                   // Brillo inicial seguro al arrancar (15%)
};

// Estados de la máquina de animación
enum AnimState {
    STATE_BOOT_WAIT,
    STATE_AMBIENT,
    STATE_WELCOME,
    STATE_GOODBYE,
    STATE_RAINBOW,
    STATE_STROBE,
    STATE_BREATHING
};

static AnimState currentAnimState = STATE_BOOT_WAIT;
static uint8_t activeMode = 1;      // 0=ECO, 1=NORMAL, 2=SPORT
static bool turnSignalActive = false;

// Variables de progresión
static int welcomeStep = 0;
static int goodbyeStep = 0;
static uint32_t turnSignalFrame = 0;

// Variables BLE
static BLECharacteristic* pDemoCharacteristic = nullptr;
static bool deviceConnected = false;

// Callbacks BLE
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        if (currentAnimState == STATE_BOOT_WAIT) {
            currentAnimState = STATE_WELCOME;
            welcomeStep = 0;
        }
        Serial.println("[BLE] Cliente conectado. Iniciando bienvenida...");
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        currentAnimState = STATE_BOOT_WAIT; // Volver a parpadear en blanco esperando conexión
        Serial.println("[BLE] Cliente desconectado. Reiniciando publicidad...");
        pServer->getAdvertising()->start();
    }
};

// Callback de lectura/escritura de comandos (GATT)
class DemoBleCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        uint8_t* rxData = pCharacteristic->getData();
        size_t len = pCharacteristic->getLength();

        if (len >= 2) {
            uint8_t cmdType = rxData[0];
            uint8_t cmdVal = rxData[1];

            if (currentAnimState == STATE_BOOT_WAIT) {
                currentAnimState = STATE_AMBIENT;
            }

            switch (cmdType) {
                case 0x01: // Cambiar Modo Ambiente
                    if (cmdVal <= 2) {
                        activeMode = cmdVal;
                        currentAnimState = STATE_AMBIENT;
                        Serial.printf("[BLE] Modo Ambiente: %d\n", activeMode);
                    }
                    break;
                case 0x02: // Disparar Bienvenida
                    currentAnimState = STATE_WELCOME;
                    welcomeStep = 0;
                    Serial.println("[BLE] Comando: Disparar Bienvenida");
                    break;
                case 0x03: // Disparar Despedida
                    currentAnimState = STATE_GOODBYE;
                    goodbyeStep = 0;
                    Serial.println("[BLE] Comando: Disparar Despedida");
                    break;
                case 0x04: // Estado Intermitente
                    turnSignalActive = (cmdVal == 1);
                    Serial.printf("[BLE] Intermitente: %s\n", turnSignalActive ? "SI" : "NO");
                    break;
                case 0x05: // Ajustar Brillo (0 a 255)
                    colors.brightness = cmdVal;
                    Serial.printf("[BLE] Brillo: %d\n", colors.brightness);
                    break;
                case 0x08: // Modos Avanzados de Test
                    if (cmdVal == 0) {
                        currentAnimState = STATE_AMBIENT;
                        Serial.println("[BLE] Volviendo a Modo Ambiente.");
                    } else if (cmdVal == 1) {
                        currentAnimState = STATE_RAINBOW;
                        Serial.println("[BLE] Activado Modo Arcoíris.");
                    } else if (cmdVal == 2) {
                        currentAnimState = STATE_STROBE;
                        Serial.println("[BLE] Activado Modo Estroboscópico.");
                    } else if (cmdVal == 3) {
                        currentAnimState = STATE_BREATHING;
                        Serial.println("[BLE] Activado Modo Respiración.");
                    }
                    break;
                default:
                    break;
            }
        }
    }
};

// Declaraciones de funciones
void processSerialCommands();
void executeCommand(String command);
void renderAmbient(CRGB baseColor);
void renderWelcome(CRGB baseColor);
void renderGoodbye(CRGB baseColor);
void renderCenterOutBlinker();
void renderRainbowWave();
void renderStrobeAlert();
void renderBreathingColors();
void showAllStrips();
void printHelpMenu();

void setup() {
    Serial.begin(115200);
    delay(200);
    printHelpMenu();

    // Registrar las 4 tiras físicas independientes en FastLED
    FastLED.addLeds<LED_TYPE, 33, COLOR_ORDER>(leds33, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, 32, COLOR_ORDER>(leds32, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds25, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds26, NUM_LEDS).setCorrection(TypicalLEDStrip);

    FastLED.setBrightness(colors.brightness);
    FastLED.clear();
    FastLED.show();

    // Inicializar BLE
    BLEDevice::init("DemoLights_BLE");
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(DEMO_SERVICE_UUID);
    pDemoCharacteristic = pService->createCharacteristic(
        DEMO_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pDemoCharacteristic->setCallbacks(new DemoBleCallbacks());

    pService->start();
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(DEMO_SERVICE_UUID);
    pAdvertising->start();

    Serial.println("[System] Servidor BLE 'DemoLights_BLE' activo en pines 33, 32, 25 y 26.");
}

void loop() {
    processSerialCommands();

    static uint32_t lastFrame = 0;
    uint32_t now = millis();

    // Refresco estable a 40 FPS
    if (now - lastFrame < 25) {
        delay(1);
        return;
    }
    lastFrame = now;

    // 1. Obtener color base del modo
    CRGB baseColor = colors.normal;
    if (activeMode == 0) baseColor = colors.eco;
    else if (activeMode == 2) baseColor = colors.sport;

    // 2. Control del renderizado según estado e intermitente
    if (currentAnimState == STATE_BOOT_WAIT) {
        bool blinkState = (millis() / 500) % 2 == 0;
        CRGB blinkColor = blinkState ? CRGB(160, 160, 160) : CRGB::Black; // Blanco medio/brillante

        for (int i = 0; i < NUM_LEDS; i++) {
            leds33[i] = blinkColor;
            leds32[i] = blinkColor;
            leds25[i] = blinkColor;
            leds26[i] = blinkColor;
        }
    } 
    else if (turnSignalActive && currentAnimState != STATE_GOODBYE) {
        renderCenterOutBlinker();
    } 
    else {
        switch (currentAnimState) {
            case STATE_AMBIENT:
                renderAmbient(baseColor);
                break;
                
            case STATE_WELCOME:
                renderWelcome(baseColor);
                break;
                
            case STATE_GOODBYE:
                renderGoodbye(baseColor);
                break;

            case STATE_RAINBOW:
                renderRainbowWave();
                break;

            case STATE_STROBE:
                renderStrobeAlert();
                break;

            case STATE_BREATHING:
                renderBreathingColors();
                break;
        }
    }

    FastLED.setBrightness(colors.brightness);
    FastLED.show();
}

// MODO AMBIENTE: Sólido replicado
void renderAmbient(CRGB baseColor) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds33[i] = baseColor;
        leds32[i] = baseColor;
        leds25[i] = baseColor;
        leds26[i] = baseColor;
    }
}

// EFECTO BIENVENIDA: Ola de llenado (0 -> 82)
void renderWelcome(CRGB baseColor) {
    welcomeStep += 1;
    if (welcomeStep >= NUM_LEDS) {
        welcomeStep = NUM_LEDS;
        currentAnimState = STATE_AMBIENT;
        Serial.println("[Anim] Bienvenida completada.");
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB col = (i < welcomeStep) ? baseColor : CRGB::Black;
        leds33[i] = col; leds32[i] = col; leds25[i] = col; leds26[i] = col;
    }
}

// EFECTO DESPEDIDA: Ola de vaciado (82 -> 0)
void renderGoodbye(CRGB baseColor) {
    goodbyeStep += 1;
    if (goodbyeStep >= NUM_LEDS) {
        goodbyeStep = NUM_LEDS;
        for (int i = 0; i < NUM_LEDS; i++) {
            leds33[i] = CRGB::Black; leds32[i] = CRGB::Black;
            leds25[i] = CRGB::Black; leds26[i] = CRGB::Black;
        }
        return; 
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB col = (i < (NUM_LEDS - goodbyeStep)) ? baseColor : CRGB::Black;
        leds33[i] = col; leds32[i] = col; leds25[i] = col; leds26[i] = col;
    }
}

// INTERMITENTE CENTRAL EXPANSIVO ÁMBAR
void renderCenterOutBlinker() {
    turnSignalFrame++;
    int center = NUM_LEDS / 2;
    int maxDist = NUM_LEDS / 2;
    int step = (turnSignalFrame) % (maxDist + 20);
    CRGB amberColor = CRGB(255, 110, 0);

    for (int i = 0; i < NUM_LEDS; i++) {
        leds33[i].nscale8(180); leds32[i].nscale8(180);
        leds25[i].nscale8(180); leds26[i].nscale8(180);
    }

    if (step <= maxDist) {
        for (int i = 0; i <= step; i++) {
            int leftLed = center - i;
            int rightLed = center + i;
            if (leftLed >= 0) {
                leds33[leftLed] = amberColor; leds32[leftLed] = amberColor;
                leds25[leftLed] = amberColor; leds26[leftLed] = amberColor;
            }
            if (rightLed < NUM_LEDS) {
                leds33[rightLed] = amberColor; leds32[rightLed] = amberColor;
                leds25[rightLed] = amberColor; leds26[rightLed] = amberColor;
            }
        }
    }
}

// EFECTO AVANZADO 1: BARRIDO DE ARCOÍRIS
void renderRainbowWave() {
    static uint8_t hue = 0;
    hue += 2; // Velocidad del arcoíris
    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB col = CHSV(hue + (i * 3), 255, 255);
        leds33[i] = col; leds32[i] = col; leds25[i] = col; leds26[i] = col;
    }
}

// EFECTO AVANZADO 2: DESTELLO POLICIAL (Rojo y Azul)
void renderStrobeAlert() {
    static uint32_t lastStrobeTime = 0;
    static bool strobeToggle = false;
    if (millis() - lastStrobeTime >= 60) {
        lastStrobeTime = millis();
        strobeToggle = !strobeToggle;
    }

    CRGB col1 = strobeToggle ? CRGB::Blue : CRGB::Red;
    CRGB col2 = strobeToggle ? CRGB::Red : CRGB::Blue;

    int half = NUM_LEDS / 2;
    for (int i = 0; i < NUM_LEDS; i++) {
        CRGB finalCol = (i < half) ? col1 : col2;
        leds33[i] = finalCol; leds32[i] = finalCol; leds25[i] = finalCol; leds26[i] = finalCol;
    }
}

// EFECTO AVANZADO 3: RESPIRACIÓN MULTICOLOR
void renderBreathingColors() {
    uint8_t breathVal = beatsin8(15, 20, 255); // Respiración lenta
    static uint8_t colorIndex = 0;
    static uint32_t lastColorTime = 0;

    if (millis() - lastColorTime >= 4000) {
        lastColorTime = millis();
        colorIndex = (colorIndex + 1) % 4; // Rotar 4 colores
    }

    CRGB baseColor;
    if (colorIndex == 0) baseColor = CRGB::Red;
    else if (colorIndex == 1) baseColor = CRGB::Green;
    else if (colorIndex == 2) baseColor = CRGB::Blue;
    else baseColor = CRGB::Yellow;

    CRGB col = baseColor.nscale8_video(breathVal);
    for (int i = 0; i < NUM_LEDS; i++) {
        leds33[i] = col; leds32[i] = col; leds25[i] = col; leds26[i] = col;
    }
}

// Procesamiento Serie
void processSerialCommands() {
    static String inputBuffer = "";
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                executeCommand(inputBuffer);
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}

// Ejecución Serie
void executeCommand(String command) {
    command.toUpperCase();

    if (currentAnimState == STATE_BOOT_WAIT) {
        currentAnimState = STATE_AMBIENT;
    }

    if (command == "ECO") {
        activeMode = 0;
        currentAnimState = STATE_AMBIENT;
        Serial.println("[Test] ECO.");
    }
    else if (command == "NOR") {
        activeMode = 1;
        currentAnimState = STATE_AMBIENT;
        Serial.println("[Test] NORMAL.");
    }
    else if (command == "SPO") {
        activeMode = 2;
        currentAnimState = STATE_AMBIENT;
        Serial.println("[Test] SPORT.");
    }
    else if (command == "BIEN") {
        currentAnimState = STATE_WELCOME;
        welcomeStep = 0;
        Serial.println("[Test] Bienvenida.");
    }
    else if (command == "DESP") {
        currentAnimState = STATE_GOODBYE;
        goodbyeStep = 0;
        Serial.println("[Test] Despedida.");
    }
    else if (command == "INT") {
        turnSignalActive = true;
        turnSignalFrame = 0;
        Serial.println("[Test] Intermitente.");
    }
    else if (command == "APAG") {
        turnSignalActive = false;
        currentAnimState = STATE_AMBIENT;
        Serial.println("[Test] Detener intermitente / Volver a ambiente.");
    }
    else if (command == "RAINBOW") {
        currentAnimState = STATE_RAINBOW;
        Serial.println("[Test] Modo Arcoíris.");
    }
    else if (command == "STROBE") {
        currentAnimState = STATE_STROBE;
        Serial.println("[Test] Modo Estroboscópico.");
    }
    else if (command == "BREATH") {
        currentAnimState = STATE_BREATHING;
        Serial.println("[Test] Modo Respiración.");
    }
    else {
        Serial.printf("[Test] Comando no reconocido: '%s'\n", command.c_str());
    }
}

void printHelpMenu() {
    Serial.println("\n==================================================");
    Serial.println("  DEMOSTRADOR DE EFECTOS AVANZADO: PINES 33,32,25,26");
    Serial.println("==================================================");
    Serial.println("  Comandos Serie: ECO, NOR, SPO, BIEN, DESP, INT, APAG");
    Serial.println("                  RAINBOW, STROBE, BREATH");
    Serial.println("==================================================\n");
}
