#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// CONFIGURACIÓN DE HARDWARE LED
// ==========================================
#define LED_PIN         33          // Pin GPIO de salida principal
#define NUM_LEDS        83          // 83 LEDs por tira (según test_led_demos.cpp)
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define DEFAULT_BRIGHTNESS 60       // Brillo inicial (0 - 255)

// ==========================================
// MAPEO DE RPM PARA OPEL CORSA 2007 DIESEL (1.3/1.7 CDTI)
// ==========================================
#define RPM_IDLE        800        // Ralentí normal diésel (~800 RPM)
#define RPM_ECO         1800       // Fin de zona ecológica / inicio de zona de par
#define RPM_POWER       3200       // Zona de máxima entrega de potencia
#define RPM_REDLINE     4300       // Inicio de zona roja (corte diésel Corsa)
#define RPM_MAX         4800       // Límite máximo de escala

// ==========================================
// ASIGNACIÓN DE NÚCLEOS FREERTOS (DUAL-CORE)
// ==========================================
#define CORE_BLE        0          // Núcleo 0: Operaciones Bluetooth (NimBLE Client & Server)
#define CORE_LED        1          // Núcleo 1: Renderizado FastLED 60FPS sin interrupciones

// ==========================================
// UUIDs SERVIDOR BLE PARA CONTROL DESDE LA WEB
// ==========================================
#define ESP32_SERVICE_UUID "a21b3690-d98e-11ec-9d64-0242ac120002"
#define ESP32_CHAR_MODE    "a21b3a32-d98e-11ec-9d64-0242ac120002" // Modo/Efecto (1 byte)
#define ESP32_CHAR_COLOR   "a21b3c00-d98e-11ec-9d64-0242ac120002" // Color RGB (3 bytes)
#define ESP32_CHAR_BRIGHT  "a21b3d00-d98e-11ec-9d64-0242ac120002" // Brillo (1 byte)
#define ESP32_CHAR_RPM     "a21b3e00-d98e-11ec-9d64-0242ac120002" // Lectura RPM para Web (2 bytes)

// ==========================================
// MODOS DE ILUMINACIÓN
// ==========================================
enum LedMode {
    MODE_WELCOME = 0,       // Animación de bienvenida (Bordes hacia el Centro)
    MODE_RPM_SHIFTLIGHT,    // Modo Tacómetro RPM Opel Corsa Diesel
    MODE_STATIC_COLOR,      // Color Fijo Personalizado
    MODE_RAINBOW,           // Efecto Arcoíris
    MODE_BREATHING,         // Efecto Respiración / Pulsación
    MODE_STROBE,            // Efecto Estroboscópico
    MODE_OFF                // Apagado / Standby
};

#endif // CONFIG_H
