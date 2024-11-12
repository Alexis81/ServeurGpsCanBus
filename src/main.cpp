#include <Adafruit_NeoPixel.h>
#include <driver/twai.h>
#include <HardwareSerial.h>
#include "Arduino.h"
#include "main.h"
#include "configGps.h"

// Structure pour les données GPS
struct GpsData
{
    float latitude;
    float longitude;
    float speed;
    int satellites;
    int precision_cm;
    bool hasValidFix;
    int hours;
    int minutes;
    int seconds;
    unsigned long lastStatsTime;
};

// Variables globales
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial GPSSerial(1);
char gpsBuffer[1024];
int bufferIndex = 0;
GpsData gpsData = {0};

// Variables pour la task LED
TaskHandle_t ledTaskHandle = NULL;
bool ledState = false;

// Variables pour le bouton
volatile bool buttonState = false;     // État actuel du bouton
volatile bool lastButtonState = false; // Dernier état stable du bouton
volatile bool buttonPressed = false;   // État final du bouton (celui utilisé par le programme)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Fonction pour la task LED
void ledTask(void *parameter)
{
    while (1)
    {
        ledState = !ledState;
        if (gpsData.hasValidFix)
        {
            // Signal GPS valide - clignotement vert
            strip.setPixelColor(0, ledState ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0));
        }
        else
        {
            // Pas de signal GPS - clignotement rouge
            strip.setPixelColor(0, ledState ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0));
        }
        strip.show();
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL));
    }
}

// Fonction d'interruption pour le bouton
void IRAM_ATTR buttonISR()
{
    buttonState = (digitalRead(BUTTON1_PIN) == LOW);
}

// Fonction à appeler dans le loop() pour gérer le debouncing
void handleButton()
{
    if (buttonState != lastButtonState)
    {
        lastDebounceTime = millis();
        lastButtonState = buttonState;
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (buttonState != buttonPressed)
        {
            buttonPressed = buttonState;
        }
    }
}

// Prototypes des fonctions
void setup_twai(void);
void configureGPS(void);
void sendTwaiMessage(void);
void parseGNRMC(const char *frame);
void parseGNGGA(const char *frame);
void processGPSData(void);
void printStats(void);

// Implémentation des fonctions
void setup_twai()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        Serial.println("Erreur installation TWAI");
        return;
    }

    if (twai_start() != ESP_OK)
    {
        Serial.println("Erreur démarrage TWAI");
        return;
    }
    // Serial.println("TWAI initialisé avec succès à 1Mbit/s");
}

void configureGPS()
{
    Serial.println("Configuration du GPS...");
    for (uint8_t i = 0; i < UBLOX_INIT_SIZE; i++) {
        uint8_t data = pgm_read_byte(&UBLOX_INIT[i]);
        Serial2.write(data);
    }

    delay(1000);
    Serial.println("GPS configuré !");
}

void sendTwaiMessage()
{
    if (gpsData.hasValidFix)
    {
        twai_message_t message;
        message.flags = TWAI_MSG_FLAG_NONE;

        // Premier message: latitude + longitude (8 bytes)
        message.identifier = TWAI_MSG_ID;
        message.data_length_code = 8;
        memcpy(message.data, &gpsData.latitude, sizeof(float));
        memcpy(message.data + 4, &gpsData.longitude, sizeof(float));

        if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            Serial.print(message.identifier, HEX);
            Serial.print(" Data: ");
            for (int i = 0; i < message.data_length_code; i++)
            {
                if (message.data[i] < 0x10)
                    Serial.print("0");
                Serial.print(message.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        else
        {
            Serial.println("Erreur envoi TWAI Heure");
        }

        // Deuxième message: vitesse + satellites + précision (7 bytes)
        message.identifier = TWAI_MSG_ID + 1;
        message.data_length_code = 7;
        memcpy(message.data, &gpsData.speed, sizeof(float));
        message.data[4] = gpsData.satellites;
        memcpy(message.data + 5, &gpsData.precision_cm, sizeof(short));

        if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            Serial.print(message.identifier, HEX);
            Serial.print(" Data: ");
            for (int i = 0; i < message.data_length_code; i++)
            {
                if (message.data[i] < 0x10)
                    Serial.print("0");
                Serial.print(message.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        else
        {
            Serial.println("Erreur envoi TWAI Heure");
        }

        // Troisième message: heure (3 bytes)
        message.identifier = TWAI_MSG_ID + 2;
        message.data_length_code = 4;
        message.data[0] = gpsData.hours;
        message.data[1] = gpsData.minutes;
        message.data[2] = gpsData.seconds;
        message.data[3] = buttonPressed ? 0x01 : 0x00;

        if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            Serial.print(message.identifier, HEX);
            Serial.print(" Data: ");
            for (int i = 0; i < message.data_length_code; i++)
            {
                if (message.data[i] < 0x10)
                    Serial.print("0");
                Serial.print(message.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        else
        {
            Serial.println("Erreur envoi TWAI Heure et bouton");
        }
    }
}

void parseGNRMC(const char *frame)
{
    char time_str[16] = {0};
    char status;
    char lat_str[16] = {0};
    char lat_dir;
    char lon_str[16] = {0};
    char lon_dir;
    char speed_str[16] = {0};

    if (sscanf(frame, "$GNRMC,%[^,],%c,%[^,],%c,%[^,],%c,%[^,]",
               time_str, &status, lat_str, &lat_dir, lon_str, &lon_dir, speed_str) == 7)
    {

        if (status == 'A')
        {
            if (strlen(time_str) >= 6)
            {
                gpsData.hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
                gpsData.minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
                gpsData.seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');
            }

            float lat_deg = (lat_str[0] - '0') * 10 + (lat_str[1] - '0');
            float lat_min = atof(lat_str + 2);
            gpsData.latitude = lat_deg + lat_min / 60.0;
            if (lat_dir == 'S')
                gpsData.latitude = -gpsData.latitude;

            float lon_deg = (lon_str[0] - '0') * 100 + (lon_str[1] - '0') * 10 + (lon_str[2] - '0');
            float lon_min = atof(lon_str + 3);
            gpsData.longitude = lon_deg + lon_min / 60.0;
            if (lon_dir == 'W')
                gpsData.longitude = -gpsData.longitude;

            gpsData.speed = atof(speed_str) * 1.852;
            gpsData.hasValidFix = true;
        }
        else
        {
            gpsData.hasValidFix = false;
        }
    }
}

void parseGNGGA(const char *frame)
{
    char dummy[16];
    int fix = 0;
    int sats = 0;
    float hdop = 0.0;

    if (sscanf(frame, "$GNGGA,%[^,],%[^,],%[^,],%[^,],%[^,],%d,%d,%f",
               dummy, dummy, dummy, dummy, dummy, &fix, &sats, &hdop) >= 8)
    {

        gpsData.satellites = sats;

        int basePrecision;
        switch (fix)
        {
        case 4:
            basePrecision = 1;
            break;
        case 5:
            basePrecision = 10;
            break;
        case 2:
            basePrecision = 100;
            break;
        case 1:
            basePrecision = 200;
            break;
        default:
            basePrecision = 500;
        }
        gpsData.precision_cm = (int)(basePrecision * hdop);
    }
}

void processGPSData()
{
    while (GPSSerial.available())
    {
        char c = GPSSerial.read();
        gpsBuffer[bufferIndex++] = c;

        if (c == '\n' || bufferIndex >= sizeof(gpsBuffer) - 1)
        {
            gpsBuffer[bufferIndex] = '\0';

            if (strncmp(gpsBuffer, "$GNRMC", 6) == 0)
            {
                parseGNRMC(gpsBuffer);
            }
            else if (strncmp(gpsBuffer, "$GNGGA", 6) == 0)
            {
                parseGNGGA(gpsBuffer);
                sendTwaiMessage();
            }

            bufferIndex = 0;
        }

        if (bufferIndex >= sizeof(gpsBuffer))
        {
            bufferIndex = 0;
            Serial.println("Buffer overflow prevented");
        }
    }
}

void printStats()
{
    unsigned long currentTime = millis();
    if (currentTime - gpsData.lastStatsTime >= 5000)
    {
        Serial.println("\n=== Stats ===");
        Serial.printf("Fix valid: %s\n", gpsData.hasValidFix ? "Yes" : "No");
        if (gpsData.hasValidFix)
        {
            Serial.printf("Position: %.6f, %.6f\n", gpsData.latitude, gpsData.longitude);
            Serial.printf("Speed: %.1f km/h\n", gpsData.speed);
            Serial.printf("Satellites: %d\n", gpsData.satellites);
            Serial.printf("Precision: %d cm\n", gpsData.precision_cm);
        }
        Serial.println("============\n");
        gpsData.lastStatsTime = currentTime;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n=== GPS-TWAI Starting ===");

    // Configuration du bouton avec pull-up et interruption
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), buttonISR, CHANGE);

    strip.begin();
    strip.show();

    // Création de la task LED
    xTaskCreate(
        ledTask,       // Fonction de la task
        "LED_Task",    // Nom pour debugging
        2048,          // Taille de la stack
        NULL,          // Paramètre de la task
        1,             // Priorité de la task
        &ledTaskHandle // Handle de la task
    );

    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    configureGPS();
    GPSSerial.flush();
    GPSSerial.updateBaudRate(115200);

    setup_twai();

    Serial.println("=== Initialization Complete ===\n");
}

void loop()
{
    handleButton();
    processGPSData();
    // printStats();
    delay(1);
}