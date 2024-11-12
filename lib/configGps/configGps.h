#ifndef CONFIG_GPS_H
#define CONFIG_GPS_H

#include <Arduino.h>

//------------------------------------------------------------------------------
// Commandes UBX pour configurer le GPS
//------------------------------------------------------------------------------

// Taille totale des commandes de configuration
static const uint8_t UBLOX_INIT_SIZE = 70;  // 14 + 28 + 28 (CFG-RATE + CFG-PRT + CFG-MSG)

// Indexes des différentes commandes dans le tableau
static const uint8_t IDX_CFG_RATE = 0;   // Configure update rate (14 bytes)
static const uint8_t IDX_CFG_PRT = 14;   // Configure port settings (28 bytes)  
static const uint8_t IDX_CFG_MSG = 42;   // Configure NMEA messages (28 bytes)

// Déclaration du tableau de configuration
extern const uint8_t UBLOX_INIT[70] __attribute__((progmem));

#endif // CONFIG_GPS_H