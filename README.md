# ServeurGpsCanBus

Permet de diffuser des trames GPS sur un Bus CAN.

Adresse de diffusion :

    * 0x400
    * Vitesse 1Mbits.

Informations diffusées :
    
    * Latitude.
    * Longitude.
    * Vitesse (km/h).
    * Nombres de satellites.
    * Précision en cm.
    * Si données valides.
    * Heure.
    * Minutes.
    * Seconds.
    * Etat d'un bouton.

Materiel utilisé :

    * Capteur GPS : NEO-8M
    * Esp32       : Lolin-S3 (Esp32-S3-Wroom-1)
    * Module CAN  : SN65HVD230 CAN Bus Transceiver Communication