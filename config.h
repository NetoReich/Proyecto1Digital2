/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "NetoReich"
#define IO_KEY       "aio_UfVh84im1ManBD2T5lhewcKVOjpo"

/******************************* WIFI **************************************/

#define WIFI_SSID "NetoIphone"
#define WIFI_PASS "balticabaltica"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
