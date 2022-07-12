
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp_app_builder.h"

#ifdef M5STICKC
#include "M5StickCPlus.h"
#endif

#ifdef M5CORE2
#include <M5Core2.h>
#endif


using namespace sensesp;

reactesp::ReactESP app;

SKOutputFloat* accelX =
    new SKOutputFloat("sensors.imu.accelX", "/sensors/imu/accelx",
                      new SKMetadata("G", "acceleration in X direction"));

SKOutputFloat* accelY =
    new SKOutputFloat("sensors.imu.accelY", "/sensors/imu/accely",
                      new SKMetadata("G", "acceleration in Y direction"));

SKOutputFloat* accelZ =
    new SKOutputFloat("sensors.imu.accelZ", "/sensors/imu/accelz",
                      new SKMetadata("G", "acceleration in Z direction"));

SKOutputFloat* yaw = new SKOutputFloat("sensors.ahrs.yaw", "/sensors/ahrs/yaw",
                                       new SKMetadata("deg", "yaw angle"));

SKOutputFloat* roll =
    new SKOutputFloat("sensors.ahrs.roll", "/sensors/ahrs/roll",
                      new SKMetadata("deg", "roll angle"));

SKOutputFloat* pitch =
    new SKOutputFloat("sensors.ahrs.pitch", "/sensors/ahrs/pitch",
                      new SKMetadata("deg", "pitch angle"));

#ifdef AMMETER
SKOutputFloat* current =
    new SKOutputFloat("sensors.ammeter.current", "/sensors/ammeter/current",
                      new SKMetadata("A", "current"));
#endif

#ifdef AMMETER
Ammeter ammeter;
#endif

#ifdef LED_BUILTIN
SystemStatusLed* led;
#endif

const unsigned int sensorInterval = 500;

void readSensors() {
  float accX, accY, accZ;
  float y, p, r;

  M5.IMU.getAccelData(&accY, &accY, &accZ);
  accelX->emit(accX);
  accelY->emit(accY);
  accelZ->emit(accZ);

  M5.IMU.getAhrsData(&p, &r, &y);
  yaw->emit(y);
  pitch->emit(p);
  roll->emit(r);

#ifdef AMMETER
  current->emit(ammeter.getCurrent());
#endif

#ifdef M5CORE2
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("accY,   accY,  accZ");
  M5.Lcd.setCursor(0, 140);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accY, accY, accZ);

  M5.Lcd.setCursor(0, 160);
  M5.Lcd.printf("pitch,  roll,  yaw");
  M5.Lcd.setCursor(0, 180);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f deg", pitch, roll, yaw);
#endif
}

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif
  M5.begin();

#ifdef LED_BUILTIN
  led = new SystemStatusLed(LED_BUILTIN);
#endif

  M5.IMU.Init();

  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname(HOST)
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("imu-demo", "somepassword")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_system_info_sensors()
                    ->enable_ota("thisisfine")
#ifdef LED_BUILTIN
                    ->set_system_status_led(led)
#endif
                    ->get_app();

  const char* sk_altitude = "bmp390.altitude";
  const int listen_delay = 1000;
  auto* altitude = new FloatSKListener(sk_altitude, listen_delay);
  altitude->connect_to(new LambdaConsumer<float>([](float a) {
    M5.Lcd.setTextColor(YELLOW, BLACK);

    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf(" alt: %.3f   ", a);
  }));

  const char* sk_vspeed = "bmp390.vspeed";
  auto* vspeed = new FloatSKListener(sk_vspeed, listen_delay);
  vspeed->connect_to(new LambdaConsumer<float>([](float vs) {
    M5.Lcd.setTextColor(YELLOW, BLACK);

    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf(" vspeed: %3.3f   ", vs);
  }));

  const char* sk_gps_altitude = "navigation.gnss.antennaAltitude";
  auto* gps_altitude = new FloatSKListener(sk_gps_altitude, listen_delay);
  gps_altitude->connect_to(new LambdaConsumer<float>([](float gps_alt) {
    M5.Lcd.setTextColor(YELLOW, BLACK);

    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf(" GPS alt: %.3f   ", gps_alt);
  }));

  const char* sk_gpsfix = "navigation.gnss.methodQuality";
  auto* gpsfix = new StringSKListener(sk_gpsfix, listen_delay);
  gpsfix->connect_to(new LambdaConsumer<String>([](String fix) {
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.printf(" gpsfix: %s   ", fix);
  }));

  app.onRepeat(sensorInterval, []() { readSensors(); });

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
