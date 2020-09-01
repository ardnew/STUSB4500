#include <STUSB4500.h>

#define USBPD_RST_PIN  22
#define USBPD_ATCH_PIN 24
#define USBPD_ALRT_PIN 26

#define PDO_STR_LEN    48

STUSB4500 usbpd(USBPD_RST_PIN);

void usbpdCableAttached(void)
{
  Serial.println("cable attached");

  // set power to 9V 3A when cable re-attached
  usbpd.setPower(9000, 3000);
  usbpd.updateSinkCapabilities();
  usbpd.requestSourceCapabilities();
}

void usbpdCableDetached(void)
{
  Serial.println("cable detached");
}

void usbpdCapabilitiesReceived(void)
{
  char pdoStr[PDO_STR_LEN];

  Serial.println("source capabilities received:");

  size_t n = usbpd.sourcePDOCount();
  for (size_t i = 0U; i < n; ++i) {
    PDO pdo = usbpd.sourcePDO(i);
    snprintf(pdoStr, PDO_STR_LEN, "  %u: %umV %umA",
        pdo.number, pdo.voltage_mV, pdo.current_mA);
    Serial.println(pdoStr);
  }

  Serial.println("sink capabilities:");

  size_t m = usbpd.sinkPDOCount();
  for (size_t i = 0U; i < m; ++i) {
    PDO pdo = usbpd.sinkPDO(i);
    snprintf(pdoStr, PDO_STR_LEN, "  %u: %umV %umA",
        pdo.number, pdo.voltage_mV, pdo.current_mA);
    Serial.println(pdoStr);
  }
}

void setup()
{
  while (!Serial && (millis() < 2000)) { continue; }
  Serial.begin(115200);

  Serial.println("initializing");

  usbpd.setCableAttached(usbpdCableAttached);
  usbpd.setCableDetached(usbpdCableDetached);
  usbpd.setSourceCapabilitiesReceived(usbpdCapabilitiesReceived);

  if (usbpd.begin(USBPD_ALRT_PIN, USBPD_ATCH_PIN)) {
    Serial.print("STUSB4500 v");
    Serial.println(usbpd.version());

    // set power to default USB (5V 1.5A) initially
    usbpd.setPowerDefaultUSB();
  }
  else {
    Serial.println("failed to initialize STUSB4500");
    while (1) { delay(1000); }
  }
}

void loop()
{
  // process interrupts
  usbpd.update();
}
