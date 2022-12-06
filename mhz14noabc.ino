#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>


// Initialize display
#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


#define INTERVAL 5000
#define MH_Z19_RX A0 //RX
#define MH_Z19_TX A1 //TX

// Adapted from https://www.letscontrolit.com/forum/viewtopic.php?t=1785&start=40
// but I dont believe in single point calibration (two point is much more legit) so I disabled Automatic Baseline Calibration (ABC) as much as possible
// can be verified by running the setup for more than 24h in a >400ppm environment and verifying a drop due to ABC doesnt occur
// calibration is still doable by connecting pins on the sensor
// other changes: dont delay for 3 minutes, send co2 value immediately, we can ignore those readings on the PC, its nice to see the warming up happen

byte mhzResp[9];    // 9 bytes bytes response
byte mhzCmdReadPPM[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
byte mhzCmdCalibrateZero[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
byte mhzCmdABCEnable[9] = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6};
byte mhzCmdABCDisable[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86};
byte mhzCmdReset[9] = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};
byte mhzCmdMeasurementRange1000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
byte mhzCmdMeasurementRange2000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
byte mhzCmdMeasurementRange3000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
byte mhzCmdMeasurementRange5000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};

int shifts=0,co2ppm;

long previousMillis = 0;

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z19

byte checksum(byte response[9]){
  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += response[i];
  }
  crc = 255 - crc + 1;
  return crc;
}


void disableABC() {
 co2Serial.write(mhzCmdABCDisable, 9);
}

int readCO2() {
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9];
  co2Serial.write(cmd, 9);
  // The serial stream can get out of sync. The response starts with 0xff, try to resync.
  while (co2Serial.available() > 0 && (unsigned char)co2Serial.peek() != 0xFF) {
    co2Serial.read();
    shifts++;
  }

  memset(response, 0, 9);
  co2Serial.readBytes(response, 9);

for (int i=0; i<9; i++) {
  Serial.print(" 0x");
  Serial.print(response[i], HEX);
}
Serial.println(" Response OK. Shifts="+String(shifts));


  if (response[1] != 0x86)
  {
    Serial.println(" Invalid response from co2 sensor!");
    return -1;
  }

  if (response[8] == checksum(response)) {
    int responseHigh = (int) response[2];
    int responseLow = (int) response[3];
    int ppm = (256 * responseHigh) + responseLow;
    return ppm;
  } else {
    Serial.println("CRC error!");
    return -1;
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long previousMillis = millis();
  co2Serial.begin(9600); //Init sensor MH-Z19(14)
  
  // set the cursor to column 0, line 0, counting begins at zero
  lcd.setCursor(0, 1);
  lcd.print("Starting up");
  
  delay(500);
  disableABC();
}

void loop() {
  unsigned long currentMillis = millis();
  if (abs(currentMillis - previousMillis) > INTERVAL)
  {
          previousMillis = currentMillis;
          Serial.print("Requesting CO2 concentration...");
          co2ppm=-999;
          co2ppm = readCO2();
          Serial.println("  PPM = " + String(co2ppm));

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(co2ppm);
          lcd.setCursor(13, 0);
          lcd.print("PPM");
  }
}
