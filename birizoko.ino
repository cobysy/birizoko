#include <avr/wdt.h>
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include <OneWire.h>
//#include <DallasTemperature.h>
#include <SoftwareSerial.h>

#define rxPin 255         // Not used, so set to invalid pin #
#define txPin 3           // Hook SER input to Arduino pin 3.
#define inverted 1       // Inverted serial (like RS-232, but TTL levels)
#define noninverted 0    // Noninverted serial (like UART output)

// Change the definition of SPOL to inverted or noninverted
// depending on your display configuration. On newer BPP-440Ls,
// if the SPOL jumper is intact (factory setting), you want
// "inverted." If it's cut, use "noninverted."
#define SPOL inverted

// For inverted serial, the output pin must be LOW when data is
// not being sent; for noninverted, HIGH.
#if SPOL
#define STOPBIT LOW
#else
#define STOPBIT HIGH
#endif

//=====================================================================
//          CONSTANTS FOR LCD INSTRUCTIONS
//
// Rather than scatter a bunch of magic numbers throughout the
// program, I've defined some self-explanatory names for the
// small subset of BPP-440L instructions needed.
#define HOME (char) 1        // Moveto character position 0.
#define BIGMODE (char) 2     // Begin big-character mode.
#define ENDBIG (char) 3      // End big-char mode. 
#define CLS (char) 12        // Clear screen, moveto 0.
#define LF (char) 10         // Line feed. 
#define CR (char) 13         // Carriage return. 
#define MOVETO (char) 16     // Position instruction
#define MIDSCREEN (char) (20+64)  // Position addres of col 20

// In the BPP-440L Big-character mode, the space character " "
// (32 dec, 0x20 hex) is three columns wide, while A-Z and
// 0-9 are five columns. So to make a big space that is
// suitable for padding for alignment, let's define a
// "bigSpace" of five clear-column instructions.
char const bigSpace[] = {
  0x11, 0x11, 0x11, 0x11, 0x11, 0x00
};

char const celcius[] = {
  0x1B, 0x44, 0x36, // ESC, D, '6' Custom-character 6 (which maps to character code 134)
  0x88, 0x94, 0x88, 0x82, 0x85, 0x84, 0x85, 0x82
};

char const crlf[] = "\r\n";

//=====================================================================
// Set up a new serial output using the definitions above.
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin, SPOL);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ds(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature sensors(&oneWire);
//
//// arrays to hold device addresses
//DeviceAddress insideThermometer, outsideThermometer;

#define MAX_DS1820_SENSORS 2
byte addr[MAX_DS1820_SENSORS][8];

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "dd2Point4"           // cannot be longer than 32 characters!
#define WLAN_PASS       "silentgrasshopper542"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
// received before closing the connection.  If you know the server
// you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "www.adafruit.com"
#define WEBPAGE      "/testwifi/index.html"


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/
double currentTemp = 99.0;
#define  setTemp 70.00

#define COOLING 1
#define RESTING 0
#define precision 0.3 //.5 degrees precision

#define minOn 120 //2 mins
#define maxOn 3600 //1h
#define minOff 600 //10m

int rest_count;
int cool_count;
byte fridge_status;
#define startup_time 10000

#define fridge_relay 5

bool tooWarm() {
  return (currentTemp >= (setTemp + precision));
}

bool restLongEnough() {
  return (rest_count >= minOff);
}

bool tooCold() {
  return (currentTemp <= setTemp); //temp keeps dropping after temp is reached
}

bool onLongEnough() {
  return (cool_count >= minOn);
}

bool onTooLong() {
  return (cool_count >= maxOn);
}

uint32_t ip = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for Leonardo only

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  fridge_status = RESTING;
  rest_count = minOff;
  cool_count = 0;

  digitalWrite(txPin, STOPBIT);  // Preset pin to stop-bit
  mySerial.begin(9600);          // Set the data rate.

  // Start up the library

//  sensors.begin();
//  if (!sensors.getAddress(insideThermometer, 0)) Serial.println(("No temp probe 0"));
//  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println(("No temp probe 1"));
//  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
//  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

  for (int i = 0; i < MAX_DS1820_SENSORS; i++)
  {
    if (!ds.search(addr[i])) 
    {
      Serial.print("No temp probe "); Serial.print(i);
      ds.reset_search();
      return;
    }
  }
  

  /* Initialise the module */
  Serial.println(("Hello ..."));
  if (!cc3000.begin())
  {
    Serial.print("CC3000 failed!");
    while (1);
  }


  //  uint32_t ipAddress = cc3000.IP2U32(192, 168, 1, 19);
  //  uint32_t netMask = cc3000.IP2U32(255, 255, 255, 0);
  //  uint32_t defaultGateway = cc3000.IP2U32(192, 168, 1, 1);
  //  uint32_t dns = cc3000.IP2U32(8, 8, 4, 4);
  //  if (!cc3000.setStaticIPAddress(ipAddress, netMask, defaultGateway, dns)) {
  //    Serial.print("Failed to set static IP!");
  //    while (1);
  //  }

  Serial.print(("Connecting to wifi ..."));
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.print(("Failed!"));
    while (1);
  }

  Serial.println(("Connected!"));

  //  /* Wait for DHCP to complete */
  Serial.println(("Getting IP ..."));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }
  Serial.println(("Got IP!"));

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE);
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(("Couldn't resolve!"));
    }
    delay(500);
  }
}

void loop(void)
{
//  sensors.requestTemperatures();
//  printTemperature(insideThermometer);
//  printTemperature(outsideThermometer);

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if (!ds.search(addr))
  {
    // No more addresses
    ds.reset_search();
    control();
    delay(1000);
    return;
  }
 
  getTemp();
  lcdPrint();

  control();
  delay(1000);
}

void control()
{
  if (fridge_status == RESTING)
  {
    if (tooWarm() && restLongEnough())
    {
      //turn fridge on after temp has risen and waiting for compressor to cool down
      digitalWrite(fridge_relay, LOW);
      delay(2000);
      cool_count = 0;
      fridge_status = COOLING;
    }
    ++rest_count;
  }
  else
  { 
    //fridge status is COOLING
    if (onTooLong() || (onLongEnough() && tooCold()))
    {
      //turn the fridge off if its cool enough or compressor on too long
      digitalWrite(fridge_relay, HIGH);
      delay(2000);
      rest_count = 0;
      fridge_status = RESTING;
    }
    ++cool_count;
  }


  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    www.fastrprint(("GET "));
    www.fastrprint(WEBPAGE);
    www.fastrprint((" HTTP/1.1"));
    www.fastrprint((crlf));
    www.fastrprint(("Host: ")); www.fastrprint(WEBSITE); www.fastrprint((crlf));
    www.fastrprint((crlf));
    www.println();
  }
  else
  {
    Serial.println("Connection failed");
    reboot(); // couldn't connect to cached ip so reboot entire system
  }

  /* Read data until either the connection is closed, or the idle timeout is reached. */
  //  unsigned long lastRead = millis();
  //  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
  //    while (www.available()) {
  //      char c = www.read();
  //      Serial.print(c);
  //      lastRead = millis();
  //    }
  //  }
  www.close();
}

void getTemp()
{
  
  //sensors.requestTemperatures();
  //double t = sensors.getTempC(insideThermometer);
  for (sensor=0;sensor<MAX_DS1820_SENSORS;sensor++)
  {
    if ( OneWire::crc8( addr[sensor], 7) != addr[sensor][7]) 
    {
      Serial.print("No valid probe "); Serial.print(i);
      return;
    }
    
    if (t > 0 && t < 120)
    { //faulty wiring causes weird temperatures..
      currentTemp = t;
    }
  }
}

void lcdPrint()
{
  mySerial.println(ftos(currentTemp));
  mySerial.println(ftos(setTemp));
  if (fridge_status == COOLING) mySerial.print("*");
  else mySerial.print(" ");
}

char* ftos(float f)
{
  char sbuff[6];
  sbuff[0] = '0' + ((int)f / 10 % 10);
  sbuff[1] = '0' + ((int)f % 10);
  sbuff[2] = '.';
  sbuff[3] = '0' + ((int)(f * 10) % 10);
  sbuff[4] = '0' + ((int)(f * 100) % 10);
  sbuff[5] = 0;
  return sbuff;
}

//void printTemperature(DeviceAddress deviceAddress)
//{
//  float tempC = sensors.getTempC(deviceAddress);
//  Serial.print(("Temp C: "));
//  Serial.println(tempC);
//}

void reboot()
{
  wdt_enable(WDTO_15MS);
  while (1);
}

