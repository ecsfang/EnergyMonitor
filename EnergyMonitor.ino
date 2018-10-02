#include <Arduino.h>
#include <Wire.h>
#include <EspM24sr.h>
#include <String.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include<stdlib.h>

#include <PubSubClient.h>
#include <SoftwareSerial.h>

#define LED 6
#define INTERRUPT_INPUT 2  //external interupt pin2 (endast pin 2 och 3 valbara)

// This length is dependent on the URI as defined in the android app
#define ANDROID_URI_LENGTH    50

EthernetClient ethClient;
PubSubClient client(ethClient);
SoftwareSerial Serial7Segment(7, 8);

unsigned long previousMillis36 = 0; 
unsigned long currentMillis36 = millis();
unsigned long previousMillis60 = 0; 
unsigned long currentMillis60 = millis();
unsigned long previousMillis24 = 0; 
unsigned long currentMillis24 = millis();

//Used to measure power.
unsigned long pulseTime,lastTime;

//power and energy
double power, oPower, elapsedkWh;
int p,op;

//Number of pulses per wh - found or set on the meter.
int ppwh = 1; //1000 pulses/kwh = 1 pulse per wh

unsigned long lastPeriod = millis();
unsigned long period = 0;


//styr millisekunder mellan varje mätperiod och RF transmittion
//36 sekunder= 1/100 timme, elmätaren ger 1000p/h alltå är 10 pulser 1kW
float kW36 =0; //uppmätt kWh senaste 36 sekunder
float kW60 =0; //uppmätt senaste 60 minuter
float kW24=0;  //uppmätt senaste 24h
volatile int pulse_counter36 =0;
volatile int pulse_counter60 =0;
volatile long pulse_counter24 =0;
int i=0;

// Update these with values suitable for your network.
// Array for holding the mac address
byte mac[6];

// IP addresses used by the system
IPAddress ip;
IPAddress netmask;
IPAddress gateway;
uint16_t port;

// This is the data structure in the NFC Tag EEPROM
struct target_board_data {
  uint8_t board_type;
  uint8_t dhcp_enabled;
  uint8_t ip[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
  uint16_t port;
};

IPAddress mqttServer(192, 168, X, X);

// The UnoNet+ has pin 3 assigned as its GPO pin
// Make sure the JP4 bridge has been shorted on the PCB.
#define GPO_PIN    3

// Instances.
EspM24SR m24sr;
struct cc_file_layout *ccfl;
struct system_file *sf;
struct target_board_data *board_data;

void printPayload(char* msg, size_t len)
{
  int i = 0;
  
  while (len) {
    unsigned char val;
    Serial.print(F("Msg["));
    Serial.print(i++);
    Serial.print(F("] = "));
    val = (unsigned char)*msg++;
    if (val < 10) Serial.write(' ');
    if (val < 100) Serial.write(' ');
    Serial.print(val, DEC);
    Serial.print(F(" - "));
    Serial.write(val);
    Serial.println();
    len--;
  }
}

void printMacAddress(byte *mac)
{
  byte cnt = 0;
  while (cnt < 6) {
    if (mac[cnt] < 0x10) Serial.write('0');
    Serial.print(mac[cnt], HEX);
    if (cnt < 5) Serial.write(':');
    cnt++;
  }
}

/*
 * This interrupt will be triggered each time a phone or tablet
 * writes new data to the NFC EEPROM.
 */
void nfcWrite(void)
{
  // The NFC Tag has issued an interrupt. We will simply reset 
  // using the watchdog to restart the system. After restart the
  // system will be start with the new values.
  wdt_enable(WDTO_1S);
  while(1);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("power/meter","hello world");
      // ... and resubscribe
      client.subscribe("power/inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  size_t length;
  // initialize digital pin 13 as an output.
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(10);
  
  Serial.begin(115200);
  Serial.println("\nEnergy Meter 0.3");

  // First disable watchdog, in case it was enabled above.
  wdt_disable();

  // Initialize the ms24sr library and notify what pin should be used
  // for signaling.
  m24sr.begin(GPO_PIN);
  // Set GPO mode 0x20, this mode will trigger the GPO pin each time new
  // data has been written to the memory by the NFC controller
  m24sr.writeGPO(0x20);
  
  // Interrupt 1 is connected to the GPO pin from the NFC Tag device.
  attachInterrupt(1, nfcWrite, RISING);

  // Get the CC and system file from the device.
  // It us mandatory to get these before retrieving the ndef record
  ccfl = m24sr.getCCFile();
  sf = m24sr.getSystemFile();
  
  // Use the UID from the NFC memory to create a mac address
  // The UID is 7 bytes long and we are using the 6 lowest bytes to
  // create the mac address. This will ensure that every device in a
  // local network has a unique mac address.
  memcpy(mac, sf->uid+1, 6);
  Serial.print(F("MAC Address: "));
  printMacAddress(mac);
  Serial.println();
  
  // Now get the NDEF record with ndef data
  char *m24sr_ndef = (char *)m24sr.getNdefMessage(&length);
  // Our board data is located in the ndef record offsetted by the URI length
  board_data = (struct target_board_data *)(m24sr_ndef + ANDROID_URI_LENGTH);

  // Copy NFC Tag EEPROM content to our local variables
  ip = board_data->ip;
  //netmask = board_data->netmask;
  //gateway = board_data->gateway;
//  ethClient.setPort(board_data->port);
  
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);

  Serial.print("Connected as ");
  Serial.println(Ethernet.localIP());
  
  client.setServer(mqttServer, 1883);
  //client.setCallback(callback);

  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  delay(5000);
  
  while(!client.connected())
    reconnect();
  
  digitalWrite(LED, LOW);
  
  Serial7Segment.begin(9600); //Talk to the Serial7Segment at 9600 bps
  Serial7Segment.write('v'); //Reset the display - this forces the cursor to return to the beginning of the display
  Serial7Segment.print("0000");
  
  digitalWrite(INTERRUPT_INPUT, HIGH); // enable external interupts
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_INPUT),interrupt_handler,FALLING);
 }
/*
void show(float mom)
{
  char buf[16];
  dtostrf(mom,4,4,buf);
  if( mom < 10.0) {
    buf[1] = buf[2];
    buf[2] = buf[3];
    buf[3] = buf[4];
    buf[4] = 0;
    Serial7Segment.print(buf);
    Serial7Segment.write(0x77);
    Serial7Segment.write(0x01);
  } else if( mom < 100.0) {
    buf[2] = buf[3];
    buf[3] = buf[4];
    buf[4] = 0;
    Serial7Segment.print(buf);
    Serial7Segment.write(0x77);
    Serial7Segment.write(0x02);
  }
}*/
// the loop function runs over and over again forever
void loop()
{
  char buf[16];
  bool bUpdate = false;
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  noInterrupts();
  p = (int)(power*100);
  interrupts();

  // Only report changes larger than 100W ...
  if( abs(p-op) > 10 ) {
    dtostrf(power,5,2,buf);
    client.publish("power/meter/moment",buf);
    op = p;
  }
  
  // mät under 10 sekunder
  /*currentMillis10 = millis();
  if(currentMillis10 - previousMillis10 > 10000) {
      previousMillis10 = currentMillis10;
   
      //out to RS232 
      kW10=3.6*pulse_counter10/10.0;
      pulse_counter10=0;
      show(kW10);
  }*/
  // mät under 36 sekunder
  currentMillis36 = millis();
  if(currentMillis36 - previousMillis36 > 36000) {
      previousMillis36 = currentMillis36;
   
      //out to RS232 
      kW36=pulse_counter36/10.0;
      pulse_counter36=0;

      dtostrf(kW36,8,2,buf);
      client.publish("power/meter/kWh",buf);
      bUpdate = true;
      //show(kW36);
}
  
  
// mät under 60 minuter
 currentMillis60 = millis();
 if(currentMillis60 - previousMillis60 > 3600000) {
      previousMillis60 = currentMillis60;
   
      //out to RS232 
      kW60=pulse_counter60/1000.00;
      pulse_counter60=0;

      dtostrf(kW60,8,2,buf);
      client.publish("power/meter/kWh60",buf);
      //dtostrf(kW24,8,2,buf);
      //client.publish("power/meter/kWh24",buf);
      bUpdate = true;      
  }
  // mät under 24 timmar
  currentMillis24 = millis();
  if(currentMillis24 - previousMillis24 > 3600000*24) {
      previousMillis24 = currentMillis24;
   
      //out to RS232 
      kW24=pulse_counter24/1000.00;
      pulse_counter24=0;
         
      dtostrf(kW24,8,2,buf);
      client.publish("power/meter/kWh24",buf);
      bUpdate = true;      
  }

  if( bUpdate ) {
    Serial.print(kW36,1);
    Serial.print("kW  1h:");
    Serial.print(kW60,2);
    Serial.print("kWh  24h:");
    Serial.print(kW24,1);
    Serial.print("  Pulser ");
    Serial.print(pulse_counter36);
    Serial.print(" / ");
    Serial.print(pulse_counter60);
    Serial.print(" / ");
    Serial.println(pulse_counter24);
  }
}

// pulse countern interrupt handler
void interrupt_handler()
{
  //used to measure time between pulses.
  lastTime = pulseTime;
  pulseTime = micros();

  if( ((pulseTime - lastTime)/10000) < 1 )
    return;

  digitalWrite(LED,HIGH); //blinka LED

  //Calculate power
  power = (3600000.0 / (pulseTime - lastTime))/ppwh;
  if( power > 25) {
    power = 25;
  }
/*  if( power > 10) {
    Serial.print("Power = (3600000.0 / (");
    Serial.print(pulseTime);
    Serial.print(" - ");
    Serial.print(lastTime);
    Serial.print("))/");
    Serial.print(ppwh);
    Serial.println(";");
  }*/
//  show(power);

//pulse_counter10 = pulse_counter10 + 1;
  pulse_counter36 = pulse_counter36 + 1;
  pulse_counter60 = pulse_counter60 + 1;
  pulse_counter24 = pulse_counter24 + 1;
  delayMicroseconds(5000); //delay fungerar ej i interuptrutin
  digitalWrite(LED,LOW);
}
