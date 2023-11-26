/*
 KeyPressed with interrupt and digital write all
 from P4 to P7
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include "Arduino.h"
#include "PCF8574.h"
#include <Wire.h>
#include <jm_LCM2004A_I2C.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "password.h"


// init the lcd on addr 0x27
jm_LCM2004A_I2C lcd(0x27);
int lcd_row_counter = 0;


// For arduino uno only pin 1 and 2 are interrupted
#define ARDUINO_UNO_INTERRUPTED_PIN D3

// Function interrupt
void ICACHE_RAM_ATTR  keyPressedOnPCF8574();
// Set i2c address
PCF8574 pcf8574_bank2(0x21, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);
PCF8574 pcf8574_bank1(0x24, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);

// assign port values
// start with 1 as the relay is active on 1
PCF8574::DigitalInput digitalInputBank1;
PCF8574::DigitalInput digitalInputBank2;

char relayStatusArray[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// mqtt setup


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;



void processRelayStatus()
{
	if(relayStatusArray[0] == 0)  { digitalInputBank1.p0 = 1;} else { digitalInputBank1.p0 = 0;}
	if(relayStatusArray[1] == 0)  { digitalInputBank1.p1 = 1;} else { digitalInputBank1.p1 = 0;}
	if(relayStatusArray[2] == 0)  { digitalInputBank1.p2 = 1;} else { digitalInputBank1.p2 = 0;}
	if(relayStatusArray[3] == 0)  { digitalInputBank1.p3 = 1;} else { digitalInputBank1.p3 = 0;}
	if(relayStatusArray[4] == 0)  { digitalInputBank1.p4 = 1;} else { digitalInputBank1.p4 = 0;}
	if(relayStatusArray[5] == 0)  { digitalInputBank1.p5 = 1;} else { digitalInputBank1.p5 = 0;}
	if(relayStatusArray[6] == 0)  { digitalInputBank1.p6 = 1;} else { digitalInputBank1.p6 = 0;}
	if(relayStatusArray[7] == 0)  { digitalInputBank1.p7 = 1;} else { digitalInputBank1.p7 = 0;}

	if(relayStatusArray[8] == 0)  { digitalInputBank2.p0 = 1;} else { digitalInputBank2.p0 = 0;}
	if(relayStatusArray[9] == 0)  { digitalInputBank2.p1 = 1;} else { digitalInputBank2.p1 = 0;}
	if(relayStatusArray[10] == 0) { digitalInputBank2.p2 = 1;} else { digitalInputBank2.p2 = 0;}
	if(relayStatusArray[11] == 0) { digitalInputBank2.p3 = 1;} else { digitalInputBank2.p3 = 0;}

	pcf8574_bank2.digitalWriteAll(digitalInputBank2);
	pcf8574_bank1.digitalWriteAll(digitalInputBank1);
}

void printToLCD(char* message) {
	lcd.set_cursor(0,lcd_row_counter);
	lcd.print("                    ");
	lcd.set_cursor(0,lcd_row_counter);
	lcd_row_counter = (lcd_row_counter + 1) % 4;
	lcd.print(message);
}

void printRuntimeToLCD(char* message) {
	lcd.set_cursor(0,0);
	lcd.print("                    ");
	lcd.set_cursor(0,0);
	lcd.print(message);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  printToLCD("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  printToLCD("WiFi connected");
  Serial.println("IP address: ");
  printToLCD("IP address: ");
  Serial.println(WiFi.localIP());
  char* char_array = new char[20]; 
  strcpy(char_array,WiFi.localIP().toString().c_str());
  printToLCD(char_array);
}

void callback(char* topic, byte* payload, unsigned int length) {
  // string to lcd
  String messageTemp;
  char statusReport[10];
  String relay1 = "GardenAutomationLights/CMD/Relay1";
  String relay2 = "GardenAutomationLights/CMD/Relay2";
  String relay3 = "GardenAutomationLights/CMD/Relay3";
  String relay4 = "GardenAutomationLights/CMD/Relay4";
  String relay5 = "GardenAutomationLights/CMD/Relay5";
  String relay6 = "GardenAutomationLights/CMD/Relay6";
  String relay7 = "GardenAutomationLights/CMD/Relay7";
  String relay8 = "GardenAutomationLights/CMD/Relay8";
  String relay9 = "GardenAutomationLights/CMD/Relay9";
  String relay10 = "GardenAutomationLights/CMD/Relay10";
  String relay11 = "GardenAutomationLights/CMD/Relay11";
  String relay12 = "GardenAutomationLights/CMD/Relay12";

  String LCD1 = "GardenAutomationLights/LCD/ROW1";
  String LCD2 = "GardenAutomationLights/LCD/ROW2";
  String LCD3 = "GardenAutomationLights/LCD/ROW3";
  String LCD4 = "GardenAutomationLights/LCD/ROW4";

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
	messageTemp += (char)payload[i];
  }
  Serial.println();

  if(relay1.compareTo(topic) == 0)
  {
    Serial.println("New number received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[0] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[0] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay1", statusReport);
  }

  if(relay2.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[1] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[1] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay2", statusReport);
  }

  if(relay3.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[2] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[2] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay3", statusReport);
  }

  if(relay4.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[3] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[3] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay4", statusReport);
  }
  
  if(relay5.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[4] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[4] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay5", statusReport);
  }

  if(relay6.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[5] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[5] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay6", statusReport);
  }

  if(relay7.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[6] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[6] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay7", statusReport);
  }

  if(relay8.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[7] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[7] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay8", statusReport);
  }

  if(relay9.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[8] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[8] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay9", statusReport);
  }

  if(relay10.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[9] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[9] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay10", statusReport);
  }

  if(relay11.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[10] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[10] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay11", statusReport);
  }

  if(relay12.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	if(messageTemp.compareTo("ON") == 0) { relayStatusArray[11] = 1; strcpy(statusReport, "ON");}
	if(messageTemp.compareTo("OFF") == 0) { relayStatusArray[11] = 0; strcpy(statusReport, "OFF");}
	client.publish("GardenAutomationLights/STAT/Relay12", statusReport);
  }


  if(LCD1.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	lcd.set_cursor(0,0);
	lcd.print("                    ");
	lcd.set_cursor(0,0);
	lcd.print(messageTemp);
  }

  if(LCD2.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	lcd.set_cursor(0,1);
	lcd.print("                    ");
	lcd.set_cursor(0,1);
	lcd.print(messageTemp);
  }

  if(LCD3.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	lcd.set_cursor(0,2);
	lcd.print("                    ");
	lcd.set_cursor(0,2);
	lcd.print(messageTemp);
  }

  if(LCD4.compareTo(topic) == 0)
  {
    Serial.println("New text received on MQTT: " + messageTemp);
	lcd.set_cursor(0,3);
	lcd.print("                    ");
	lcd.set_cursor(0,3);
	lcd.print(messageTemp);
  }

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

  //process the messages that are arriving
  processRelayStatus();

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
	  client.subscribe("GardenAutomationLights/CMD/Relay1");
	  client.subscribe("GardenAutomationLights/CMD/Relay2");
	  client.subscribe("GardenAutomationLights/CMD/Relay3");
	  client.subscribe("GardenAutomationLights/CMD/Relay4");
	  client.subscribe("GardenAutomationLights/CMD/Relay5");
	  client.subscribe("GardenAutomationLights/CMD/Relay6");
	  client.subscribe("GardenAutomationLights/CMD/Relay7");
	  client.subscribe("GardenAutomationLights/CMD/Relay8");
	  client.subscribe("GardenAutomationLights/CMD/Relay9");
	  client.subscribe("GardenAutomationLights/CMD/Relay10");
	  client.subscribe("GardenAutomationLights/CMD/Relay11");
	  client.subscribe("GardenAutomationLights/CMD/Relay12");

	  //LCD handling
	  client.subscribe("GardenAutomationLights/LCD/ROW1");
	  client.subscribe("GardenAutomationLights/LCD/ROW2");
	  client.subscribe("GardenAutomationLights/LCD/ROW3");
	  client.subscribe("GardenAutomationLights/LCD/ROW4");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup()
{
	// initialize the serial port for debug
	Serial.begin(115200);
	delay(1000);
	Serial.println("Intializing the I2C ports...");
	
	// initialize the port for the relays
	pcf8574_bank1.pinMode(P0, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P1, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P2, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P3, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P4, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P5, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P6, OUTPUT, LOW);
	pcf8574_bank1.pinMode(P7, OUTPUT, LOW);

	pcf8574_bank2.pinMode(P0, OUTPUT, LOW);
	pcf8574_bank2.pinMode(P1, OUTPUT, LOW);
	pcf8574_bank2.pinMode(P2, OUTPUT, LOW);
	pcf8574_bank2.pinMode(P3, OUTPUT, LOW);


	digitalInputBank1.p0 = 1;
	digitalInputBank1.p1 = 1;
	digitalInputBank1.p2 = 1;
	digitalInputBank1.p3 = 1;
	digitalInputBank1.p4 = 1;
	digitalInputBank1.p5 = 1;
	digitalInputBank1.p6 = 1;
	digitalInputBank1.p7 = 1;

	digitalInputBank2.p0 = 1;
	digitalInputBank2.p1 = 1;
	digitalInputBank2.p2 = 1;
	digitalInputBank2.p3 = 1;


	Serial.print("Init pcf8574_bank1...");
	if (pcf8574_bank1.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}

	Serial.print("Init pcf8574_bank2...");
	if (pcf8574_bank2.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}

	Serial.println("Ready to start");

	pcf8574_bank2.digitalWriteAll(digitalInputBank2);
	pcf8574_bank1.digitalWriteAll(digitalInputBank1);

	// initialize the LCD display
	Wire.begin();
	lcd.begin();
	// wait for the lcd to boot up
	delay(500);

	
	printToLCD("Boot-up ...");
	printToLCD("I2C port init...");

	//setup the mqtt connection
	//pinMode(BUILTIN_LED, OUTPUT);   
	setup_wifi();
	client.setServer(mqtt_server, 1883);
	client.setCallback(callback);
}

void loop()
{
	if (!client.connected()) {
    reconnect();
	}
	client.loop();
}

void keyPressedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
}