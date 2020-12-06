#pragma once

#if defined (ESP32) || defined(ESP8266)
#include "compile_config.h"

#define PROGNAME               " SIGNALESP JH "
#define VERSION_1              0x34
#define VERSION_2              0x1f
#define BAUDRATE               115200
#define FIFO_LENGTH            200

#define ETHERNET_PRINT
#define WIFI_MANAGER_OVERRIDE_STRINGS

// EEProm Addresscommands
#define EE_MAGIC_OFFSET        0
#define addr_features          0xff
#define MAX_SRV_CLIENTS        2

#include "compile_config.h"

const char* ssid = "<ssid>";
const char* password = "<password>";

void serialEvent();
void ICACHE_RAM_ATTR cronjob(void *pArg);
int freeRam();
inline void ethernetEvent();

//unsigned long getUptime();
//void enDisPrint(bool enDis);
//void getFunctions(bool *ms, bool *mu, bool *mc);
//void initEEPROM(void);
uint8_t rssiCallback() { return 0; }; // Dummy return if no rssi value can be retrieved from receiver
size_t writeCallback(const uint8_t *buf, uint8_t len = 1);
void ICACHE_RAM_ATTR sosBlink(void *pArg);

volatile uint32_t net_con_count = 0;
volatile uint32_t broadcast_count = 0;
volatile uint32_t loop_count = 0;

#if defined(ESP8266)
extern "C" {
#include "user_interface.h"
}
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#endif
#if defined(ESP32)
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <WiFiType.h>
#endif

#include <FS.h>   
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include "ArduinoJson.h"     //Local WebServer used to serve the configuration portal


#include "output.h"
#include "bitstore.h"  // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#include "SimpleFIFO.h"

#ifdef CMP_CC1101
#include "cc1101.h"
#include <SPI.h>      // prevent travis errors
#endif

SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#include "signalDecoder.h"
#include "commands.h"
#include "functions.h"
#include "send.h"
#include "FastDelegate.h" 
#define WIFI_MANAGER_OVERRIDE_STRINGS
#include "wifi-config.h"
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager

IPAddress staticIP(192,168,0,60);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

void eventWiFi(WiFiEvent_t event);

WiFiServer Server(23);  //  port 23 = telnet
WiFiClient serverClient;

SignalDetectorClass musterDec;

#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";
volatile unsigned long lastTime = micros();

#ifdef ESP32
esp_timer_create_args_t cronTimer_args;
esp_timer_create_args_t blinksos_args;
esp_timer_handle_t cronTimer_handle;
esp_timer_handle_t blinksos_handle;
#elif defined(ESP8266)
os_timer_t cronTimer;
os_timer_t blinksos;
os_timer_t watchjh;
#endif

bool hasCC1101 = false;
char IB_1[14]; // Input Buffer one - capture commands


const char sos_sequence[] = "0101010001110001110001110001010100000000";
const char boot_sequence[] = "00010100111";


void ICACHE_RAM_ATTR sosBlink (void *pArg) {

	static uint8_t pos = 0;
	const char* pChar;
	pChar = (const char*)pArg; //OK in both C and C++


	digitalWrite(PIN_LED, pChar[pos] == '1' ? HIGH : LOW);
	pos++;
	if (pos == sizeof(pChar) * sizeof(pChar[1]))
		pos = 0;

}

void ICACHE_RAM_ATTR watchJH (void *pArg) {

	if (net_con_count > 22000000 
		|| broadcast_count > 9000
		|| loop_count < 1000000) {
   		ESP.restart();
	}

	Serial.printf("Counter: %d %d %d\n",net_con_count,broadcast_count,loop_count);
	loop_count = 0;
}

void setup() {

ESP.wdtEnable(2000);
//ESP.wdtDisable();
#ifdef ESP32
	blinksos_args.callback = sosBlink;
	blinksos_args.dispatch_method = ESP_TIMER_TASK;
	blinksos_args.name = "blinkSOS";
	blinksos_args.arg = (void *)boot_sequence;
	esp_timer_create(&blinksos_args, &blinksos_handle);
	esp_timer_start_periodic(blinksos_handle, 300000);
#elif defined(ESP8266)
	os_timer_setfn(&blinksos, &sosBlink, (void *)boot_sequence);
	os_timer_arm(&blinksos, 300, true);
	os_timer_setfn(&watchjh, &watchJH, (void *)boot_sequence);
	os_timer_arm(&watchjh, 60000, true);
#endif

//	WiFi.setAutoConnect(false);

  WiFi.config(staticIP, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent(eventWiFi); 
  WiFi.begin(ssid, password);

	Serial.begin(BAUDRATE);
	Serial.setDebugOutput(true);
	while (!Serial)
		delay(90);

	Serial.println("\n\n");

	pinMode(PIN_RECEIVE, INPUT);
	pinMode(PIN_LED, OUTPUT);
  
	#ifdef CMP_CC1101
	cc1101::setup();
	#endif

	initEEPROM();

#ifdef CMP_CC1101
	DBG_PRINT(FPSTR(TXT_CCINIT));
	cc1101::CCinit();					 // CC1101 init
	hasCC1101 = cc1101::checkCC1101();	 // Check for cc1101

	if (hasCC1101)
	{
		DBG_PRINT(FPSTR(TXT_CC1101));
		DBG_PRINTLN(FPSTR(TXT_FOUND));
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
	}
	else {
		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
	}
#endif 

/*
		1. starts a config portal in access point mode (timeout 60 seconds)
		2. Ii config portal times out, try connecting to a previous stored ssid in client mode
		3. if no connection is possible, start wps mode 
		If wps connection is successfull, ip address is retrieved via dhcp and saved. Otherwise esp is reseted and we start at 1. again  

		ip configuration can be switched between static and dhcp mode
*/
//	IPAddress _ip, _gw, _sn;

	//bool wps_successfull=false;
	Serial.printf("Starting config portal with SSID: %s\n", ssid);
//	wifiManager.autoConnect("NodeDuinoConfig",NULL);


#ifdef ESP32
	cronTimer_args.callback = cronjob;
	cronTimer_args.name = "cronTimer";
	cronTimer_args.dispatch_method = ESP_TIMER_TASK;
	esp_timer_create(&cronTimer_args, &cronTimer_handle);
#elif defined(ESP8266)
	os_timer_disarm(&cronTimer);
	os_timer_setfn(&cronTimer, &cronjob, 0);
#endif

	musterDec.setStreamCallback(writeCallback);
#ifdef CMP_CC1101
	if (!hasCC1101 || cc1101::regCheck()) {
#endif
		enableReceive();
		DBG_PRINT(FPSTR(TXT_RECENA));
#ifdef CMP_CC1101
	}
	else {
		DBG_PRINT(FPSTR(TXT_CC1101));
		DBG_PRINT(FPSTR(TXT_DOFRESET));
		DBG_PRINTLN(FPSTR(TXT_COMMAND));
	}
#endif
	MSG_PRINTER.setTimeout(400);

#ifdef ESP32
	esp_timer_start_periodic(cronTimer_handle, 31000);
	esp_timer_stop(blinksos_handle);
#elif defined(ESP8266)
	os_timer_arm(&cronTimer, 31, true);
	os_timer_disarm(&blinksos);
#endif
	pinAsOutput(PIN_SEND);
	digitalLow(PIN_LED);
}

void ICACHE_RAM_ATTR cronjob(void *pArg) {

	cli();
	static uint8_t cnt = 0;

	const unsigned long  duration = micros() - lastTime;
	long timerTime = maxPulse - duration + 1000;
	if (timerTime < 1000)
	    timerTime=1000;
    
#ifdef ESP32
	esp_timer_stop(cronTimer_handle);
	esp_timer_start_periodic(cronTimer_handle, timerTime);
#elif defined(ESP8266)
	os_timer_disarm(&cronTimer);
	os_timer_arm(&cronTimer, timerTime / 1000, true);
#endif


	if (duration > maxPulse) { //Auf Maximalwert pruefen.
		int sDuration = maxPulse;
		if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			sDuration = -sDuration;
		}
		FiFo.enqueue(sDuration);
		lastTime = micros();
	}
	else if (duration > 10000) {
		//os_timer_disarm(&cronTimer);
		//os_timer_arm(&cronTimer, 20, true);
	}
#ifdef PIN_LED_INVERSE	
	digitalWrite(PIN_LED, !blinkLED);
#else
	digitalWrite(PIN_LED, blinkLED);
#endif
	blinkLED = false;

	sei();
	// Infrequent time uncritical jobs (~ every 2 hours)
	if (cnt++ == 0)  // if cnt is 0 at start or during rollover
		getUptime();

}


void loop() {
	static int aktVal = 0;
	bool state = false;

	if (!serverClient || !serverClient.connected() || (WiFi.status() != WL_CONNECTED)) {
		net_con_count++;
	}
  	else {
		net_con_count = 0;
	}

//	Serial.printf("cnt: %d %d\n",net_con_count, broadcast_count);

//	wifiManager.process();
	
	serialEvent();
	ethernetEvent();

	while (FiFo.count()>0) { //Puffer auslesen und an Dekoder uebergeben
		aktVal = FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (state) blinkLED = true; //LED blinken, wenn Meldung dekodiert
		if (!state) broadcast_count++;
		else broadcast_count = 0;
		if (FiFo.count()<120) yield();
	}

	loop_count++;

}

//============================== Write callback =========================================

#define _USE_WRITE_BUFFER

#ifdef _USE_WRITE_BUFFER
const size_t writeBufferSize = 128;
size_t writeBufferCurrent = 0;
uint8_t writeBuffer[writeBufferSize];
#endif
size_t writeCallback(const uint8_t *buf, uint8_t len)
{
#ifdef _USE_WRITE_BUFFER
	if (!serverClient || !serverClient.connected())
		return 0;

	size_t result = 0;

	while (len > 0) {
		size_t copy = (len > writeBufferSize - writeBufferCurrent ? writeBufferSize - writeBufferCurrent : len);
		if (copy > 0)
		{
			memcpy(writeBuffer + writeBufferCurrent, buf, copy);
			writeBufferCurrent = writeBufferCurrent + copy;
		}
		// Buffer full or \n detected - force send
		if ((len == 1 && *buf == char(0xA)) || (writeBufferCurrent == writeBufferSize))
		{
			size_t byteswritten = 0;
			if (serverClient && serverClient.connected()) {
				byteswritten = serverClient.write(writeBuffer, writeBufferCurrent);
			}

			if (byteswritten < writeBufferCurrent) {
				memmove(writeBuffer, writeBuffer + byteswritten, writeBufferCurrent - byteswritten);
				writeBufferCurrent -= byteswritten;
			}
			else {
				writeBufferCurrent = 0;
			}
			result += byteswritten;
		}
		// buffer full
		len = len - copy;
		if (len > 0)
		{
			memmove((void*)buf, buf + copy, len);
		}
	}
	return len;
#else

	while (!serverClient.available()) {
		yield();
		if (!serverClient.connected()) return 0;
	}
	DBG_PRINTLN("Called writeCallback");

	memccpy()

		return serverClient.write(buf, len);
	//serverClient.write("test");
#endif
}


inline void ethernetEvent()
{
	//check if there are any new clients
	if (Server.hasClient()) {
		
		if (!serverClient || !serverClient.connected()) {
			if (serverClient) serverClient.stop();
			serverClient = Server.available();
			serverClient.flush();
			//DBG_PRINTLN("New client: ");
			//DBG_PRINTLN(serverClient.remoteIP());
		} else {
			WiFiClient rejectClient = Server.available();
            rejectClient.stop();
			//DBG_PRINTLN("Reject new Client: ");
			//DBG_PRINTLN(rejectClient.remoteIP());
		}
	}

	if(serverClient && !serverClient.connected())
	{
		//DBG_PRINTLN("Client disconnected: ");
		//DBG_PRINTLN(serverClient.remoteIP());
		serverClient.stop();
	}
}

void serialEvent()
{
	static uint8_t idx = 0;
	while (MSG_PRINTER.connected() > 0 && MSG_PRINTER.available())
	{
		if (idx == 14) {
			// Short buffer is now full
			MSG_PRINT("Command to long: ");
			MSG_PRINTLN(IB_1);
			idx = 0;
			return;
		}
		else {
			IB_1[idx] = (char)MSG_PRINTER.read();
			// DBG_PRINTLN(idx);
			// DBG_PRINT(IB_1[idx]);

			switch (IB_1[idx])
			{
			case '\n':
			case '\r':
			case '\0':
			case '#':
#if defined(ESP32)
				esp_task_wdt_reset();
				yield();
#elif defined(ESP8266)
				wdt_reset();
#endif
				if (idx > 0) {
					// DBG_PRINT("HSC");
					commands::HandleShortCommand();  // Short command received and can be processed now
				}
				idx = 0;
				return; //Exit function
			case ';':
				DBG_PRINT("send cmd detected ");
				// DBG_PRINTLN(idx);
				IB_1[idx + 1] = '\0';
				if (idx > 0)
					send_cmd();
				idx = 0; // increments to 1
				return; //Exit function
			case '!':
        		ESP.restart();
			}
			idx++;
		}
		yield();
	}
}


int freeRam() {
#ifdef ESP32
	return ESP.getFreeHeap();
#elif defined(ESP8266)
	return system_get_free_heap_size();
#endif
}

/********************************************************
/*  Handle WiFi events                                  *
/********************************************************/
void eventWiFi(WiFiEvent_t event) {

	Serial.println(event);
     
  switch(event) {
    case WIFI_EVENT_STAMODE_CONNECTED:
      Serial.println("EV1");
    break;
    
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      Serial.println("EV2");
      ESP.restart();
    break;
    
     case WIFI_EVENT_STAMODE_AUTHMODE_CHANGE:
      Serial.println("EV3");
    break;
    
    case WIFI_EVENT_STAMODE_GOT_IP:
      Serial.println("EV4");
	  Server.begin();  // start telnet server
	  Server.setNoDelay(true);
    break;
    
    case WIFI_EVENT_STAMODE_DHCP_TIMEOUT:
      Serial.println("EV5");
      ESP.restart();
    break;
    
    case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
      Serial.println("EV6");
	  Server.begin();  // start telnet server
	  Server.setNoDelay(true);
    break;
    
    case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
      Serial.println("EV7");
      ESP.restart();
    break;
    
    case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
      Serial.println("EV8");
    break;
  }
}

#endif
