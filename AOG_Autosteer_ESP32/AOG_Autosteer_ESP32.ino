// ESP32 code for autosteer unit for AgOpenGPS

//Version für Adafruit feather auf Fendt 209V
 

// PINs for GormR central unit PCB (V1.8) see GitHub https://github.com/GormR/HW_for_AgOpenGPS
// by MTZ8302 see GitHub https://github.com/mtz8302 and Youtube Ma Ha MTZ8302 https://www.youtube.com/channel/UCv44DlUXQJKbQjzaOUssVgw

byte vers_nr = 55;
char VersionTXT[120] = " - 17. Apr 2023 by MTZ8302<br>(AsyncUDP for WiFi, AgIO heartbeat, CMPS14/BNO085 via I2C and Ethernet support, new OTA)";

//##########################################################################################################
//### Setup Zone ###########################################################################################
//
//Set default values here. They can be changed in webinterface later. (x.x.x.77 or 192.168.1.1)
//
//If you don't get access to webinterface, use serial monitor with USB. IP address is displayed at start.
//
//##########################################################################################################

//to do: steerPostionZero from AOG is ignorred, only zero button in WebIO working 


#define useLED_BUILTIN  1//0	          // some ESP board have a build in LED, some not. Here it's the same funtion as the WiFi LED


struct Storage {
	//WiFi
	char ssid1[24] = "";					      // WiFi network Client name
	char password1[24] = "";                      // WiFi network password//Accesspoint name and password
	char ssid2[24] = "Fendt_209V_Tablet";	      // WiFi network Client name
	char password2[24] = "Fendt_209V";            // WiFi network password//Accesspoint name and password
	char ssid3[24] = "Fendt_209V";                // WiFi network Client name
	char password3[24] = "";                      // WiFi network password//Accesspoint name and password
	char ssid4[24] = "";                          // WiFi network Client name
	char password4[24] = "";                      // WiFi network password//Accesspoint name and password
	char ssid5[24] = "";                          // WiFi network Client name
	char password5[24] = "";                      // WiFi network password//Accesspoint name and password
	char ssid_ap[24] = "Autosteer_unit_Net";      // name of Access point, if no WiFi found, NO password!!
	uint16_t timeoutRouterWiFi = 120;             // time (seconds) to wait for WIFI access, after that own Access Point starts
	uint16_t timeoutRouterEth = 120;              // time (s) to search for cable Ethernet connection, when time has passed changing to WiFi data transfer
	byte timeoutWebIO = 255;                      // time (min) afterwards webinterface is switched off

	byte WiFi_myip[4] = { 192, 168, 137, 77 };    // autosteer module 
	byte WiFi_gwip[4] = { 192, 168, 137, 1 };     // Gateway IP only used if Accesspoint created
	byte WiFi_ipDest_ending = 255;                // ending of IP address to send UDP data to
	byte mask[4] = { 255, 255, 255, 0 };
	byte myDNS[4] = { 8, 8, 8, 8 };               //optional

	//Ethernet
	byte Eth_myip[4] = { 192, 168, 5, 78 };       // autosteer module 
	byte Eth_ipDest_ending = 255;                 // ending of IP address to send UDP data to
	byte Eth_mac[6] = { 0x70,0x69,0x69,0x2D,0x30,0x31 };
	bool Eth_static_IP = false;					  // false = use DHPC and set last number to 80 (x.x.x.80) / true = use IP as set above

	unsigned int PortAutostToAOG = 5577;          // this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
	unsigned int PortFromAOG = 8888;              // port to listen for AOG
	unsigned int PortDestination = 9999;          // port of AOG that listens
	uint16_t BaudRate = 38400;                    // Baudrate = speed of serial port or USB or Bluetooth. AOG uses 38400 for UART

	//general settings
	uint8_t AgIOHeartbeat_answer = 0;			  //0: don't answer (default for WiFi), IP is send in any case	
	uint8_t aogVersion = 20;			          // Version number not used at the moment

	byte DataTransVia = 7;                        // transfer data via 0 = USB / 7 = WiFi UDP / 10 = Ethernet UDP

	uint8_t output_type = 2;                      // set to 1  if you want to use Stering Motor + Cytron MD30C Driver
												  // set to 2  if you want to use Stering Motor + IBT 2  Driver
												  // set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
												  // set to 4  if you want to use  IBT 2  Driver + Danfoss Valve PVE A/H/M


	uint16_t PWMOutFrequ = 20000;                 // PWM frequency for motordriver: 1000Hz:for low heat at PWM device 20000Hz: not hearable

	uint8_t	MotorDriveDirection = 0;              // 0 = normal, 1 = inverted

	uint8_t MotorSlowDriveDegrees = 5;	          // How many degrees before decreasing Max PWM

	uint8_t WASType = 2;                          // 0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 36 (pin set below) (attention 3,3V only)
												  // 1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
												  // 2 = Differential Mode - Connect Sensor GND to A1, Signal to A0

	uint8_t IMUType = 0;                          // 0: none, 1: BNO055 IMU, 2: CMPS14, 3: BNO080 + BNO085

	//CMPS14	
	int CMPS14_ADDRESS = 0x60;                    // Address of CMPS14 shifted right one bit for arduino wire library
	float CMPS14HeadingCorrection = 0.0;		  // not used at the moment
	float CMPS14RollCorrection = 0.0;		      // not used at the moment

	//BNO08x
	uint8_t bno08xAddresses[2] = { 0x4A,0x4B };	  // BNO08x address variables to check where it is
	float BNOHeadingCorrection = 0.0;		      // not used at the moment
	float BNORollCorrection = 0.0;		          // not used at the moment
	uint8_t BNOUsedAxis = 0;		              // not used at the moment

	uint8_t InvertRoll = 0;                       // 0: no, set to 1 to change roll direction
	uint8_t InvertWAS = 0;                        // set to 1 to Change Direction of Wheel Angle Sensor - to + 

	uint8_t ShaftEncoder = 0;                     // Steering Wheel ENCODER Installed
	uint8_t PressureSensor = 0;		              // (not supported at the moment)
	uint8_t CurrentSensor = 0;		              // (not supported at the moment)
	uint8_t pulseCountMax = 3;                    // Switch off Autosteer after x Pulses from Steering wheel encoder 

	uint16_t WebIOSteerPosZero = 10300;//;	4096  // first value for steer zero position ADS: 11000 for EPS32 AD PIN: 2048

	uint8_t AckermanFix = 78;		              // if values for left and right are the same: 100 

	uint8_t SteerSwitchType = 1;                  // 0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
												  // 3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V
												  // 255 = no steer switch, allways on if AOG steering is active

	uint8_t WorkSW_mode = 2;                      // 0 = disabled // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)

	uint8_t Invert_WorkSW = 0;                    // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low

	float autoSteerMinSpeed = 0.2;                // Min speed to use autosteer km/h
	float autoSteerMaxSpeed = 30;                 // Max speed to use autosteer km/h

	uint16_t WorkSW_Threshold = 1600;             // Value for analog hitch level to switch workswitch  (0-4096)


	// IO pins ------------------------------------------------------------------

	// set to 255 for unused32 !!!!!
	uint8_t SDA = 21;	                       // I2C Pins
	uint8_t SCL = 22;

	uint8_t AutosteerLED_PIN = 2;             // light on active autosteer and IBT2
	uint8_t LEDWiFi_PIN = 0;     ;            // light on WiFi connected, flashes on searching Network. If GPIO 0 is used LED must be activ LOW otherwise ESP won't boot
	uint8_t LEDWiFi_ON_Level = HIGH;          // HIGH = LED on high, LOW = LED on low

	// (not supported at the moment) relais for section control
	uint8_t Relay_PIN[16] = { 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255 }; 
	uint8_t Tram_PIN[3] = { 255,255,255 };  // (not supported at the moment) relais for tramline control
	uint8_t Relays_ON = HIGH;		        // HIGH = Relay on high, LOW = Relay on low

	uint8_t WAS_PIN = 36;                // PIN for Wheel Angle Sensor (none, if ADS used)
	uint8_t WAS_Diff_GND_PIN = 39;
	uint8_t WORKSW_PIN = 33;            // PIN for workswitch (can be analog or on/off switch see WorkSW_mode)
	uint8_t STEERSW_PIN = 34;           // Pin for steer button or switch (see SteerSwitchType)
	uint8_t encA_PIN = 4;               // Pin for steer encoder, to turn off autosteer if steering wheel is used
	uint8_t encB_PIN = 32;              // Pin for steer encoder, to turn off autosteer if steering wheel is used

	uint8_t Servo_PIN = 16;	            // (not supported at the moment) Pin for servo to pull motor to steering wheel

	uint8_t PWM_PIN = 27;               // PWM Output to motor controller (IBT2 or cytron)
	uint8_t DIR_PIN = 26;               // direction output to motor controller (IBT2 or cytron)
	uint8_t Current_sens_PIN = 35;	    // (not supported at the moment) current sensor for IBT2 to read the force needed to turn steering wheel

	uint8_t Eth_CS_PIN = 5;             // CS PIN with SPI Ethernet hardware  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5

	uint8_t CAN_RX_PIN = 25;	        //(not supported at the moment) CAN bus 
	uint8_t CAN_TX_PIN = 17;	        //(not supported at the moment)

	//##########################################################################################################
	//### End of Setup Zone ####################################################################################
	//##########################################################################################################
	//filter variables set by AOG via PGN settings sentence
	float Ko = 0.05f;     //overall gain  
	float Kp = 20.0f;     //proportional gain  
	float Ki = 0.001f;    //integral gain
	float Kd = 1.0f;      //derivative gain 
	float AOGSteerPositionZero = 0;
	float steerSensorCounts = 68;
	uint16_t roll_corr = 200;
	byte minPWM = 40, highPWM = 150, lowPWM = 60;

	bool debugmode = false;
	bool debugmodeDataFromAOG = false;

};  Storage Set;

boolean EEPROM_clear = false;  //set to true when changing settings to write them as default values: true -> flash -> boot -> false -> flash again


// sentences to AOG V5 -------------------------------------------------------------------	
uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 }; //hello from SC sent back to AgIO

const byte FromAOGSentenceHeader[3] = { 0x80,0x81,0x7F };
#define AgIO_heartbeat 0xC8// = 0xC8
#define AgIO_ScanRequest 0xCA//AgIO asking for IP -> Ethernet/WiFi GPS sends IP 
#define steerDataToAOGHeader  0xFD
#define steerDataFromAOGHeader  0xFE 
#define steerArdConfFromAOGHeader 0xFB
#define steerSettingsFromAOGHeader  0xFC
#define steerDataSentenceToAOGLength  14

byte incomSentenceDigit = 0;
bool isSteerDataFound = false, isSteerSettingFound = false, isSteerArdConfFound = false, isSteerArdConfigFound = false;
bool isAgIOHeartbeatFound = false, isAgIOScanRequestFound = false;
bool SectRelaysUsed = false;

#define incommingDataArraySize 10
byte incommingBytes[incommingDataArraySize][512], incommingBytesArrayNr = 0, incommingBytesArrayNrToParse = 0;
unsigned int incommingDataLength[incommingDataArraySize] = { 0,0,0,0,0 };
#define SentenceFromAOGMaxLength 14
byte SentenceFromAOG[SentenceFromAOGMaxLength], SentenceFromAOGLength;

byte steerToAOG[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };


//libraries ---------------------------------------------------------------------------------------
#include "EEPROM.h"
#include <Update.h>
#include "Wire.h"
#include "BNO055_AOG.h"
#include <Adafruit_ADS1X15.h>
#include <AsyncUDP.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <Ethernet.h>
#include "BNO08x_AOG.h"
#include "zAOG_ESP32Ping.h"
#include "zAOG_Ping.h"


// Instances --------------------------------------------------------------------------------------
Adafruit_ADS1115 ads; 
BNO055 BNO(0X28);	//I2C address selection pin LOW BNO055
BNO080 bno08x;
AsyncUDP WiFiUDPFromAOG;
AsyncUDP WiFiUDPToAOG;

EthernetUDP EthUDPToAOG;
EthernetUDP EthUDPFromAOG;
WebServer WiFi_Server(80);

byte mainLoopDelay = 0;
TaskHandle_t taskHandle_Eth_connect;
TaskHandle_t taskHandle_WiFi_connect;
TaskHandle_t taskHandle_DataFromAOGUSB; bool USBDataTaskRunning = false;
TaskHandle_t taskHandle_DataFromAOGEth; bool EthDataTaskRunning = false;
TaskHandle_t taskHandle_WebIO;
TaskHandle_t taskHandle_LEDBlink;


// Variables --------------------------------------------------------------------------------------
// WiFi status LED blink times: searching WIFI: blinking 4x faster; connected: blinking as times set; data available: light on; no data for 2 seconds: blinking
unsigned long LED_WIFI_time = 0, DataFromAOGTime = 0;
#define LED_WIFI_pulse 2000   //light on in ms 
#define LED_WIFI_pause 1500    //light off in ms
boolean LED_WIFI_ON = false;

//WIFI+Ethernet
unsigned long WebIOTimeOut = 0, WebIOLastUsePlus3 = 0, WiFi_network_search_timeout = 0, Eth_network_search_timeout = 0;
byte Eth_connect_step, WiFi_connect_step = 10, WiFi_STA_connect_call_nr = 1, WiFi_netw_nr = 0, my_WiFi_Mode = 0; // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint
IPAddress WiFi_ipDestination, Eth_ipDestination; //set in network.ino
bool EthUDPRunning = false, WebIORunning = true, WiFiUDPRunning = false, newDataFromAOG = false, task_WiFiConnectRunning = false, task_EthConnectRunning = false;

//loop time variables in microseconds
const unsigned int Data_LOOP_TIME = 110; //10hz + 10ms to wait for AOG data
const unsigned int WAS_LOOP_TIME = 20;//50Hz
unsigned int DataLoopNextTime = 100, now = 100, WASLoopNextTime = 100;
byte watchdogTimer = 0;

//program flow
int AnalogValue = 0;
bool steerEnable = false, toggleSteerEnable = false, SteerButtonPressed = false, steerEnableOld = false, remoteSwitchPressed = false;
byte guidanceStatus = 0, workSwitch = 0, workSwitchOld = 0, steerSwitch = 1, switchByte = 0;
float gpsSpeed = 0, distanceFromLine = 0; 

//steering variables
float steerAngleActual = 0, steerAngleSetPoint = 0, steerAngleError = 0; //setpoint - actual
long steeringPosition = 0,  actualSteerPosRAW = 0; //from steering sensor steeringPosition_corr = 0,
int  pulseCount = 0, prevEncAState = 0, prevEncBState = 0; // Steering Wheel Encoder
bool encDebounce = false; // Steering Wheel Encoder

//IMU, inclinometer variables
int16_t roll16 = 0, heading16 = 0;
uint16_t x_, y_, z_;
float roll = 0, lastRoll = 0, diff = 0, heading = 0;
float bno08xHeading = 0;
double bno08xRoll = 0;
double bno08xPitch = 0;
int bno08xHeading10x = 0, bno08xRoll10x = 0;

// BNO08x address variables to check where it is
int nrBNO08xAdresses = sizeof(Set.bno08xAddresses) / sizeof(Set.bno08xAddresses[0]);
byte bno08xAddress, REPORT_INTERVAL;

//Kalman variables
float Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float rollMMA = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.001; //0,00025 smaller is more filtering

//pwm variables
int pwmDrive = 0,  pwmDisplay = 0, pwmOut = 0, errorAbs = 0, highLowPerDeg = 0;
float pValue = 0;// iValue = 0, dValue = 0;drive = 0,

//Relais, Tramlines
byte SectGrFromAOG[2] = { 0,0 }, Tram = 0;

//webpage
long argVal = 0;



// Setup procedure -----------------------------------------------------------------------------------------------

void setup() {
	delay(300);//wait for power to stabilize
	delay(300);//wait for IO chips to get ready

	//init USB
	Serial.begin(Set.BaudRate);
	delay(200); //without waiting, no serial print
	Serial.println();

	//get EEPROM data
	restoreEEprom();
	delay(100);

	// for PWM High to Low interpolator
	highLowPerDeg = ((float)(Set.highPWM - Set.lowPWM)) / Set.MotorSlowDriveDegrees;

	//write PGN to output sentence	
	steerToAOG[0] = FromAOGSentenceHeader[0];   //0x80
	steerToAOG[1] = FromAOGSentenceHeader[1];   //0x81
	steerToAOG[2] = FromAOGSentenceHeader[2];   //0x7F
	steerToAOG[3] = steerDataToAOGHeader;
	steerToAOG[4] = steerDataSentenceToAOGLength - 6; //length of data = all - header - length - CRC
	incomSentenceDigit = 0;

	Serial.print("ESP code Version Nr ");
	Serial.print(vers_nr);
	Serial.println(VersionTXT);

	//set GPIOs
	assignGPIOs_start_extHardware();
	delay(200);

	//start Ethernet
	if (Set.DataTransVia == 10) {
		Eth_connect_step = 10;
		xTaskCreate(Eth_handle_connection, "EthConnectHandle", 3072, NULL, 1, &taskHandle_Eth_connect);
		delay(500);
	}
	else { Eth_connect_step = 255; }

	WiFi_connect_step = 10;//step 10 = begin of starting a WiFi connection

	//start WiFi
	xTaskCreate(WiFi_handle_connection, "WiFiConnectHandle", 3072, NULL, 1, &taskHandle_WiFi_connect);
	delay(500);

	//get Data
	if (Set.DataTransVia < 5) {//USB
		xTaskCreate(getDataFromAOGUSB, "DataFromAOGHandleUSB", 5000, NULL, 1, &taskHandle_DataFromAOGUSB);
	}
	else {
		if (Set.DataTransVia == 10) {//Ethernet UDP
			xTaskCreate(getDataFromAOGEth, "DataFromAOGHandleEth", 5000, NULL, 1, &taskHandle_DataFromAOGEth);
		}
	}
	delay(500);

	//handle WiFi LED status
	xTaskCreate(WiFi_LED_blink_and_Connection_check, "WiFiLEDBlink", 3072, NULL, 0, &taskHandle_LEDBlink);
	delay(500);

	vTaskDelay(5000); //waiting for other tasks to start
}

// Main loop -----------------------------------------------------------------------------------------------


void loop() {

	//runs allways (not in timed loop)	

	//new data from AOG? Data comes via extra task and is written into byte array. Parsing called here
	for (;;) {
		if (incommingBytesArrayNr == incommingBytesArrayNrToParse) {
			vTaskDelay(2);//3 wait if no new data to give time to other tasks 
			break;
		}
		else { parseDataFromAOG(); }
	}


	//check, if steering wheel is moved. Debounce set to LOW in timed loop 10Hz
	if (Set.ShaftEncoder == 1) {
		if ((digitalRead(Set.encA_PIN) != prevEncAState) && !encDebounce) { pulseCount++; encDebounce = HIGH; }
		if ((digitalRead(Set.encB_PIN) != prevEncBState) && !encDebounce) { pulseCount++; encDebounce = HIGH; }
	}

	//read steer switch
	int tempvalue = 0;
	switch (Set.SteerSwitchType)
	{
	case 0:
		steerEnable = digitalRead(Set.STEERSW_PIN);
		steerSwitch = steerEnable;
		if (steerEnableOld != steerEnable) {
			if (Set.debugmode) {
				if (steerEnable) { Serial.println("Autosteer ON by Switch"); }
				else { Serial.println("Autosteer OFF by Switch"); }
			}
			steerEnableOld = steerEnable;
		}
		break;
	case 1:
		steerEnable = !digitalRead(Set.STEERSW_PIN);
		steerSwitch = steerEnable;
		if (steerEnableOld != steerEnable) {
			if (Set.debugmode) {
				if (steerEnable) { Serial.println("Autosteer ON by Switch"); }
				else { Serial.println("Autosteer OFF by Switch"); }
			}
			steerEnableOld = steerEnable;
		}
		break;
	case 2:
		if (toggleSteerEnable) //may set to true in timed loop (to debounce)
		{
			steerEnable = !steerEnable;
			steerEnableOld = steerEnable;
			if (Set.debugmode) { Serial.println("Steer-Break: IRQ occured? Button pressed?"); }
			toggleSteerEnable = false;
		}
		steerSwitch = steerEnable;
		break;
	case 3:
		tempvalue = analogRead(Set.STEERSW_PIN);
		if (tempvalue < 800) { steerEnable = false; }
		if (tempvalue > 3200) { steerEnable = true; }
		if (steerEnableOld != steerEnable) {
			if (Set.debugmode) {
				if (steerEnable) { Serial.println("Autosteer ON by Switch"); }
				else { Serial.println("Autosteer OFF by Switch"); }
			}
			steerEnableOld = steerEnable;
		}
		steerSwitch = steerEnable;
		break;
	case 255:
		// No steer switch and no steer button
		// So set the correct value. When guidanceStatus = 1, 
		// it should be on because the button is pressed in the GUI
		// But the guidancestatus should have set it off first (first run)
		if (bitRead(guidanceStatus, 0)) {
			if ((steerEnable) || (steerEnableOld))//not first run
			{
				steerEnable = true;
			}
		}
		else {
			steerEnable = false;
			steerEnableOld = true;//reset first run, so next time will turn on
		}

		steerSwitch = steerEnable;
		break;
	}

	//valid conditions to turn on autosteer?
	if (steerEnable) {//steerEnable was set by switch so now check if really can turn on
		//auto Steer is off if AOG autosteer not active, Speed is too slow/high, encoder pulses
		if ((!bitRead(guidanceStatus, 0)) || (pulseCount >= Set.pulseCountMax) ||
			(gpsSpeed < Set.autoSteerMinSpeed) || (gpsSpeed > Set.autoSteerMaxSpeed))
		{
			steerEnable = false;
			if (steerEnable != steerEnableOld) {
				if (Set.debugmode) { Serial.println(" Steer-Break:  AOG not active or Speed too low or Steering Wheel Encoder.."); }
				steerEnableOld = steerEnable;
			}
			pulseCount = 0;
			digitalWrite(Set.AutosteerLED_PIN, LOW); //turn LED off
		}
	}

	//turn autosteer OFF or ON//steer (motor...)
	if ((steerEnable) && (watchdogTimer < 200))
	{
		digitalWrite(Set.AutosteerLED_PIN, HIGH);  //turn LED on (LED connected to MotorDriver = ON = Motor moves)
		steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error 

		calcSteeringPID();   //do the pid     
		motorDrive();       //out to motors the pwm value 
	}
	else
	{
		digitalWrite(Set.AutosteerLED_PIN, LOW);  //turn LED off 

		steerEnable = false;

		if ((steerEnable != steerEnableOld) && (watchdogTimer >= 200)) {
			//we've lost the comm to AgOpenGPS
			if (Set.debugmode) { Serial.println("Steer-Break: watch dog timer runs out, no Data from AOG"); }
			steerEnableOld = steerEnable;
		}

		pwmDrive = 0; //turn off steering motor
		motorDrive(); //out to motors the pwm value   
		pulseCount = 0; //Reset counters if Autosteer is offline  
	}

	vTaskDelay(mainLoopDelay);

	//timed loop for WAS
	// Loop triggers every 20 msec
	now = millis();

	if (now > WASLoopNextTime)
	{
		WASLoopNextTime = now + WAS_LOOP_TIME;

		if (SectRelaysUsed) { SetRelays(); } //turn on off sections, do in timed loop, if new data comes in

		encDebounce = LOW; //reset steerEncoder debounce

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250) watchdogTimer = 250;

		//steering position and steer angle
		switch (Set.WASType) {
		case 1:
			steeringPosition = ads.getLastConversionResults();
			//steeringPosition = steeringPosition >> 1; //divide by 2
			// Start another conversion.
			ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
			break;
		case 2:
			steeringPosition = ads.getLastConversionResults();
			//steeringPosition = steeringPosition >> 1; //divide by 2
			// Start another conversion.
			ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
			break;
		default: // directly to arduino
			steeringPosition = analogRead(Set.WAS_PIN);    vTaskDelay(2);
			steeringPosition += analogRead(Set.WAS_PIN);
			break;
		}
		//Serial.print("SteeringPosRAW: "); Serial.println(steeringPosition);
		actualSteerPosRAW = steeringPosition; // stored for >zero< Funktion

		//center the steering position sensor  
		steeringPosition = steeringPosition - Set.WebIOSteerPosZero - Set.AOGSteerPositionZero;

		//invert position, left must be minus
		if (Set.InvertWAS == 1) steeringPosition *= -1;

		//Ackermann fix: correct non linear values = right side factor
		if (steeringPosition > 0) {
			steeringPosition = long((steeringPosition * Set.AckermanFix) / 100);
		}

		//convert position to steer angle
		steerAngleActual = (steerAngleActual * 0.7) + (((float(steeringPosition)) / Set.steerSensorCounts) * 0.3);
		
	/*	Serial.print("steerPos long: "); Serial.print(steeringPosition); 
		Serial.print(" steerPos float: "); Serial.print(float(steeringPosition));
		Serial.print(" steerAg Act: "); Serial.print(steerAngleActual);
		Serial.print(" CountPerDeg: "); Serial.println(Set.steerSensorCounts);
		*/
		vTaskDelay(1);
	}//WAS timed loop


// data timed loop
// Loop triggers every 100 msec or when data from AOG came in and sends back gyro heading16, and roll, steer angle etc
	now = millis();

	if ((now > DataLoopNextTime) || (newDataFromAOG))
	{
		newDataFromAOG = false;

		DataLoopNextTime = now + Data_LOOP_TIME;

		//workswitch
		switch (Set.WorkSW_mode)
		{
		case 1:
			if (Set.Invert_WorkSW == 0) workSwitch = digitalRead(Set.WORKSW_PIN);    // read digital work switch
			if (Set.Invert_WorkSW == 1) workSwitch = !digitalRead(Set.WORKSW_PIN);    // read digital work switch
			break;
		case 2:
			AnalogValue = analogRead(Set.WORKSW_PIN);
			delay(2);
			AnalogValue += analogRead(Set.WORKSW_PIN);
			AnalogValue = AnalogValue >> 1;
			if (Set.Invert_WorkSW == 0) {
				if (AnalogValue < Set.WorkSW_Threshold)   workSwitch = 1;
				else workSwitch = 0;
			}
			if (Set.Invert_WorkSW == 1) {
				if (AnalogValue > Set.WorkSW_Threshold)   workSwitch = 1;
				else workSwitch = 0;
			}
			break;
		}
		if (Set.debugmode) {
			if (workSwitch != workSwitchOld) {
				if (workSwitch > 0) {
					Serial.println("workswitch: ON");
					workSwitchOld = workSwitch;
				}
				else {
					Serial.println("workSwitch: OFF");
					workSwitchOld = workSwitch;
				}
			}
		}

		//steer Button momentary
		if (Set.SteerSwitchType == 2)
		{
			if (digitalRead(Set.STEERSW_PIN) == LOW) {//pressed
				if (!SteerButtonPressed) { toggleSteerEnable = true; SteerButtonPressed = true; }
			}
			else {
				SteerButtonPressed = false;
			}
		}

		switchByte = 0;
		switchByte |= (!steerSwitch << 1); //put steerswitch status in bit 1 position
		switchByte |= workSwitch;



		//Build Autosteer Packet
		int temInt;

		//actual steer angle
		temInt = (100 * steerAngleActual);
		steerToAOG[5] = (byte)(temInt);
		steerToAOG[6] = (byte)(temInt >> 8);

		if (Set.IMUType == 1) {   // Initialize the BNO055 if not done
			BNO.readIMU();
		}

		if (Set.IMUType > 0) {//imu heading16 --- * 10 in degrees,  V17= *16
			switch (Set.IMUType) {
			case 1:// BNO055 heading16
				heading16 = (int)BNO.euler.head;  //heading16 in degrees * 16 from BNO
				heading = float(heading16) / 16;
				temInt = int(heading * 10);
				steerToAOG[8] = temInt >> 8;
				steerToAOG[7] = byte(temInt);
				break;

			case 2://CMPS14			
				Wire.beginTransmission(Set.CMPS14_ADDRESS);
				Wire.write(0x02);
				Wire.endTransmission();

				Wire.requestFrom(Set.CMPS14_ADDRESS, 2);
				while (Wire.available() < 2);

				//the heading16 x10
				steerToAOG[8] = Wire.read();
				steerToAOG[7] = Wire.read();
				//convert to float for WebIO
				heading16 = steerToAOG[8];
				heading16 <<= 8;
				heading16 += steerToAOG[7];
				heading = float(heading16) * 0.1;

				Wire.beginTransmission(Set.CMPS14_ADDRESS);
				Wire.write(0x1C);
				Wire.endTransmission();

				Wire.requestFrom(Set.CMPS14_ADDRESS, 2);
				while (Wire.available() < 2);

				//the roll x10
				steerToAOG[10] = Wire.read();
				steerToAOG[9] = Wire.read();
				//convert to float for WebIO
				roll16 = steerToAOG[10];
				roll16 <<= 8;
				roll16 += steerToAOG[9];
				roll = float(roll16) * 0.1;
				//if (bitRead(roll16, 15)) { roll -= 6554; }
				if (Set.InvertRoll) { roll *= -1; }//affects only number shown in Webinterface
				break;

			case 3:
				if (bno08x.dataAvailable() == false) { vTaskDelay(2); }//wait 2ms if no new data from BNO			
				if (bno08x.dataAvailable() == true)
				{
					bno08xHeading = (bno08x.getYaw()) * 180.0 / PI; // Convert yaw / heading16 to degrees
					bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

					if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180�;180�] to [0;360�]
					{
						bno08xHeading = bno08xHeading + 360;
					}

					bno08xRoll = (bno08x.getRoll()) * 180.0 / PI; //Convert roll to degrees
					bno08xPitch = (bno08x.getPitch()) * 180.0 / PI; // Convert pitch to degrees

					bno08xHeading10x = (int)(bno08xHeading * 10);
					bno08xRoll10x = (int)(bno08xRoll * 10);
					heading = bno08xHeading;
					roll = float(bno08xRoll);

					//the heading16 x10
					steerToAOG[8] = (byte)bno08xHeading10x;
					steerToAOG[7] = bno08xHeading10x >> 8;

					//the roll x10
					steerToAOG[10] = (byte)bno08xRoll10x;
					steerToAOG[9] = bno08xRoll10x >> 8;
				}
				break;

			}//switch heading16
		}

		//switch byte
		steerToAOG[11] = switchByte;

		//pwm value
		if (pwmDisplay < 0) pwmDisplay *= -1;
		steerToAOG[12] = pwmDisplay;

		//add the checksum
		int CRCtoAOG = 0;
		for (byte i = 2; i < sizeof(steerToAOG) - 1; i++)
		{
			CRCtoAOG = (CRCtoAOG + steerToAOG[i]);
		}
		steerToAOG[sizeof(steerToAOG) - 1] = CRCtoAOG;

		//Build Autosteer Packet completed
		//send packet  0 = USB / 7 = WiFi UDP / 10 = Ethernet UDP
		if (Set.DataTransVia < 5) {//USB
			Serial.write(steerToAOG, sizeof(steerToAOG));
			//Serial.println("send Data via USB");
		}
		else {
			if (Set.DataTransVia < 10) {//WiFi UDP
				if (WiFiUDPRunning) {
					WiFiUDPToAOG.writeTo(steerToAOG, sizeof(steerToAOG), WiFi_ipDestination, Set.PortDestination);
				}
			}
			else {//Ethernet
				if (EthUDPRunning) {
					EthUDPToAOG.beginPacket(Eth_ipDestination, Set.PortDestination);
					EthUDPToAOG.write(steerToAOG, sizeof(steerToAOG));
					EthUDPToAOG.endPacket();
				}
			}
		}//transmit to AOG
		if (Set.debugmode) {
			Serial.println("Data to AOG: steerangle, steerangleSetPoint, IMU heading, roll, switchByte, PWM, checksum");
			Serial.print(steerAngleActual); //The actual steering angle in degrees
			Serial.print(",");
			Serial.print(steerAngleSetPoint);
			Serial.print(",");
			Serial.print(heading);
			Serial.print(",");
			Serial.print(roll);
			Serial.print(",");
			Serial.print(switchByte);
			Serial.print(",");
			Serial.print(pwmDisplay);
			Serial.print(",");
			Serial.println(CRCtoAOG);
			Serial.print("ESP time (millis): ");
			Serial.println(millis());
		}

	}  //end of  data timed loop
}
