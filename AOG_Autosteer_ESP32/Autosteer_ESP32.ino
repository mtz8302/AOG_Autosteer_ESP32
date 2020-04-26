// ready for AOG V4 + V4.2  by MTZ8302  26.04.2020
//##########################################################################################################
//### Setup Zone ###########################################################################################
//
//Set default values here. They can be changed in webinterface later. (x.x.x.77 or 192.168.1.1)
//
//If you don't get access to webinterface, use serial monitor with USB. IP address is displayed at start.
//
//##########################################################################################################



#define useLED_BUILTIN  1	          // some ESP board have a build in LED, some not

#define HardwarePlatform  0         //0 = runs on ESP32 1 = run on NANO 33 IoT (not working yet!!)


struct Storage {
	char ssid[24] = "Fendt_209V";     // WiFi network Client name
	char password[24] = "";           // WiFi network password
	uint16_t timeoutRouter = 100;     // Time (seconds) to wait for WIFI access, after that own Access Point starts

	uint8_t aogVersion = 8;			  // Version number for version check 4.2.01 = 4+2+1 = 7	

	uint8_t DataTransVia = 1;         //transfer data via 0: USB, 1: WiFi, 4: USB 10 byte format for AOG V4

	uint8_t output_type = 2;          //set to 1  if you want to use Stering Motor + Cytron MD30C Driver
								      //set to 2  if you want to use Stering Motor + IBT 2  Driver
								      //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
								      //set to 4  if you want to use  IBT 2  Driver + Danfoss Valve PVE A/H/M
	uint16_t PWMOutFrequ = 20000;     //PWM frequency for motordriver: 1000Hz:for low heat at PWM device 20000Hz: not hearable

	uint8_t	MotorDriveDirection = 0;  // 0 = normal, 1 = inverted

	uint8_t MotorSlowDriveDegrees = 5;	  //How many degrees before decreasing Max PWM

	uint8_t input_type = 2;           //0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 4 (attention 3,3V only)
								                    //1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
								                    //2 = Differential Mode - Connect Sensor GND to A1, Signal to A0

	uint8_t BNOInstalled = 0;         // set to 1 to enable BNO055 IMU

	uint8_t InclinometerInstalled = 0;// set to 1 if MMA8452 is installed at address 1C (Adr PIN to GND) set to 2 at address 1D (Adr PIN open)

	uint8_t UseMMA_X_Axis = 1;		  // 1: use X axis (default) 0: use Y axis

	uint8_t InvertRoll = 1;           // 0: no, set to 1 to change roll direction
	byte roll_MAX_STEP = 20;

	uint8_t Invert_WAS = 0;              // set to 1 to Change Direction of Wheel Angle Sensor - to + 

	uint8_t ShaftEncoder = 0;         // Steering Wheel ENCODER Installed
	uint8_t pulseCountMax = 3;        // Switch off Autosteer after x Pulses from Steering wheel encoder 

	uint16_t SteerPosZero = 10300;	  // first value for steer zero position ADS: 11000 for EPS32 AD PIN: 2048

	uint8_t AckermanFix = 78;		  //if values for left and right are the same: 100 

	uint8_t SteerSwitch = 1;          //0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
								      //3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V

	uint8_t SteerRemoteSwitch = 0;    //not supported by V4 0 = not installed, 1 = switch to GND, 2 = use steer switch may not work, just a test

	uint8_t WorkSW_mode = 2;          // 0 = disabled   // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)

	uint8_t Invert_WorkSW = 0;        // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low

	uint8_t autoSteerMinSpeed4 = 2;    //AOG sends speed * 4 as byte so lowest is 1 = 0.25 km/h 2=0.5 3=0.75 4=1 km/h

	uint8_t autosteerMaxSpeed4 = 255;	// set to 255 to be able to drive backwards

	uint16_t WorkSW_Threshold = 1500; // Value for analog hitch level to switch workswitch  (0-4096)

	
	// IO pins --------------------------------

	// set to 255 for unused !!!!!
	uint8_t SDA = 32;//21			        //I2C Pins
	uint8_t SCL = 15;//22

	uint8_t AutosteerLED_PIN = 19;    //light on active autosteer
	uint8_t LEDWiFi_PIN = 18;         //light on WiFi connected, flashes on searching Network
	uint8_t LEDWiFi_ON_Level = HIGH;	//HIGH = LED on high, LOW = LED on low

	uint8_t Relay1_PIN = 255;
	uint8_t Relay2_PIN = 255;
	uint8_t Relays_ON = HIGH;		      //HIGH = Relay on high, LOW = Relay on low

	uint8_t W_A_S = 255;              //PIN for Wheel Angle Sensor (none, if ADS used)
	uint8_t WORKSW_PIN = 39;          //PIN for workswitch (can be analog or on/off switch see WorkSW_mode)
	uint8_t STEERSW_PIN = 5;          //Pin for steer button or switch (see SteerSwitch)
	uint8_t REMOTE_PIN = 255;		  //PIN for AOG autosteer ON/OFF
	uint8_t encA_PIN = 255;           //Pin for steer encoder, to turn off autosteer if steering wheel is used
	uint8_t encB_PIN = 255;           //Pin for steer encoder, to turn off autosteer if steering wheel is used

	uint8_t PWM_PIN = 17;             //PWM Output to motor controller (IBT2 or cytron)
	uint8_t DIR_PIN = 16;             //direction output to motor controller (IBT2 or cytron)

	//##########################################################################################################
	//### End of Setup Zone ####################################################################################
	//##########################################################################################################
	//set up the pgn for communication with AOG
	uint8_t DataToAOGHeader[2] = { 0x7F,0xFD }, DataFromAOGHeader[2] = { 0x7F, 0xFE }, SettingsFromAOGHeader[2] = { 0x7F,0xFC }, ArdConfigFromAOGHeader[2] = { 0x7F,0xFB };
	char ssid_ap[24] = "AG_Autosteer_ESP_Net";  //used if accesspoint is created, no password!!
	uint8_t myip[4] = { 192, 168, 1, 77 };      // Autosteer module
	uint8_t gwip[4] = { 192, 168, 1, 1 };       // Gateway & Accesspoint IP
	uint8_t mask[4] = { 255, 255, 255, 0 };
	uint8_t myDNS[4] = { 8, 8, 8, 8 };          //optional
	uint8_t myIPEnding = 77;                    //getting IP from DHCP and change to x.x.x.77 

	uint16_t portMy = 5577;                     //this is port of this module: Autosteer = 5577
	uint16_t portAOG = 8888;                    // port to listen for AOG

	//IP address to send UDP data to:
	uint8_t ipDestination[4] = { 192, 168, 1, 255 };
	uint16_t portDestination = 9999;            // Port of AOG that listens

	//filter variables set by AOG via PGN settings sentence, stored automatically in EEPROM for faster start
	float Ko = 0.05f;   //overall gain  
	float Kp = 5.0f;    //proportional gain  
	float Ki = 0.001f;  //integral gain
	float Kd = 1.0f;    //derivative gain 
	float steeringPositionZero = 12540;  uint8_t minPWMValue = 30;
	uint16_t maxIntegralValue = 20; //max PWM value for integral PID component
	float steerSensorCounts = 100;  uint16_t roll_corr = 200;
	float deadZone = 0.0f;  //band of no action
	byte minPWM = 30;
	byte maxPWM = 100;//max PWM value
	
	
	boolean debugmode = false;


};  Storage steerSet;

boolean EEPROM_clear = false;  //set to true when changing settings to write them as default values: true -> flash -> boot -> false -> flash again


//libraries -------------------------------
#if HardwarePlatform == 1
#include <WiFiNINA.h>
#include <FlashAsEEPROM.h>
#endif
#if HardwarePlatform == 0
#include "EEPROM.h"
#endif
//#include <ESPmDNS.h>//OTA
//#include <Update.h>
#include "Wire.h"
#include "BNO055_AOG.h"
#include "zADS1015.h"
#include "MMA8452_AOG.h"  //MMA inclinometer
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFi.h>

#define A 0X28             //I2C address selection pin LOW BNO055
#define B 0x29             //                          HIGH

// Instances ------------------------------
Adafruit_ADS1115 ads;     // Use this for the 16-bit version ADS1115
//MMA8452 accelerometer;
MMA8452 MMA1D(0x1D);
MMA8452 MMA1C(0x1C);
BNO055 IMU(A);
WiFiServer server(80);
WiFiClient client_page;
WiFiUDP UDPFromAOG;
WiFiUDP UDPToAOG;

// Variables ------------------------------
byte my_WiFi_Mode = 0;  // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint

// WiFi status LED blink times: searching WIFI: blinking 4x faster; connected: blinking as times set; data available: light on; no data for 2 seconds: blinking
unsigned long LED_WIFI_time = 0;
unsigned long LED_WIFI_pulse = 1000;   //light on in ms 
unsigned long LED_WIFI_pause = 500;   //light off in ms
boolean LED_WIFI_ON = false;
unsigned long DataFromAOGTime = 0;

//loop time variables in microseconds
const unsigned int LOOP_TIME = 40; //25hz 
unsigned int lastTime = LOOP_TIME;
unsigned int currentTime = LOOP_TIME, remoteSwitchDelay = 0;
byte watchdogTimer = 0;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float XeRoll = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.001; //0,00025 smaller is more filtering

//program flow
//bool webupdate = false;
bool isDataFound = false, isSettingFound = false, isArdConfigFound = false, isMachineFound = false, steerSettingChanged = false;
bool MMAinitialized = false;
int AnalogValue = 0;
bool steerEnable = false, toggleSteerEnable = false, SteerButtonPressed = false, steerEnableOld = false, remoteSwitchPressed = false;
byte relay = 0, relayHi = 0,uTurn = 0, workSwitch = 0, workSwitchOld = 0, steerSwitch = 1, switchByte = 0, remoteSwitch = 0;
float gpsSpeed = 0, distanceFromLine = 0, olddist = 0; // not used, corr = 0 
int16_t idistanceFromLine = 0;
//steering variables
float steerAngleActual = 0;
//int steerPrevSign = 0, steerCurrentSign = 0; // the steering wheels angle currently and previous one
int16_t isteerAngleSetPoint = 0; //the desired angle from AgOpen
float steerAngleSetPoint = 0;
long steeringPosition = 0,  actualSteerPos = 0; //from steering sensor steeringPosition_corr = 0,
float steerAngleError = 0; //setpoint - actual
//float distanceError = 0; //
int  pulseCount = 0, prevEncAState = 0, prevEncBState = 0; // Steering Wheel Encoder
bool encDebounce = false; // Steering Wheel Encoder
float corr = 0;

//IMU, inclinometer variables
bool imu_initialized = 0;
int16_t roll = 0, heading = 0;
uint16_t x_, y_, z_;
float lastRoll = 0, diff = 0;

//pwm variables
int pwmDrive = 0, drive = 0, pwmDisplay = 0, pwmOut = 0, errorAbs = 0, highLowPerDeg = 0;
float pValue = 0;// iValue = 0, dValue = 0;

//integral values - **** change as required *****
int maxIntErr = 200; //anti windup max
int maxIntegralValue = 20; //max PWM value for integral PID component 

//Array to send data back to AgOpenGPS
byte toSend[] = { 0,0,0,0,0,0,0,0,0,0 };
//data that will be received from server
uint8_t DataFromAOG[10];





// Setup procedure -----------------------------------------------------------------------------------------------

void setup() {
	delay(300);//wait for power to stabilize
	delay(300);//wait for IO chips to get ready
	
	//init USB
	Serial.begin(38400);
	delay(100); //without waiting, no serial print
	Serial.println();	
	
	//get EEPROM data
	restoreEEprom();
	delay(300);

	//write PGN to output sentence
	toSend[0] = steerSet.DataToAOGHeader[0];
	toSend[1] = steerSet.DataToAOGHeader[1];

	//set GPIOs
	assignGPIOs_start_extHardware();
	delay(200);

	// Start WiFi Client
	WiFi_Start_STA();
	if (my_WiFi_Mode == 0) WiFi_Start_AP(); // if failed start AP
	delay(500);

	Serial.print("gain of ADS1115: ");
	Serial.println(ads.getGain());

	delay(500);
	UDP_Start();  // start the UDP Client
	delay(200);
	//start server for settings homepage
	server.begin();
	delay(200);
	Serial.print("Debugmode "); if (steerSet.debugmode) { Serial.println("ON"); }
	else { Serial.println("OFF"); }
	if (steerSet.DataTransVia == 0) { Serial.println("data transfer via USB 8 Byte sentence AOG -2019"); }
	else {
		if (steerSet.DataTransVia == 4) { Serial.println("data transfer via USB 10 Byte sentence AOG V4"); }
		else { Serial.println("data transfer via UDP"); }
	}

}

// Main loop -----------------------------------------------------------------------------------------------


void loop() {
	
	//runs allways (not in timed loop)	

#if HardwarePlatform == 0 //ESP32
	doWebInterface(); 
#endif

	WiFi_LED_blink();

#if HardwarePlatform == 1 //nano33iot
	delay(5);//do WiFi
#endif

	getDataFromAOG();

	//check, if steering wheel is moved. Debounce set to LOW in timed loop 10Hz
	if (steerSet.ShaftEncoder == 1) {
		if ((digitalRead(steerSet.encA_PIN) != prevEncAState) && !encDebounce) { pulseCount++; encDebounce = HIGH; }
		if ((digitalRead(steerSet.encB_PIN) != prevEncBState) && !encDebounce) { pulseCount++; encDebounce = HIGH; }
	}

	//read steer switch
	switch (steerSet.SteerSwitch)
	{
	case 0:
		steerEnable = digitalRead(steerSet.STEERSW_PIN);
		steerSwitch = steerEnable;
		if (steerEnableOld != steerEnable) {
			if (steerSet.debugmode) {
				if (steerEnable) { Serial.println("Autosteer ON by Switch"); }
				else { Serial.println("Autosteer OFF by Switch"); }
			}
			steerEnableOld = steerEnable;
		}
		break;
	case 1:
		steerEnable = !digitalRead(steerSet.STEERSW_PIN);
		steerSwitch = steerEnable;
		if (steerEnableOld != steerEnable) {
			if (steerSet.debugmode) {
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
			if (steerSet.debugmode) { Serial.println("Steer-Break: IRQ occured? Button pressed?"); }
			toggleSteerEnable = false;
		}
		steerSwitch = steerEnable;
		break;
	case 3:
		byte tempvalue = analogRead(steerSet.STEERSW_PIN);
		if (tempvalue < 800) { steerEnable = false; }
		if (tempvalue > 3200) { steerEnable = true; }
		if (steerEnableOld != steerEnable) {
			if (steerSet.debugmode) {
				if (steerEnable) { Serial.println("Autosteer ON by Switch"); }
				else { Serial.println("Autosteer OFF by Switch"); }
			}
			steerEnableOld = steerEnable;
		}
		steerSwitch = steerEnable;
		break;
	}

	//valid conditions to turn on autosteer?
	if (steerEnable) {//steerEnable was set by switch so now check if really can turn on
		//auto Steer is off if 32020,Speed is too slow, encoder pulses 
		if ((int(distanceFromLine) == 32020) || (int(distanceFromLine) == 32000) ||
			(gpsSpeed < (float(steerSet.autoSteerMinSpeed4) * 0.25)) || (gpsSpeed > (float(steerSet.autosteerMaxSpeed4) * 0.25))
			|| (pulseCount >= steerSet.pulseCountMax))
		{
			steerEnable = false;
			if (steerEnable != steerEnableOld) {
				Serial.println(" Steer-Break:  AOG not active or Speed too low or Steering Wheel Encoder..");
				steerEnableOld = steerEnable;
			}
	
			//if (olddist == 32020)  steerEnable = true;              // Take over AOG State on startup

			pulseCount = 0;
			//watchdogTimer = 20;//turn off steering motor
			digitalWrite(steerSet.AutosteerLED_PIN, LOW); //turn LED off
		}
	}

	//turn autosteer OFF or ON//steer (motor...)
	if ((steerEnable) && (watchdogTimer < 18))
	{
		digitalWrite(steerSet.AutosteerLED_PIN, HIGH);  //turn LED on (LED connected to MotorDriver = ON = Motor moves)
		steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error 
		//if (abs(steerAngleError) < steerSet.deadZone) steerAngleError = 0;

		calcSteeringPID();   //do the pid     
		motorDrive();       //out to motors the pwm value 
	}
	else
	{
		digitalWrite(steerSet.AutosteerLED_PIN, LOW);  //turn LED off 
		
		steerEnable = false;

		if ((steerEnable != steerEnableOld) && (watchdogTimer >= 18)) {
			//we've lost the comm to AgOpenGPS
			Serial.println("Steer-Break: watch dog timer runs out, no Data from AOG");    
			steerEnableOld = steerEnable;
		}

		pwmDrive = 0; //turn off steering motor
		motorDrive(); //out to motors the pwm value   
		pulseCount = 0; //Reset counters if Autosteer is offline  
	}

	SetRelays(); //turn on off sections

//timed loop
	//* Loop triggers every 40 msec and sends back gyro heading, and roll, steer angle etc
	currentTime = millis();

	if (currentTime - lastTime >= LOOP_TIME)
	{
		lastTime = currentTime;

		encDebounce = LOW; //reset steerEncoder debounce

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250) watchdogTimer = 100;

		if (steerSet.BNOInstalled == 1) {   // Initialize the BNO055 if not done
			IMU.readIMU();
		}

		if (steerSet.InclinometerInstalled == 0) { XeRoll = 0; }
		else {
			if (steerSet.InclinometerInstalled == 1)  // MMA8452 Inclinometer (1C) 
			{
				if (MMAinitialized)
				{
					MMA1C.getRawData(&x_, &y_, &z_);

					if (steerSet.UseMMA_X_Axis==1)
						roll = x_; //Conversion uint to int
					else roll = y_;

					if (roll > 4200)  roll = 4200;
					if (roll < -4200) roll = -4200;
					//rollK = map(roll, -4200, 4200, -960, 960); //16 counts per degree (good for 0 - +/-30 degrees) 
					roll = roll >> 3;  //divide by 8  +-525

					// limit the differential  
					diff = roll - lastRoll;
					if (diff > steerSet.roll_MAX_STEP) roll = lastRoll + steerSet.roll_MAX_STEP;
					else if (diff < -steerSet.roll_MAX_STEP) roll = lastRoll - steerSet.roll_MAX_STEP;
					lastRoll = roll;

					//divide by 2 -268 to +268    -17 to +17 degrees
					roll = roll >> 1;

					//if not positive when rolling to the right
					if (steerSet.InvertRoll)
						roll *= -1.0;

					//divide by 16
					rollK = roll;				
				}
			}

			// MMA8452 Inclinometer (1D)        
			else if (steerSet.InclinometerInstalled == 2)
			{
				if (MMAinitialized)
				{
					MMA1D.getRawData(&x_, &y_, &z_);

					if (steerSet.UseMMA_X_Axis==1)
						roll = x_; //Conversion uint to int
					else roll = y_;

					if (roll > 4200)  roll = 4200;
					if (roll < -4200) roll = -4200;
					//rollK = map(roll, -4200, 4200, -960, 960); //16 counts per degree (good for 0 - +/-30 degrees) 
					roll = roll >> 3;  //divide by 8  +-525

					// limit the differential  
					diff = roll - lastRoll;
					if (diff > steerSet.roll_MAX_STEP) roll = lastRoll + steerSet.roll_MAX_STEP;
					else if (diff < -steerSet.roll_MAX_STEP) roll = lastRoll - steerSet.roll_MAX_STEP;
					lastRoll = roll;

					//divide by 2 -268 to +268    -17 to +17 degrees
					roll = roll >> 1;

					//if not positive when rolling to the right
					if (steerSet.InvertRoll)
						roll *= -1.0;

					//divide by 16
					rollK = roll;
				}
			}


			//Kalman filter input: rollK
			Pc = P + varProcess;
			G = Pc / (Pc + varRoll);
			P = (1 - G) * Pc;
			Xp = XeRoll;
			Zp = Xp;
			XeRoll = G * (rollK - Zp) + Xp;  //output: XeRoll

			if (steerSet.debugmode) {
				Serial.print("roll from MMA: "); Serial.print(rollK);
				Serial.print(" filtered roll: "); Serial.println(XeRoll);
			}
		}

		//workswitch
		switch (steerSet.WorkSW_mode)
		{
			case 1:
				if (steerSet.Invert_WorkSW == 0) workSwitch = digitalRead(steerSet.WORKSW_PIN);    // read digital work switch
				if (steerSet.Invert_WorkSW == 1) workSwitch = !digitalRead(steerSet.WORKSW_PIN);    // read digital work switch
				break;
			case 2:
				AnalogValue = analogRead(steerSet.WORKSW_PIN);
				delay(2);
				AnalogValue += analogRead(steerSet.WORKSW_PIN);
				AnalogValue = AnalogValue >> 1;
				if (steerSet.Invert_WorkSW == 0) {
					if (AnalogValue < steerSet.WorkSW_Threshold)   workSwitch = 1;
					else workSwitch = 0;
				}
				if (steerSet.Invert_WorkSW == 1) {
					if (AnalogValue > steerSet.WorkSW_Threshold)   workSwitch = 1;
					else workSwitch = 0;
				}
				break;
		}
		if (workSwitch != workSwitchOld) {
			if (workSwitch > 0) {
				if (steerSet.debugmode) { Serial.println("workswitch: ON"); }
				workSwitchOld = workSwitch;
			}
			else {
				if (steerSet.debugmode) { Serial.println("workSwitch: OFF"); }
				workSwitchOld = workSwitch;
			}
		}

		//steer Button momentary
		if (steerSet.SteerSwitch == 2)
		{
			if (digitalRead(steerSet.STEERSW_PIN) == LOW) {//pressed
				if (!SteerButtonPressed) { toggleSteerEnable = true; SteerButtonPressed = true; }
			}
			else {
				SteerButtonPressed = false;
			}
		}

		//read auto steer enable switch open = OFF closed = ON  or send steerSwitch if activated in settings
		if (steerSet.SteerRemoteSwitch == 1) {
			remoteSwitch = digitalRead(steerSet.REMOTE_PIN); 
		}
		else {
			if ((steerSet.SteerRemoteSwitch == 2) && (steerSet.SteerSwitch < 2)) {				
				if (steerSet.SteerSwitch == 1) { remoteSwitch = digitalRead(steerSet.STEERSW_PIN); }
				else { remoteSwitch = !digitalRead(steerSet.STEERSW_PIN); }
			}
		}

		switchByte = 0;
		switchByte |= (remoteSwitch << 2); //put remote in bit 2
		switchByte |= (!steerSwitch << 1);   //put steerswitch status in bit 1 position
		switchByte |= workSwitch;

	//steering position and steer angle
		switch (steerSet.input_type) {
		case 1:  // ADS 1115 single
			steeringPosition = ads.readADC_SingleEnded(0);    delay(1);        //ADS1115 Standard Mode
			steeringPosition += ads.readADC_SingleEnded(0);    delay(1);
			steeringPosition += ads.readADC_SingleEnded(0);    delay(1);
			steeringPosition += ads.readADC_SingleEnded(0);
			break;
		case 2:  // ADS 1115 differential
			steeringPosition = ads.readADC_Differential_0_1();    delay(1);    //ADS1115 Differential Mode 
			steeringPosition += ads.readADC_Differential_0_1();   delay(1);    //Connect Sensor GND to A1
			steeringPosition += ads.readADC_Differential_0_1();   delay(1);    //Connect Sensor Signal to A0
			steeringPosition += ads.readADC_Differential_0_1();
			//if (steerSet.debugmode) { Serial.print("get WAS: "); Serial.println(steeringPosition); }
			break;
		default: // directly to arduino
			steeringPosition = analogRead(steerSet.W_A_S);    delay(2);
			steeringPosition += analogRead(steerSet.W_A_S);
			break;
		}
		steeringPosition = steeringPosition >> 2; //divide by 4
		actualSteerPos = steeringPosition; // stored for >zero< Funktion
		steeringPosition = (steeringPosition - steerSet.steeringPositionZero);   //center the steering position sensor  

		//invert position, left must be minus
		if (steerSet.Invert_WAS == 1) steeringPosition *= -1;

		//correct non linear values = right side factor
		if (steeringPosition > 0) {
			steeringPosition = long((steeringPosition * steerSet.AckermanFix) / 100);}

		//convert position to steer angle
		steerAngleActual = (float)(steeringPosition) / steerSet.steerSensorCounts;

		// add the roll
		//if (steerSet.InclinometerInstalled >= 1) steerAngleActual = steerAngleActual - (XeRoll * (steerSet.Kd / 800));     

//Build Autosteer Packet: Send to agopenGPS **** you must send 10 Byte or 5 Int
		int temp;

		//actual steer angle
		temp = (100 * steerAngleActual);
		toSend[2] = (byte)(temp >> 8);
		toSend[3] = (byte)(temp);

		//imu heading --- * 16 in degrees
		//read BNO heading
		if (steerSet.BNOInstalled == 1) {
			heading = (int)IMU.euler.head;     //heading in degrees * 16 from BNO
			temp = heading;
		}
		else { temp = 0; }                   //No IMU installed
		toSend[4] = (byte)(temp >> 8);
		toSend[5] = (byte)(temp);

		//Vehicle roll --- * 16 in degrees
		if (steerSet.InclinometerInstalled > 0) {
			temp = (int)XeRoll;
		}  
		else { temp = 0; }                //no Dogs installed
		temp = XeRoll;
		toSend[6] = (byte)(temp >> 8);
		toSend[7] = (byte)(temp);

		//switch byte
		toSend[8] = switchByte;

		//pwm value
		if (pwmDisplay < 0) pwmDisplay *= -1;
		toSend[9] = pwmDisplay;

		//Build Autosteer Packet completed
		//send packet
		if (steerSet.DataTransVia == 4) //send 2 header bytes first
			{ Serial.print(steerSet.DataToAOGHeader[0]); Serial.print(","); 
			Serial.print(steerSet.DataToAOGHeader[1]); Serial.print(",");}
		if ((steerSet.DataTransVia == 0) || (steerSet.DataTransVia == 4)) {
			//USB  //NO header with serial data!!			
			//steerangle			
			Serial.print(int(steerAngleActual*100)); Serial.print(",");
			//steerangle set point NOT used in AOG!!
			Serial.print(int(steerAngleSetPoint * 100)); Serial.print(",");
			//heading from BNO
			if (steerSet.BNOInstalled == 1) { Serial.print(heading); }
			else {Serial.print(0);}
			Serial.print(",");
			//roll from MMA
			Serial.print(int(XeRoll*100)); Serial.print(",");
			//switch byte
			Serial.print(toSend[8]);
			if (steerSet.aogVersion > 0) { Serial.print(","); Serial.print(toSend[9]); }
			Serial.println();
		}

		else { Send_UDP(); delay(2); }//transmit to AOG

		if (steerSet.debugmode) {
			Serial.println("Data to AOG: steerangle, steerangleSetPoint, IMU heading, roll, switchByte, PWM");
			Serial.print(steerAngleActual); //The actual steering angle in degrees
			Serial.print(",");
			Serial.print(steerAngleSetPoint);
			Serial.print(",");
			Serial.print(heading);   
			Serial.print(",");
			Serial.print(XeRoll);   
			Serial.print(",");
			Serial.print(switchByte); 
			Serial.print(",");
			Serial.println(pwmDisplay);
		}
	}  //end of timed loop
}
