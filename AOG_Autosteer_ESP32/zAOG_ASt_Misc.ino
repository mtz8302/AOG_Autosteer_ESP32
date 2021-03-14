void SetRelays(void)
{
	if (bitRead(SectGrFromAOG[0], 0)) digitalWrite(Set.Relay1_PIN, Set.Relays_ON);
	else digitalWrite(Set.Relay1_PIN, !Set.Relays_ON);
	if (bitRead(SectGrFromAOG[0], 1)) digitalWrite(Set.Relay2_PIN, Set.Relays_ON);
	else digitalWrite(Set.Relay2_PIN, !Set.Relays_ON);

}

//-------------------------------------------------------------------------------------------------
//9. Maerz 2021

void WiFi_LED_blink(void* pvParameters)
{
	unsigned long now;
	byte blkSpeed;

	for (;;) {

		now = millis();

		if (WiFi_connect_step > 0) { blkSpeed = 2; }
		else {
			if (now > (DataFromAOGTime + 1000L)) {
				if (DataFromAOGTime != 0) { blkSpeed = 3; }
				else { blkSpeed = 0; }
			}
			else { //all OK = LED on
				vTaskDelay(800);
				blkSpeed = 255;
				LED_WIFI_ON = true;
				digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
#if useLED_BUILTIN
				digitalWrite(LED_BUILTIN, HIGH);
#endif
			}
		}

		if (blkSpeed != 255) {
			if (!LED_WIFI_ON) {
				LED_WIFI_time = now;
				LED_WIFI_ON = true;
#if useLED_BUILTIN
				digitalWrite(LED_BUILTIN, HIGH);
#endif
				digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);

				vTaskDelay(LED_WIFI_pause >> blkSpeed);

			}
			if (LED_WIFI_ON) {
				LED_WIFI_time = now;
				LED_WIFI_ON = false;
#if useLED_BUILTIN
				digitalWrite(LED_BUILTIN, LOW);
#endif
				digitalWrite(Set.LEDWiFi_PIN, !Set.LEDWiFi_ON_Level);

				vTaskDelay(LED_WIFI_pulse >> blkSpeed);
			}
		}
	}
}

//-------------------------------------------------------------------------------------------------

/*void WiFi_LED_blink(byte blkSpeed)   //8. Maerz 2020
{
	unsigned long now = millis();
	if (!LED_WIFI_ON) {
		if (now > (LED_WIFI_time + (LED_WIFI_pause >> blkSpeed))) {
			LED_WIFI_time = now;
			LED_WIFI_ON = true;
#if useLED_BUILTIN
			digitalWrite(LED_BUILTIN, HIGH);
#endif
			digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
		}
	}
	if (LED_WIFI_ON) {
		if (now > (LED_WIFI_time + (LED_WIFI_pulse >> blkSpeed))) {
			LED_WIFI_time = now;
			LED_WIFI_ON = false;
#if useLED_BUILTIN
			digitalWrite(LED_BUILTIN, LOW);
#endif
			digitalWrite(Set.LEDWiFi_PIN, !Set.LEDWiFi_ON_Level);
		}
	}
}*/

//-------------------------------------------------------------------------------------------------
/*
void handleDataFromAOG(byte nr) {
	gpsSpeed = float(SteerDataFromAOG[nr].speed) * 0.1;
	steerAngleSetPoint = float(SteerDataFromAOG[nr].steerAngleSetPoint) * 0.01;

}

//-------------------------------------------------------------------------------------------------

void handleSteerSettingsFromAOG(byte nr) {
	//float tempFlo;

	Set.Kp = (float)SteerSettingsFromAOG[nr].Kp;       // read Kp from AgOpenGPS
	//Set.deadZone = (float)SteerSettingsFromAOG[nr].;     // read deadZone from AgOpenGPS
	//Set.Kd = (float)SteerSettingsFromAOG[nr]. * 1.0;       // read Kd from AgOpenGPS
	//Set.Ko = (float)SteerSettingsFromAOG[nr] * 0.1;       // read Ko from AgOpenGPS
	//Set.steeringPositionZero = (WAS_ZERO)+DataFromAOG[6];//read steering zero offset  
	//tempFlo = (Set.WebIOSteerPosZero - 127) + SteerSettingsFromAOG[nr].;//read steering zero offset  
	//if (tempFlo != Set.steeringPositionZero) { Set.steeringPositionZero = tempFlo; steerSettingChanged = true; }
	Set.highPWM = SteerSettingsFromAOG[nr].highPWM;
	Set.lowPWM = SteerSettingsFromAOG[nr].lowPWM; //read the minimum amount of PWM for instant on
	Set.minPWM = SteerSettingsFromAOG[nr].minPWM;

	if ((SteerSettingsFromAOG[nr].steerSensorCounts > 20) && (SteerSettingsFromAOG[nr].steerSensorCounts < 255)) {
		Set.steerSensorCounts = SteerSettingsFromAOG[nr].steerSensorCounts;	}
	if ((SteerSettingsFromAOG[nr].AckermannFix > 50) && (SteerSettingsFromAOG[nr].AckermannFix < 200)){
	Set.AckermanFix = SteerSettingsFromAOG[nr].AckermannFix; }

	//steerSettingChanged = true;

	//byte checksum = 0;
	//for (int i = 2; i < 10; i++) checksum += SteerSettingsFromAOG[nr];

	//send data back - version number. 
	//SendTwoThirty((byte)checksum);

	highLowPerDeg = (Set.highPWM - Set.lowPWM) / Set.MotorSlowDriveDegrees;



	//if (steerSettingChanged) { 
	EEprom_write_all();
	//}

	if (Set.debugmode) { Serial.println("NEW NEW:   got NEW steer settings from AOG");
	}


}


//-------------------------------------------------------------------------------------------------

void handleSteerArdConfigFromAOG(byte nr) {
	//float tempFlo;

	//Set.autoSteerMinSpeed=SteerArdConfigFromAOG[nr].minSteerSpeed;       // read Kp from AgOpenGPS
/*
	if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
	if (bitRead(sett, 1)) steerConfig.isRelayActiveHigh = 1; else steerConfig.isRelayActiveHigh = 0;
	if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
	if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
	if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
	if (bitRead(sett, 5)) steerConfig.SteerSwitchType = 1; else steerConfig.SteerSwitchType = 0;
	if (bitRead(sett, 6)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;
*/
//if (steerSettingChanged) { 
/*	EEprom_write_all();
	//}

	if (Set.debugmode) { Serial.println("NEW NEW:   got NEW steer Config from AOG"); }


}
*/




