void SetRelays(void)
{
	if (bitRead(SectGrFromAOG[0], 0)) digitalWrite(Set.Relay_PIN[0], Set.Relays_ON);
	else digitalWrite(Set.Relay_PIN[0], !Set.Relays_ON);
	if (bitRead(SectGrFromAOG[0], 1)) digitalWrite(Set.Relay_PIN[1], Set.Relays_ON);
	else digitalWrite(Set.Relay_PIN[1], !Set.Relays_ON);

}

void UpdateStepperSettings (){

  //calculate values
  stepPerPositionDegree = Set.Kp + Set.stepperKpToDegreesOffset; 
  Set.stepperMaxSpeed = Set.highPWM * Set.stepperhighPWMToMaxSpeedFactor; 
  Set.stepperAcceleration = Set.lowPWM * Set.stepperlowPWMToAccelerationFactor; 

  //set values
 
  if (stepperPossible) {
    if (Set.MotorDriveDirection == 1){
      stepper->setDirectionPin(Set.stepperDirPIN, true);
      if (Set.debugmode) {
        Serial.print("stepperDirPIN (HIGH counts up) is set to ");
        Serial.println(Set.stepperDirPIN);
      }
    }
    else {
      stepper->setDirectionPin(Set.stepperDirPIN, false);
      if (Set.debugmode) {
        Serial.print("stepperStepPIN (LOW counts up) is set to ");
        Serial.println(Set.stepperDirPIN);
      }
    }
  
    stepper->setSpeedInHz(Set.stepperMaxSpeed); 
    stepper->setAcceleration(Set.stepperAcceleration);
    
    if (Set.debugmode) { 
      Serial.println("Update StepperSettings: "); 
      Serial.print("stepPerPositionDegree: ");
      Serial.println(stepPerPositionDegree); 
      Serial.print("stepperMaxSpeed: ");
      Serial.println(Set.stepperMaxSpeed); 
      Serial.print("stepperAcceleration: ");
      Serial.println(Set.stepperAcceleration); 
      Serial.print("MotorDriveDirection: ");
      Serial.println(Set.MotorDriveDirection);
    }
  }
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
