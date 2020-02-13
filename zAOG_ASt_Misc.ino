void SetRelays(void)
{
	if (bitRead(relay, 0)) digitalWrite(steerSet.Relay1_PIN, HIGH);
	else digitalWrite(steerSet.Relay1_PIN, LOW);
	if (bitRead(relay, 1)) digitalWrite(steerSet.Relay2_PIN, HIGH);
	else digitalWrite(steerSet.Relay2_PIN, LOW);
	//if (bitRead(relay,2)) digitalWrite(led3, HIGH);
	//else digitalWrite(led3, LOW); 
	//if (bitRead(relay,3)) digitalWrite(led4, HIGH);
	//else digitalWrite(led4, LOW); 
}



//-------------------------------------------------------------------------------------------------

void WiFi_LED_blink()
{
	//no data for more than 2 secs = blink
	if (millis() > (DataFromAOGTime + 2000L)) {
		if (!LED_WIFI_ON) {
			if (millis() > (LED_WIFI_time + LED_WIFI_pause)) {
				LED_WIFI_time = millis();
				LED_WIFI_ON = true;
#if useLED_BUILTIN
				digitalWrite(LED_BUILTIN, HIGH);
#endif
				digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
			}
		}
		if (LED_WIFI_ON) {
			if (millis() > (LED_WIFI_time + LED_WIFI_pulse)) {
				LED_WIFI_time = millis();
				LED_WIFI_ON = false;
#if useLED_BUILTIN
				digitalWrite(LED_BUILTIN, LOW);
#endif
				digitalWrite(steerSet.LEDWiFi_PIN, !steerSet.LEDWiFi_ON_Level);
			}
		}
	}

	else {
		digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
#if useLED_BUILTIN
		digitalWrite(LED_BUILTIN, HIGH);
#endif			
	}
}


//-------------------------------------------------------------------------------------------------
// Interrupt handling
//-------------------------------------------------------------------------------------------------

/* interrups didn't work with UDP: replaced by check of button state in timed loop of main loop, so 10Hz checked, no need to debounce
//ISR SteerSwitch_toRemoteAutosteer Interrupt if steerswitch is a button
void Steersw_ISR() // handle pin change interrupt for Steersw Pin 
{
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 300ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > 300)
	{
		//steerEnable = !steerEnable;
		toggleSteerEnable = 1;
	}
	last_interrupt_time = interrupt_time;
}
*/
//-------------------------------------------------------------------------------------------------
/*
//ISR Steering Wheel Encoder
void EncoderA_ISR()
{
#if (ShaftEncoder >=0)      
	pulseACount++;
	//digitalWrite(led1, !digitalRead(led1));

#endif     
}
//ISR Steering Wheel Encoder
void EncoderB_ISR()
{
#if (ShaftEncoder >=0)      
	pulseBCount++;
	//digitalWrite(led2, !digitalRead(led2));
#endif     
}

*/



