//Apr 2023: pin nr check
void assignGPIOs_start_extHardware() {
	delay(50);

	//init GPIO pins, if 255 = unused/not connected
#if useLED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif	
	if (Set.LEDWiFi_PIN != 255) pinMode(Set.LEDWiFi_PIN, OUTPUT);
	if (Set.AutosteerLED_PIN != 255) pinMode(Set.AutosteerLED_PIN, OUTPUT);
	if (Set.PWM_PIN < 100) {
		pinMode(Set.PWM_PIN, OUTPUT);
		ledcSetup(0, Set.PWMOutFrequ, 8);
		ledcAttachPin(Set.PWM_PIN, 0);
	}
	if (Set.DIR_PIN < 100) {
		pinMode(Set.DIR_PIN, OUTPUT);
		ledcSetup(1, Set.PWMOutFrequ, 8);
		ledcAttachPin(Set.DIR_PIN, 1);
	}
	for (byte i = 0; i < 16; i++) {
		if (Set.Relay_PIN[i] < 100) {
			pinMode(Set.Relay_PIN[i], OUTPUT);
			SectRelaysUsed = true;
		}
	}

	delay(200);

	//init wire for ADS and MMA or BNO or CMPS
	if (!Wire.begin(Set.SDA, Set.SCL, 400000)) {
	//if (!Wire.begin()) {
		Serial.println("error INIT wire, ADS, BNO, CMPS, MMA will not work");
	}

	delay(300);

	//ADS1115
	if (Set.WASType > 0) {
		if (!ads.begin()) {
			Serial.println("ADS1115 not connected!");
		}
		ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V   0.1875mV (default)		
		// Start the first conversion.
		if (Set.WASType == 2) ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
		else ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
	}
	else { pinMode(Set.WAS_PIN, INPUT); } //no ADS, but ESP32 GPIO

	byte error = 0;
	switch (Set.IMUType) {
	case 0:
		//roll no hardware = 8888
		steerToAOG[9] = 0xB8;
		steerToAOG[10] = 0x22;
		roll = 0;
		//heading16 no hardware = 9999     
		steerToAOG[7] = 0x0F;
		steerToAOG[8] = 0x27;
		heading = 0;
		break;

		/*	case 1:	// BNO055 init
			BNO.init();
			delay(10);
			BNO.setExtCrystalUse(true);   //use external 32K crystal
			//roll no hardware = 8888
			steerToAOG[9] = 0xB8;
			steerToAOG[10] = 0x22;
			roll = 0;
			break;*/

	case 2://test if CMPS working		
		Wire.beginTransmission(Set.CMPS14_ADDRESS);
		error = Wire.endTransmission();
		if (error == 0)
		{
			if (Set.debugmode) {
				Serial.println("Error = 0");
				Serial.print("CMPS14 ADDRESs: 0x");
				Serial.println(Set.CMPS14_ADDRESS, HEX);
				Serial.println("CMPS14 Ok.");
			}
		}
		else
		{
			Serial.println("Error = 4");
			Serial.print("CMPS not Connected or Found at address 0x");
			Serial.println(Set.CMPS14_ADDRESS, HEX);
			Set.IMUType = 0;
		}
	}
	/*	break;

	case 3:
		for (int i = 0; i < nrBNO08xAdresses; i++)
		{
			bno08xAddress = Set.bno08xAddresses[i];

			Serial.print("\r\nChecking for BNO08X on ");
			Serial.println(bno08xAddress, HEX);
			Wire.beginTransmission(bno08xAddress);
			error = Wire.endTransmission();

			if (error == 0)
			{
				Serial.println("Error = 0");
				Serial.print("BNO08X ADDRESs: 0x");
				Serial.println(bno08xAddress, HEX);
				Serial.println("BNO08X Ok.");

				// Initialize BNO080 lib
				if (bno08x.begin(bno08xAddress))
				{
					Wire.setClock(400000); //Increase I2C data rate to 400kHz

					// Use gameRotationVector
					bno08x.enableGameRotationVector(REPORT_INTERVAL); //Send data update every REPORT_INTERVAL in ms for BNO085

					// Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
					if (bno08x.getFeatureResponseAvailable() == true)
					{
						if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

						// Break out of loop
					   // useBNO08x = true;
						break;
					}
					else
					{
						Set.IMUType = 0;
						Serial.println("BNO08x init fails!!");
					}
				}
				else
				{
					Serial.println("BNO080 not detected at given I2C address.");
				}
			}
			else
			{
				Serial.println("Error = 4");
				Serial.println("BNO08X not Connected or Found");
			}
		}
		break;
	}//switch IMU


*/
}