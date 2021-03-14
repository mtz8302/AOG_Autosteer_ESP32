void assignGPIOs_start_extHardware() {
	delay(50);

	//init wire for ADS and MMA or BNO or CMPS
	if (!Wire.begin(Set.SDA, Set.SCL, 400000)) {
		Serial.println("error INIT wire, ADS, BNO, CMPS, MMA will not work");
	}
	delay(20);

	//init GPIO pins, if 255 = unused/not connected
#if useLED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif	
	if (Set.REMOTE_PIN < 255) { pinMode(Set.REMOTE_PIN, INPUT_PULLUP); }
	if (Set.LEDWiFi_PIN < 255) { pinMode(Set.LEDWiFi_PIN, OUTPUT); }
	if ((Set.WorkSW_mode > 0) && (Set.WORKSW_PIN < 255)) { pinMode(Set.WORKSW_PIN, INPUT_PULLUP); }
	if (Set.Relay1_PIN < 255) { pinMode(Set.Relay1_PIN, OUTPUT); }
	if (Set.Relay2_PIN < 255) { pinMode(Set.Relay2_PIN, OUTPUT); }

	//no check if < 255 as needed for autosteer in every case
	pinMode(Set.AutosteerLED_PIN, OUTPUT);
	pinMode(Set.PWM_PIN, OUTPUT);
	pinMode(Set.DIR_PIN, OUTPUT);
	delay(2);
	ledcSetup(0, Set.PWMOutFrequ, 8);  // PWM Output with channel 0, x kHz, 8-bit resolution (0-255)
	ledcSetup(1, Set.PWMOutFrequ, 8);  // PWM Output with channel 1, x kHz, 8-bit resolution (0-255)
	delay(2);
	ledcAttachPin(Set.PWM_PIN, 0);  // attach PWM PIN to Channel 0
	ledcAttachPin(Set.DIR_PIN, 1);  // attach PWM PIN to Channel 1

	//if (Set.WASType == 0)  Set.WebIOSteerPosZero = 2048;                //Starting Point with ESP ADC 2048 
	//if (Set.WASType > 0 && Set.WASType < 3)  Set.WebIOSteerPosZero = 13000;  //with ADS start with 13000  

	if (Set.SteerSwitchType == 0) { pinMode(Set.STEERSW_PIN, INPUT_PULLDOWN); }
	if (Set.SteerSwitchType > 0) { pinMode(Set.STEERSW_PIN, INPUT_PULLUP); }

	//Setup Interrupt -Steering Wheel encoder
	if (Set.encA_PIN < 255) { pinMode(Set.encA_PIN, INPUT_PULLUP); }
	if (Set.encB_PIN < 255) { pinMode(Set.encB_PIN, INPUT_PULLUP); }

	delay(50);

	//test if CMPS working
	if (Set.IMUType == 2) {
		byte error;
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

	// BNO055 init
	if (Set.IMUType == 1)
	{
		BNO.init();
		delay(10);
		BNO.setExtCrystalUse(true);   //use external 32K crystal
	}
	if (Set.MMAInstalled == 1)
	{
		// MMA8452 (1) Inclinometer
		MMAinitialized = MMA1C.init();
		delay(10);

		if (MMAinitialized)
		{
			MMA1C.setDataRate(MMA_800hz);
			MMA1C.setRange(MMA_RANGE_8G);
			MMA1C.setHighPassFilter(false);
			if (Set.debugmode) { Serial.println("MMA init OK"); }
		}
		else { Serial.println("MMA init fails at I2C address 1C!!"); Set.MMAInstalled = 0; }
	}
	else if (Set.MMAInstalled == 2)
	{
		// MMA8452 (1) Inclinometer
		MMAinitialized = MMA1D.init();
		delay(10);
		if (MMAinitialized)
		{
			MMA1D.setDataRate(MMA_800hz);
			MMA1D.setRange(MMA_RANGE_8G);
			MMA1D.setHighPassFilter(false);
			if (Set.debugmode) { Serial.println("MMA init OK"); }
		}
		else { Serial.println("MMA init fails at I2C address 1D!!"); Set.MMAInstalled = 0; }
	}

	//ADS1115
	adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
	adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}