void assignGPIOs_start_extHardware() {
	delay(50);

	//init wire for ADS and MMA
	Wire.begin(steerSet.SDA, steerSet.SCL, 400000);

	//init GPIO pins, if 255 = unused/not connected
#if useLED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif	
	if (steerSet.REMOTE_PIN < 255) { pinMode(steerSet.REMOTE_PIN, INPUT_PULLUP); }
	if (steerSet.LEDWiFi_PIN < 255) { pinMode(steerSet.LEDWiFi_PIN, OUTPUT); }
	if ((steerSet.WorkSW_mode > 0) && (steerSet.WORKSW_PIN < 255)) { pinMode(steerSet.WORKSW_PIN, INPUT_PULLUP); }
	if (steerSet.Relay1_PIN < 255) { pinMode(steerSet.Relay1_PIN, OUTPUT); }
	if (steerSet.Relay2_PIN < 255) { pinMode(steerSet.Relay2_PIN, OUTPUT); }

	//no check if < 255 as needed for autosteer in every case
	pinMode(steerSet.AutosteerLED_PIN, OUTPUT);
	pinMode(steerSet.PWM_PIN, OUTPUT);
	pinMode(steerSet.DIR_PIN, OUTPUT);
	delay(2);
	ledcSetup(0, steerSet.PWMOutFrequ, 8);  // PWM Output with channel 0, x kHz, 8-bit resolution (0-255)
	ledcSetup(1, steerSet.PWMOutFrequ, 8);  // PWM Output with channel 1, x kHz, 8-bit resolution (0-255)
	delay(2);
	ledcAttachPin(steerSet.PWM_PIN, 0);  // attach PWM PIN to Channel 0
	ledcAttachPin(steerSet.DIR_PIN, 1);  // attach PWM PIN to Channel 1

	//if (steerSet.input_type == 0)  steerSet.SteerPosZero = 2048;                //Starting Point with ESP ADC 2048 
	//if (steerSet.input_type > 0 && steerSet.input_type < 3)  steerSet.SteerPosZero = 13000;  //with ADS start with 13000  
	
	if (steerSet.SteerSwitch == 0) { pinMode(steerSet.STEERSW_PIN, INPUT_PULLDOWN); }
	if (steerSet.SteerSwitch > 0) { pinMode(steerSet.STEERSW_PIN, INPUT_PULLUP); }

	//Setup Interrupt -Steering Wheel encoder
	if (steerSet.encA_PIN < 255) { pinMode(steerSet.encA_PIN, INPUT_PULLUP); }
	if (steerSet.encB_PIN < 255) { pinMode(steerSet.encB_PIN, INPUT_PULLUP); }

	delay(50);

	// BNO055 init
	if (steerSet.BNOInstalled == 1)
	{
		IMU.init();
		delay(10);
		IMU.setExtCrystalUse(true);   //use external 32K crystal
	}
	if (steerSet.InclinometerInstalled == 1)
	{
		// MMA8452 (1) Inclinometer
		MMAinitialized = MMA1C.init();
		delay(10);

		if (MMAinitialized)
		{
			MMA1C.setDataRate(MMA_800hz);
			MMA1C.setRange(MMA_RANGE_8G);
			MMA1C.setHighPassFilter(false);
			if (steerSet.debugmode) { Serial.println("MMA init OK"); }
		}
		else if (steerSet.debugmode) { Serial.println("MMA init fails!!"); }
	}
	else if (steerSet.InclinometerInstalled == 2)
	{
		// MMA8452 (1) Inclinometer
		MMAinitialized = MMA1D.init();
		delay(10);
		if (MMAinitialized)
		{
			MMA1D.setDataRate(MMA_800hz);
			MMA1D.setRange(MMA_RANGE_8G);
			MMA1D.setHighPassFilter(false);
			if (steerSet.debugmode) { Serial.println("MMA init OK"); }
		}
		else if (steerSet.debugmode) { Serial.println("MMA init fails!!"); }
	}

}