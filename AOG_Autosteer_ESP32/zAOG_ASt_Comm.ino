
void getDataFromAOG()
{
	byte incomingBytes[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 }, incomingByteNum = 0, tempByt = 0;
	int tempInt = 0;
	float tempFlo = 0;
	bool steerSettingChanged = false;
	isDataFound = false;
	isSettingFound = false;

	//get new AOG Data
	//USB
	if ((steerSet.DataTransVia == 0) || (steerSet.DataTransVia == 4)) {		

		while (Serial.available())
		{
			incomingBytes[0] = Serial.read();
			if (isDataFound || isSettingFound || isArdConfigFound)
			{
				DataFromAOG[incomingByteNum] = incomingBytes[0];
				incomingByteNum++;
				if (steerSet.debugmode) {
					Serial.print(incomingBytes[0]); Serial.print(" ");
				}
				if (incomingByteNum > 10) { break; }//sentence too long
			}
			else {//sentence not started yet
				if ((incomingBytes[0] == steerSet.DataFromAOGHeader[0]) || (incomingBytes[0] == steerSet.SettingsFromAOGHeader[0]) || (incomingBytes[0] == steerSet.ArdConfigFromAOGHeader[0]))
				{
					incomingBytes[0] = Serial.read();
					if (incomingBytes[0] == steerSet.DataFromAOGHeader[1]) { 
						isDataFound = true; 
						DataFromAOG[0] = steerSet.DataFromAOGHeader[0];
						DataFromAOG[1] = steerSet.DataFromAOGHeader[1];
						incomingByteNum = 2; }
					else {
						if (incomingBytes[0] == steerSet.SettingsFromAOGHeader[1]) {
							isSettingFound = true;
							DataFromAOG[0] = steerSet.SettingsFromAOGHeader[0];
							DataFromAOG[1] = steerSet.SettingsFromAOGHeader[1];
							incomingByteNum = 2;
						}
						else {
							if (incomingBytes[0] == steerSet.ArdConfigFromAOGHeader[1]) {
								isArdConfigFound = true;
								DataFromAOG[0] = steerSet.DataFromAOGHeader[0];
								DataFromAOG[1] = steerSet.DataFromAOGHeader[1];
								incomingByteNum = 2;
							}
						}
					}
					if (steerSet.debugmode) { Serial.print("data from AOG via USB: "); }
				}
			}
		}
	}//end USB


	//WiFi UDP 
	if ((steerSet.DataTransVia == 1))
	{
		//Serial.println("checking for UDP packet");
		isDataFound = false;
		isSettingFound = false;
		isArdConfigFound = false;

		byte leng = UDPFromAOG.parsePacket();
		//check packet length and process only fitting ones
		if (leng > 0 && steerSet.debugmode) { Serial.println("UDP packet found"); }
		if ((leng >= 6) && (leng <= 10))
		{
			UDPFromAOG.read(incomingBytes, leng);
			if ((incomingBytes[0] == steerSet.DataFromAOGHeader[0]) && (incomingBytes[1] == steerSet.DataFromAOGHeader[1]))
			{
				isDataFound = true;
			}
			else {
				if ((incomingBytes[0] == steerSet.SettingsFromAOGHeader[0]) && (incomingBytes[1] == steerSet.SettingsFromAOGHeader[1]))
				{
					isSettingFound = true;
				}
				else {
					if ((incomingBytes[0] == steerSet.ArdConfigFromAOGHeader[0]) && (incomingBytes[1] == steerSet.ArdConfigFromAOGHeader[1]))
					{
						isArdConfigFound = true;
					}
				}
			}
			if (steerSet.debugmode) { Serial.print("data from AOG via UDP: "); }

			for (byte n = 0; n < leng; n++) {
				if (steerSet.debugmode) { Serial.print(incomingBytes[n]); Serial.print(" "); }
				DataFromAOG[n] = incomingBytes[n];
			}

			if (steerSet.debugmode) { Serial.println(); }
		}

	}//end UDP


	if (isDataFound)
	{
		if (steerSet.debugmode) { Serial.println("steer data from AOG received"); }
		relay = DataFromAOG[2];   // read relay control from AgOpenGPS     
		gpsSpeed = float(DataFromAOG[3]) / 4;  //actual speed times 4, single byte

		//distance from the guidance line in mm
		olddist = distanceFromLine;
		idistanceFromLine = (DataFromAOG[4] << 8 | DataFromAOG[5]);   //high,low bytes     
		distanceFromLine = (float)idistanceFromLine;

		//set point steer angle * 10 is sent
		isteerAngleSetPoint = ((DataFromAOG[6] << 8 | DataFromAOG[7])); //high low bytes 
		steerAngleSetPoint = (float)isteerAngleSetPoint * 0.01;

		DataFromAOGTime = millis();
		watchdogTimer = 0;
	}
#if HardwarePlatform == 1 //nano33iot
	else delay(5);//do WiFi
#endif

	//autosteer settings packet
	if (isSettingFound)
	{
		if (steerSet.aogVersion == 0) {
			steerSettingChanged = false;

			tempFlo = (float)DataFromAOG[2] * 1.0;  // read Kp from AgOpenGPS
			if (tempFlo != steerSet.Kp) { steerSet.Kp = tempFlo; steerSettingChanged = true; }

			tempFlo = (float)DataFromAOG[3] * 0.001;   // read Ki from AgOpenGPSS
			if (tempFlo != steerSet.Ki) { steerSet.Ki = tempFlo; steerSettingChanged = true; }

			tempFlo = (float)DataFromAOG[4] * 1.0;   // read Kd from AgOpenGPS
			if (tempFlo != steerSet.Kd) { steerSet.Kd = tempFlo; steerSettingChanged = true; }

			tempFlo = (float)DataFromAOG[5] * 0.1;   // read Ko from AgOpenGPS
			if (tempFlo != steerSet.Ko) { steerSet.Ko = tempFlo; steerSettingChanged = true; }

			tempFlo = (steerSet.SteerPosZero - 127) + DataFromAOG[6];//read steering zero offset  
			if (tempFlo != steerSet.steeringPositionZero) { steerSet.steeringPositionZero = tempFlo; steerSettingChanged = true; }

			tempByt = DataFromAOG[7]; //read the minimum amount of PWM for instant on
			if (tempByt != steerSet.minPWMValue) { steerSet.minPWMValue = tempByt; steerSettingChanged = true; }

			maxIntegralValue = DataFromAOG[8] * 0.1; //

			tempFlo = DataFromAOG[9]; //sent as 10 times the setting displayed in AOG
			if (tempFlo != steerSet.steerSensorCounts) { steerSet.steerSensorCounts = tempFlo; steerSettingChanged = true; }
		}
		else {
			steerSet.Kp = (float)DataFromAOG[2];       // read Kp from AgOpenGPS
			steerSet.deadZone = (float)DataFromAOG[3];     // read deadZone from AgOpenGPS
			steerSet.Kd = (float)DataFromAOG[4] * 1.0;       // read Kd from AgOpenGPS
			steerSet.Ko = (float)DataFromAOG[5] * 0.1;       // read Ko from AgOpenGPS
			//steerSet.steeringPositionZero = (WAS_ZERO)+DataFromAOG[6];//read steering zero offset  
			tempFlo = (steerSet.SteerPosZero - 127) + DataFromAOG[6];//read steering zero offset  
			if (tempFlo != steerSet.steeringPositionZero) { steerSet.steeringPositionZero = tempFlo; steerSettingChanged = true; }

			steerSet.minPWM = DataFromAOG[7]; //read the minimum amount of PWM for instant on
			steerSet.maxPWM = DataFromAOG[8]; //
			steerSet.steerSensorCounts = DataFromAOG[9];

			steerSettingChanged = true;

			byte checksum = 0;
			for (int i = 2; i < 10; i++) checksum += DataFromAOG[i];

			//send data back - version number. 
			SendTwoThirty((byte)checksum);

			highLowPerDeg = (steerSet.maxPWM - steerSet.deadZone) / steerSet.MotorSlowDriveDegrees;
		}


		if (steerSettingChanged) { EEprom_write_all(); }

		if (steerSet.debugmode) {
			if (steerSettingChanged) { Serial.println("NEW NEW:   got NEW steer settings from AOG"); }
			else { Serial.println("got allready known steer settings from AOG"); }
		}
	}

	//Arduino Config
	if (isArdConfigFound) {
		byte checksum = 0;
		for (int i = 2; i < 10; i++) checksum += DataFromAOG[i];

		//send data back - version number. 
		SendTwoThirty((byte)checksum);
	}

}


//-------------------------------------------------------------------------------------------------

void Send_UDP()
{	//Send Packet
	UDPToAOG.beginPacket(steerSet.ipDestination, steerSet.portDestination);
	for (byte idx = 0; idx < 10; idx++) {
		UDPToAOG.write(toSend[idx]);
	}
	UDPToAOG.endPacket();

	//UDPToAOG.writeTo(toSend, sizeof(toSend), ipDestination, portDestination);
}


//-------------------------------------------------------------------------------------------------
//send back checksum and version

void SendTwoThirty(byte check)
{
	toSend[0] = 127;
	toSend[1] = 230;
	toSend[2] = check;
	toSend[3] = steerSet.aogVersion; //version to match in AOG
	toSend[4] = 0;
	toSend[5] = 0;
	toSend[6] = 0;
	toSend[7] = 0;
	toSend[8] = 0;	
	toSend[9] = 0;

	//off to AOG
	if (steerSet.DataTransVia == 4) {
		for (byte n = 0; n < 9; n++) {
			Serial.print(toSend[n]); Serial.print(",");
		}
		Serial.println(toSend[9]);//drop , after byte 9
	}
	else {	if (steerSet.DataTransVia == 1) { Send_UDP(); }	}

	//back to sending steerData pgn
	toSend[0] = steerSet.DataToAOGHeader[0];//0x7F;
	toSend[1] = steerSet.DataToAOGHeader[1];//0xFD;
}