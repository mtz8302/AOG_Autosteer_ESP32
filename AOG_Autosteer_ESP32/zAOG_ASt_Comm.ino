//getData 7. Maerz 2021
void getDataFromAOGUSB(void* pvParameters)
{
	byte nextincommingBytesArrayNr;
	unsigned int packetLength;

	vTaskDelay(5000);//wait for other tasks to start

	USBDataTaskRunning = true;
	if (Set.debugmode) { Serial.println("started Task get Data via USB"); }
	bitSet(mainLoopDelay, 0);
	for (;;) {
		//get Data
		packetLength = Serial.available();
		if (packetLength > 0) {
			nextincommingBytesArrayNr = (incommingBytesArrayNr + 1) % incommingDataArraySize;
			for (int i = 0; i < packetLength; i++) { incommingBytes[nextincommingBytesArrayNr][i] = Serial.read(); }
			incommingDataLength[incommingBytesArrayNr] = packetLength;			
			incommingBytesArrayNr = nextincommingBytesArrayNr;
		}
		else { vTaskDelay(5); }
	}
}

//-------------------------------------------------------------------------------------------------

void getDataFromAOGEth(void* pvParameters)
{
	byte nextincommingBytesArrayNr;
	unsigned int packetLength;

	EthDataTaskRunning = true;

	for (;;) {
		if (!EthUDPRunning) { vTaskDelay(3000); }
		else { break; }
	}
	if (Set.debugmode) { Serial.println("started Task get Data via Ethernet"); }
	bitSet(mainLoopDelay, 0);
	for (;;) {
		//get Data		
		packetLength = EthUDPFromAOG.parsePacket();
		if (packetLength) {
			nextincommingBytesArrayNr = (incommingBytesArrayNr + 1) % incommingDataArraySize;
			EthUDPFromAOG.read(incommingBytes[nextincommingBytesArrayNr], packetLength);
			incommingDataLength[nextincommingBytesArrayNr] = packetLength;		
			incommingBytesArrayNr = nextincommingBytesArrayNr;
		}
		else { vTaskDelay(5); }
	}
}


//-------------------------------------------------------------------------------------------------
//parseData 7. Maerz 2021, 25. Feb 2023: AgIO request added, V17 deleted

void parseDataFromAOG() {
	for (int i = 0; i < incommingDataLength[incommingBytesArrayNrToParse]; i++) {
		//sentence comming? V5: 80 81 7F PGN
		if (incomSentenceDigit < 3) {
			if (incommingBytes[incommingBytesArrayNrToParse][i] == FromAOGSentenceHeader[incomSentenceDigit]) {
				//Serial.println("first 3 Bytes fit: sentence");
				SentenceFromAOG[incomSentenceDigit] = incommingBytes[incommingBytesArrayNrToParse][i];
				incomSentenceDigit++;
			}
			else { incomSentenceDigit = 0; }
		}//<3
		else {
			//write incoming Data to sentence array if it fits in
			if (incomSentenceDigit <= SentenceFromAOGMaxLength) {
				SentenceFromAOG[incomSentenceDigit] = incommingBytes[incommingBytesArrayNrToParse][i];
			}
			if (incomSentenceDigit == 3) {
				incomSentenceDigit++;
				//which sentence comming? PGN
				switch (incommingBytes[incommingBytesArrayNrToParse][i]) {
				case steerDataFromAOGHeader:
					isSteerDataFound = true;
					break;
				case steerSettingsFromAOGHeader:
					isSteerSettingFound = true;
					break;
				case steerArdConfFromAOGHeader:
					isSteerArdConfFound = true;
					break;
				case AgIO_heartbeat:
					if (Set.AgIOHeartbeat_answer > 0) { AutoStHeartbeatSend(); }//called here, as AgIO checksum is not correct
					isAgIOHeartbeatFound = true;
					break;
				case AgIO_ScanRequest:
					isAgIOScanRequestFound = true;
					break;
				default:
					//Serial.println("no matching PGN");
					incomSentenceDigit = 0;
					break;
				}//switch
			}//==3
			else {// >3
				if (incomSentenceDigit == 4) {//lenght
					SentenceFromAOGLength = incommingBytes[incommingBytesArrayNrToParse][i];
					incomSentenceDigit++;

				}//==4
				else
				{//>4	
					if (incomSentenceDigit == (SentenceFromAOGLength + 5)) { //sentence complete Length: + 4 byte header + 1 length + 1 CRC - 1 (starting at 0) 
						//sentence complete
						int CRCDataFromAOG = 0;
						for (byte chk = 2; chk < (SentenceFromAOGLength + 5); chk++)
						{
							CRCDataFromAOG = (CRCDataFromAOG + SentenceFromAOG[chk]);
						}
						if (byte(CRCDataFromAOG) != incommingBytes[incommingBytesArrayNrToParse][i]) 
						{//checksum error
							if (Set.debugmodeDataFromAOG) {
								Serial.print("Checksum failed: exp: ");
								Serial.print(byte(CRCDataFromAOG));
								Serial.print(" chk rvd: ");
								Serial.print(incommingBytes[incommingBytesArrayNrToParse][SentenceFromAOGLength + 5]);
								Serial.print(" AOG data lgth: "); Serial.println(SentenceFromAOGLength);
							}
							isSteerDataFound = false;
							isSteerSettingFound = false;
							isSteerArdConfigFound = false;
							isAgIOHeartbeatFound = false;
							isAgIOScanRequestFound = false;
							incomSentenceDigit = 255;
						}//checksum error

						if (Set.debugmodeDataFromAOG) {
							Serial.println("data recieved in parser: ");
							for (byte b = 0; b < (SentenceFromAOGLength + 6); b++) {
								Serial.print(SentenceFromAOG[b]); Serial.print(" ");
							}
							Serial.println();
						}

						if (isSteerDataFound) {
							SectGrFromAOG[0] = SentenceFromAOG[11];   // read Section control from AgOpenGPS 
							SectGrFromAOG[1] = SentenceFromAOG[12];   // read Section control from AgOpenGPS 

							Tram = SentenceFromAOG[10];

							gpsSpeed = ((float)(SentenceFromAOG[6] << 8 | SentenceFromAOG[5])) * 0.1;

							guidanceStatus = SentenceFromAOG[7];

							//Bit 8,9    set point steer angle * 100 is sent
							steerAngleSetPoint = ((float)(SentenceFromAOG[9] << 8 | SentenceFromAOG[8])) * 0.01; //high low bytes
							if (steerAngleSetPoint > 500) { steerAngleSetPoint -= 655.35; }
							//	Serial.print("SteerSetPoint: "); Serial.println(steerAngleSetPoint);

							newDataFromAOG = true;
							isSteerDataFound = false;
							incomSentenceDigit = 255;
							DataFromAOGTime = millis();
							watchdogTimer = 0;
							if (Set.debugmodeDataFromAOG) {
								Serial.print("speed: "); Serial.print(gpsSpeed);
								Serial.print(" GuidStat: "); Serial.print(guidanceStatus);
								Serial.print(" SteerAngSet: "); Serial.print(steerAngleSetPoint);
								Serial.print(" SectGrFromAOG[0]: "); Serial.print(SectGrFromAOG[0]);
								Serial.print(" SectGrFromAOG[1]: "); Serial.println(SectGrFromAOG[1]);
							}
						}
						else {
							if (isSteerSettingFound) {
								//PID values
								Set.Kp = ((float)SentenceFromAOG[5]);   // read Kp from AgOpenGPS
								Set.highPWM = SentenceFromAOG[6];
								Set.lowPWM = (float)SentenceFromAOG[7];   // read lowPWM from AgOpenGPS
								Set.minPWM = SentenceFromAOG[8]; //read the minimum amount of PWM for instant on
								Set.steerSensorCounts = float(SentenceFromAOG[9]); //sent as setting displayed in AOG
						//		Set.wasOffset = (SentenceFromAOG[10]);  //read was zero offset Hi
						//		Set.wasOffset |= (SentenceFromAOG[11] << 8);  //read was zero offset Lo
								Set.AckermanFix = SentenceFromAOG[12];

								EEprom_write_all();

								// for PWM High to Low interpolator
								highLowPerDeg = ((float)(Set.highPWM - Set.lowPWM)) / Set.MotorSlowDriveDegrees;

								if (Set.debugmodeDataFromAOG) { Serial.println("got NEW steer settings from AOG"); }
								isSteerSettingFound = false;
								incomSentenceDigit = 255;
							}
							else {
								if (isSteerArdConfigFound) {
									if (bitRead(SentenceFromAOG[5], 0)) Set.InvertWAS = 1; else Set.InvertWAS = 0;
									if (bitRead(SentenceFromAOG[5], 1)) Set.Relays_ON = 1; else Set.Relays_ON = 0;
									if (bitRead(SentenceFromAOG[5], 2)) Set.MotorDriveDirection = 1; else Set.MotorDriveDirection = 0;
									//if (bitRead(SentenceFromAOG[5], 3)) Set.SingleInputWAS = 1; else Set.SingleInputWAS = 0;
									//if (bitRead(SentenceFromAOG[5], 4)) Set.CytronDriver = 1; else Set.CytronDriver = 0;
									if (bitRead(SentenceFromAOG[5], 5)) Set.SteerSwitchType = 1;//switch to GND
									if (bitRead(SentenceFromAOG[5], 6)) Set.SteerSwitchType = 2;//button
									else {
										if (!bitRead(SentenceFromAOG[5], 5)) { Set.SteerSwitchType = 255; }//none
									}
									if (bitRead(SentenceFromAOG[5], 7)) Set.ShaftEncoder = 1; else Set.ShaftEncoder = 0;

									Set.pulseCountMax = SentenceFromAOG[6];

									//if (bitRead(SentenceFromAOG[8], 0)) Set.IsDanfoss = 1; else Set.IsDanfoss = 0;
									//if (bitRead(SentenceFromAOG[8], 1)) Set.PressureSensor = 1; else Set.PressureSensor = 0;
									//if (bitRead(SentenceFromAOG[8], 2)) Set.CurrentSensor = 1; else Set.CurrentSensor = 0;

									EEprom_write_all();

									if (Set.debugmodeDataFromAOG) { Serial.println("got NEW Arduino settings from AOG V5 or higher"); }

									isSteerArdConfigFound = false;
									incomSentenceDigit = 255;
								}
								else {
									if (isAgIOHeartbeatFound) {
										Serial.println("AgIO heartbeat request found");
										AutoStHeartbeatSend();
										isAgIOHeartbeatFound = false;
										incomSentenceDigit = 255;
										DataFromAOGTime = millis();
									}
									else {
										if (AgIO_ScanRequest) {
											AutoStScanRequestReply();
											isAgIOScanRequestFound = false;
											incomSentenceDigit = 255;
											DataFromAOGTime = millis();
										}
									}
								}
							}
						}
					}//sentence complete

					incomSentenceDigit++;

					//sentence too long
					if (incomSentenceDigit > (SentenceFromAOGLength + 6)) { incomSentenceDigit = 0; }

				}//>4
			}//==3
		}//<3
	}//for packetLength
	incommingBytesArrayNrToParse = (incommingBytesArrayNrToParse + 1) % incommingDataArraySize;
}

//-------------------------------------------------------------------------------------------------
// 23. Feb 2023

void AutoStHeartbeatSend()
{
	if (Set.AgIOHeartbeat_answer > 0) {
		int16_t sa = (int16_t)(steerAngleActual * 100);

		helloFromAutoSteer[5] = (uint8_t)sa;
		helloFromAutoSteer[6] = sa >> 8;

		helloFromAutoSteer[7] = (uint8_t)actualSteerPosRAW;
		helloFromAutoSteer[8] = actualSteerPosRAW >> 8;
		helloFromAutoSteer[9] = switchByte;
		//checksum
		int16_t CK_A = 0;
		for (uint8_t i = 2; i < sizeof(helloFromAutoSteer) - 1; i++)
		{
			CK_A = (CK_A + helloFromAutoSteer[i]);
		}
		helloFromAutoSteer[sizeof(helloFromAutoSteer) - 1] = CK_A;

		//USB
		if (Set.DataTransVia < 5) {
			Serial.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
		}
		else {
			if (Set.DataTransVia < 10)//WiFi UDP
			{
				WiFiUDPToAOG.writeTo(helloFromAutoSteer, sizeof(helloFromAutoSteer), WiFi_ipDestination, Set.PortDestination);
			}
			else //Ethernet
			{
				EthUDPToAOG.beginPacket(Eth_ipDestination, Set.PortDestination);
				EthUDPToAOG.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
				EthUDPToAOG.endPacket();
			}
		}
	}
}


//-------------------------------------------------------------------------------------------------
// 23. Feb 2023

void AutoStScanRequestReply()
{
	uint8_t AutoSteerScanReply[] = { 128, 129, 126, 203, 7,
								WiFi_ipDestination[0],WiFi_ipDestination[1],WiFi_ipDestination[2],Set.WiFi_myip[3],0,0,0, 23 };
	if (Set.DataTransVia >= 10) {
		AutoSteerScanReply[5] = Eth_ipDestination[0]; AutoSteerScanReply[6] = Eth_ipDestination[1];
		AutoSteerScanReply[7] = Eth_ipDestination[2]; AutoSteerScanReply[8] = Set.Eth_myip[3];
	}
	//checksum
	int16_t CK_A = 0;
	for (uint8_t i = 2; i < sizeof(AutoSteerScanReply) - 1; i++)
	{
		CK_A = (CK_A + AutoSteerScanReply[i]);
	}
	AutoSteerScanReply[sizeof(AutoSteerScanReply) - 1] = CK_A;

	//USB
	if (Set.DataTransVia < 5) {
		Serial.write(AutoSteerScanReply, sizeof(AutoSteerScanReply));
	}
	else {
		if (Set.DataTransVia < 10)//WiFi UDP
		{
			WiFiUDPToAOG.writeTo(AutoSteerScanReply, sizeof(AutoSteerScanReply), WiFi_ipDestination, Set.PortDestination);
		}
		else //Ethernet
		{
			EthUDPToAOG.beginPacket(Eth_ipDestination, Set.PortDestination);
			EthUDPToAOG.write(AutoSteerScanReply, sizeof(AutoSteerScanReply));
			EthUDPToAOG.endPacket();
		}
	}
}