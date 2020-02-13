// WIFI handling 11. Feb 2020 for ESP32 and Nano 33 IoT -------------------------------------------

void WiFi_Start_STA() {
	unsigned long timeout;

	WiFi.mode(WIFI_STA);   //  Workstation
/*	if (!WiFi.config(steerSet.myip, steerSet.gwip, steerSet.mask, steerSet.myDNS))
	{
		Serial.println("STA Failed to configure");
	}*/
	delay(300);
	WiFi.begin(steerSet.ssid, steerSet.password);
	timeout = millis() + (steerSet.timeoutRouter * 1000);
	LED_WIFI_time = millis();
	Serial.print("trying to connect to WiFi network: "); Serial.println(steerSet.ssid);
	Serial.print("trying for max seconds: "); Serial.println(steerSet.timeoutRouter);
	while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
		delay(250);
		Serial.print(".");
		//WIFI LED blink in double time while connecting
		if (!LED_WIFI_ON) {
			if (millis() > (LED_WIFI_time + (LED_WIFI_pause >> 2))) {
				LED_WIFI_time = millis();
				LED_WIFI_ON = true;
				#if useLED_BUILTIN
					digitalWrite(LED_BUILTIN, HIGH);
				#endif
				digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
			}
		}
		if (LED_WIFI_ON) {
			if (millis() > (LED_WIFI_time + (LED_WIFI_pulse >> 2))) {
				LED_WIFI_time = millis();
				LED_WIFI_ON = false;
				#if useLED_BUILTIN
					digitalWrite(LED_BUILTIN, LOW);
				#endif
				digitalWrite(steerSet.LEDWiFi_PIN, !steerSet.LEDWiFi_ON_Level);
			}
		}
	}

	Serial.println("");
	if (WiFi.status() == WL_CONNECTED)
	{
		delay(250);
		Serial.print("Connected IP - Address : ");
		IPAddress IPTemp = WiFi.localIP();
		for (byte n = 0; n < 4; n++) {
			steerSet.myip[n] = IPTemp[n];
			Serial.print(steerSet.myip[n]);
			Serial.print(".");			
		}
		IPTemp = WiFi.gatewayIP();
		steerSet.myip[3] = steerSet.myIPEnding; //set ESP32 IP to x.x.x.myIP_ending
		Serial.print("changing IP to: ");
		for (byte n = 0; n < 4; n++) {
			Serial.print(steerSet.myip[n]);
			Serial.print(".");
			steerSet.gwip[n] = IPTemp[n];
		}
		if (!WiFi.config(steerSet.myip, steerSet.gwip, steerSet.mask, steerSet.myDNS))
		{
			Serial.println("STA Failed to configure");
		}
		delay(300);
		Serial.print("WiFi Client successfully connected to : ");
		Serial.println(steerSet.ssid);
		Serial.print("Connected IP - Address : ");
		IPTemp = WiFi.localIP();
		for (byte n = 0; n < 4; n++) {
			Serial.print(steerSet.myip[n]);
			Serial.print(".");
			steerSet.myip[n] = IPTemp[n];
			steerSet.ipDestination[n] = IPTemp[n];
		}
		Serial.println();		
		steerSet.ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
		LED_WIFI_ON = true;
		digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
		#if useLED_BUILTIN
			digitalWrite(LED_BUILTIN, HIGH);
		#endif
		delay(300);
		server.begin();
		my_WiFi_Mode = WIFI_STA;

	}
	else
	{
		WiFi.mode(WIFI_OFF);
		Serial.println("WLAN-Client-Connection failed");
	}
	delay(100);
}

//-------------------------------------------------------------------------------------------------

void WiFi_Start_AP() {
	WiFi.mode(WIFI_AP);   // Accesspoint
	WiFi.softAP(steerSet.ssid_ap, "");
	while (!SYSTEM_EVENT_AP_START) // wait until AP has started
	{
		delay(250);
		Serial.print(".");
	}
	delay(500);
	WiFi.softAPConfig(steerSet.gwip, steerSet.gwip, steerSet.mask);  // set fix IP for AP
	
	//WiFi.softAPConfig(steerSet.gwip, steerSet.gwip,);  // set fix IP for AP
	delay(500);
	IPAddress myIP = WiFi.softAPIP();
	my_WiFi_Mode = WIFI_AP;

	server.begin();
	Serial.println();
	Serial.print("Accesspoint started - Name : ");
	Serial.print(steerSet.ssid_ap);
	Serial.print(" IP address: ");
	Serial.println(myIP);
}

//-------------------------------------------------------------------------------------------------

void UDP_Start()
{
	if (UDPToAOG.begin(steerSet.portMy))
	{
		Serial.print("UDP sendig to IP: ");
		for (byte n = 0; n < 4; n++) {
			Serial.print(steerSet.ipDestination[n]);
			Serial.print(".");
		}		
		Serial.print(" from port: ");
		Serial.print(steerSet.portMy);
		Serial.print(" to port: ");
		Serial.println(steerSet.portDestination);
	}
	delay(300);
	if (UDPFromAOG.begin(steerSet.portAOG))
	{
		Serial.print("UDP listening for AOG data on IP: ");
		Serial.println(WiFi.localIP());
		Serial.print(" on port: ");
		Serial.println(steerSet.portAOG);
		getDataFromAOG();
	}
}

