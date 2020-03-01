// WIFI handling 1. März 2020 for ESP32 and Nano 33 IoT -------------------------------------------

void WiFi_Start_STA() {
    unsigned long timeout, timeout2;
#if HardwarePlatform == 0  //ESP32  
    WiFi.mode(WIFI_STA);   //  Workstation
#endif
        Serial.print("try to connect to WiFi: "); Serial.println(steerSet.ssid);
        WiFi.begin(steerSet.ssid, steerSet.password);
        timeout = millis() + (steerSet.timeoutRouter * 1000);
        timeout2 = timeout - (steerSet.timeoutRouter * 500);
    while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
        delay(300);
        Serial.print(".");
        if ((millis() > timeout2) && (WiFi.status() != WL_CONNECTED)) {
#if HardwarePlatform == 0  //ESP32  
            WiFi.disconnect();
#endif
#if HardwarePlatform == 1  //nano 33iot 
            WiFi.end();
#endif
            delay(200);
            WiFi.begin(steerSet.ssid, steerSet.password); 
            timeout2 = timeout + 100;
        }
        //WIFI LED blink in double time while connecting
        if (!LED_WIFI_ON) {
            if (millis() > (LED_WIFI_time + (LED_WIFI_pause >> 2)))
            {
                LED_WIFI_time = millis();
                LED_WIFI_ON = true;
                digitalWrite(steerSet.LEDWiFi_PIN, !steerSet.LEDWiFi_ON_Level);
            }
        }
        if (LED_WIFI_ON) {
            if (millis() > (LED_WIFI_time + (LED_WIFI_pulse >> 2))) {
                LED_WIFI_time = millis();
                LED_WIFI_ON = false;
                digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
            }
        }
    }  //connected or timeout  

    Serial.println(""); //NL  
    if (WiFi.status() == WL_CONNECTED)
    {
        delay(200);
        Serial.println();
        Serial.print("WiFi Client successfully connected to : ");
        Serial.println(steerSet.ssid); 
        Serial.print("Connected IP - Address : ");
        IPAddress myIP = WiFi.localIP();
        Serial.println(myIP);
        IPAddress gwIP = WiFi.gatewayIP();
        //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
        myIP[3] = steerSet.myIPEnding; //set ESP32 IP to x.x.x.myIP_ending
        Serial.print("changing IP to: ");
        Serial.println(myIP);
#if HardwarePlatform == 0  //ESP32 
        if (!WiFi.config(myIP, gwIP, steerSet.mask, gwIP)) { Serial.println("STA Failed to configure"); }
#endif
#if HardwarePlatform == 1  //nano 33iot
        WiFi.config(myIP, gwIP, gwIP, steerSet.mask);
#endif
        delay(200);
        Serial.print("Connected IP - Address : ");
        myIP = WiFi.localIP();
        Serial.println(myIP);
        Serial.print("Gateway IP - Address : ");
        Serial.println(gwIP);
        steerSet.ipDestination[0] = myIP[0];
        steerSet.ipDestination[1] = myIP[1];
        steerSet.ipDestination[2] = myIP[2];
        steerSet.ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
        LED_WIFI_ON = true;
        digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
        my_WiFi_Mode = 1;// WIFI_STA;
    }
    else
    {
        // WiFi.end();
        Serial.println("WLAN-Client-Connection failed");
        Serial.println();
        LED_WIFI_ON = false;
        digitalWrite(steerSet.LEDWiFi_PIN, !steerSet.LEDWiFi_ON_Level);
    }
    delay(20);
}


//-------------------------------------------------------------------------------------------------
// start WiFi Access Point = only if no existing WiFi

//ESP32
#if HardwarePlatform == 0 
void WiFi_Start_AP() {
    WiFi.mode(WIFI_AP);   // Accesspoint
    WiFi.softAP(steerSet.ssid_ap, "");
    while (!SYSTEM_EVENT_AP_START) // wait until AP has started
    {
        delay(100);
        Serial.print(".");
    }
    delay(100);//right IP adress only with this delay 
    WiFi.softAPConfig(steerSet.gwip, steerSet.gwip, steerSet.mask);  // set fix IP for AP  

    IPAddress getmyIP = WiFi.softAPIP();
    delay(300);

    //AP_time = millis();
    Serial.print("Accesspoint started - Name : ");
    Serial.println(steerSet.ssid_ap);
    Serial.print(" IP address: ");
    Serial.println(getmyIP);
    LED_WIFI_ON = true;
    digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
    my_WiFi_Mode = WIFI_AP;
}
#endif

// AccessPoint on Nano 33 IoT

#if HardwarePlatform == 1
void WiFi_Start_AP() {

    my_WiFi_Mode = WiFi.beginAP(steerSet.ssid_ap, "");
    delay(500);
    if (my_WiFi_Mode != WL_AP_LISTENING) {
        Serial.println("Creating access point failed");
        delay(500);
        WiFi.end();
        delay(500);
        my_WiFi_Mode = WiFi.beginAP(steerSet.ssid_ap, "");
        if (my_WiFi_Mode != WL_AP_LISTENING) {
            Serial.println("Creating access point failed");
        }
    }
    delay(300);//right IP adress only with this delay 
    WiFi.config(steerSet.gwIP, steerSet.gwIP, steerSet.gwIP, steerSet.mask);  // set fix IP for AP  
    delay(300);
    IPAddress getmyIP = WiFi.localIP();
    delay(300);

    //AP_time = millis();
    Serial.print("Accesspoint started - Name : ");
    Serial.println(steerSet.ssid_ap);
    Serial.print(" IP address: ");
    Serial.println(getmyIP);
    LED_WIFI_ON = true;
    digitalWrite(steerSet.LEDWiFi_PIN, steerSet.LEDWiFi_ON_Level);
    my_WiFi_Mode = WiFi.status();
}

#endif
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

