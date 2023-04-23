// definitions and variables for webinterface
char HTML_String[40000];

#define ACTION_LoadDefaultVal   1
#define ACTION_RESTART          2
#define ACTION_SET_WAS_ZERO     3
#define ACTION_SET_INCL_ZERO    4
#define ACTION_SET_WS_THRESHOLD 5

// Radiobutton output
char output_driver_tab[5][22] = { "None", "Cytron MD30 + SWM", "IBT_2 +SWM", "IBT_2 +PWM Valve", "IBT_2 +Danfoss Valve" };

// Radiobutton analog input
char was_input_tab[3][25] = { "direct to ESP PIN", "ADS 1115 single", "ADS 1115 differential" };

// Radiobutton IMU Heading Unit
char imu_type_tab[3][10] = { "None", "BNO 055","CMPS14" };

// Radiobutton Steerswitch
char steersw_type_tab[6][22] = { "Switch High", "Switch Low", "Toggle Button", "Analog Buttons","none (AOG controlled)","" };

// Radiobutton Workswitch
char worksw_type_tab[4][8] = { "None", "Digital", "Analog", "" };

// Radiobutton WorkSwitch Invert
char worksw_invert_tab[2][15] = { "not inverted", "inverted" };

// Radiobutton Encoder
char encoder_type_tab[2][11] = { "None", "Installed" };




//-------------------------------------------------------------------------------------------------
//7. Maerz 2021
// Mrz 23: main loop delay

void doWebinterface(void* pvParameters) {
	for (;;) {
		WiFi_Server.handleClient(); //does the Webinterface
		if (WebIOLastUsePlus3 < millis()) {//not called in the last 3 sec
			//Serial.println("Webinterface no client for 3 sec");
			bitClear(mainLoopDelay, 2);
			vTaskDelay(1000);
		}
		else {
			bitSet(mainLoopDelay, 2);//delay main loop for 4 ms to have time for WebIO
			vTaskDelay(20);
		}
		if ((now > WebIOTimeOut) && (Set.timeoutWebIO != 255)) {
			WebIORunning = false;
			WiFi_Server.close();
			Serial.println("closing Webinterface task");
			delay(1);
			vTaskDelete(NULL);
			delay(1);
		}
	}
}

//-------------------------------------------------------------------------------------------------
//7. Maerz 2021
// Mrz 2023 WebbIOlastUse

void handleRoot() {
	make_HTML01();
	WiFi_Server.sendHeader("Connection", "close");
	WiFi_Server.send(200, "text/html", HTML_String);
	WebIOLastUsePlus3 = 3000 + millis();
	WebIOTimeOut = WebIOLastUsePlus3 + long((Set.timeoutWebIO * 60000));
	if (Set.debugmode) {
		Serial.print("used size of HTML string: "); Serial.println(strlen(HTML_String));
		Serial.println("Webpage root");
		Serial.print("Timeout WebIO: "); Serial.println(WebIOTimeOut);
	}
	process_Request();
}

//-------------------------------------------------------------------------------------------------
//10. Mai 2020
// Mrz 2023 OTA changed

void WiFiStartServer() {

	WiFi_Server.on("/", HTTP_GET, []() {handleRoot(); });
	//file selection for firmware update
	WiFi_Server.on("/serverIndex", HTTP_GET, []() {
		WiFi_Server.sendHeader("Connection", "close");
		WiFi_Server.send(200, "text/html", serverIndex);
		});
	//handling uploading firmware file 
	WiFi_Server.on("/update", HTTP_POST, [&]() {
		WiFi_Server.sendHeader("Connection", "close");
		WiFi_Server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
		}, [&]() {
			HTTPUpload& upload = WiFi_Server.upload();
			if (upload.status == UPLOAD_FILE_START) {
				Serial.printf("Update: %s\n", upload.filename.c_str());
				if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_WRITE) {
				/* flashing firmware to ESP*/
				if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_END) {
				if (Update.end(true)) { //true to set the size to the current progress
					Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
				}
				else {
					Update.printError(Serial);
				}
			}
		});
	WiFi_Server.onNotFound(handleNotFound);

	WiFi_Server.begin();
}
//---------------------------------------------------------------------
// Process given values 10. Mai 2020
//---------------------------------------------------------------------
void process_Request()
{
	int temInt = 0;
	long temLong = 0;
	double temDoub = 0.0;

	if (Set.debugmode) { Serial.print("From webinterface: number of arguments: "); Serial.println(WiFi_Server.args()); }
	for (byte n = 0; n < WiFi_Server.args(); n++) {
		if (Set.debugmode) {
			Serial.print("argName "); Serial.print(WiFi_Server.argName(n));
			Serial.print(" val: "); Serial.println(WiFi_Server.arg(n));
		}
		if (WiFi_Server.argName(n) == "ACTION") {
			temInt = int(WiFi_Server.arg(n).toInt());
			if (Set.debugmode) { Serial.print("Action found: "); Serial.println(temInt); }
		}
		if (temInt != ACTION_RESTART) { EEprom_unblock_restart(); }
		if (temInt == ACTION_LoadDefaultVal) {
			if (Set.debugmode) { Serial.println("load default settings from EEPROM"); }
			EEprom_read_default();
			delay(2);
		}
		//save changes
		if (WiFi_Server.argName(n) == "Save") {
			if (Set.debugmode) { Serial.println("Save button pressed in webinterface"); }
			EEprom_write_all();
		}

		if (WiFi_Server.argName(n) == "SSID_MY1") {
			for (int i = 0; i < 24; i++) Set.ssid1[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.ssid1, temInt);
		}
		if (WiFi_Server.argName(n) == "Password_MY1") {
			for (int i = 0; i < 24; i++) Set.password1[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.password1, temInt);
		}
		if (WiFi_Server.argName(n) == "SSID_MY2") {
			for (int i = 0; i < 24; i++) Set.ssid2[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.ssid2, temInt);
		}
		if (WiFi_Server.argName(n) == "Password_MY2") {
			for (int i = 0; i < 24; i++) Set.password2[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.password2, temInt);
		}
		if (WiFi_Server.argName(n) == "SSID_MY3") {
			for (int i = 0; i < 24; i++) Set.ssid3[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.ssid3, temInt);
		}
		if (WiFi_Server.argName(n) == "Password_MY3") {
			for (int i = 0; i < 24; i++) Set.password3[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.password3, temInt);
		}
		if (WiFi_Server.argName(n) == "SSID_MY4") {
			for (int i = 0; i < 24; i++) Set.ssid4[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.ssid4, temInt);
		}
		if (WiFi_Server.argName(n) == "Password_MY4") {
			for (int i = 0; i < 24; i++) Set.password4[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.password4, temInt);
		}
		if (WiFi_Server.argName(n) == "SSID_MY5") {
			for (int i = 0; i < 24; i++) Set.ssid5[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.ssid5, temInt);
		}
		if (WiFi_Server.argName(n) == "Password_MY5") {
			for (int i = 0; i < 24; i++) Set.password5[i] = 0x00;
			temInt = WiFi_Server.arg(n).length() + 1;
			WiFi_Server.arg(n).toCharArray(Set.password5, temInt);
		}
		if (WiFi_Server.argName(n) == "timeoutRout") {
			argVal = WiFi_Server.arg(n).toInt();
			if ((argVal >= 20) && (argVal <= 1000)) { Set.timeoutRouterWiFi = int(argVal); }
		}
		if (WiFi_Server.argName(n) == "timeoutWebIO") {
			temLong = WiFi_Server.arg(n).toInt();
			if ((temLong >= 2) && (temLong <= 255)) { Set.timeoutWebIO = byte(temLong); }
		}
		if (WiFi_Server.argName(n) == "AgIOHeartBeat") {
			if (WiFi_Server.arg(n) == "true") { Set.AgIOHeartbeat_answer = 1; }
			else { Set.AgIOHeartbeat_answer = 0; }
		}
		if (WiFi_Server.argName(n) == "DataTransfVia") {
			temInt = WiFi_Server.arg(n).toInt();
			if ((temInt <= 20) && (temInt >= 0)) { Set.DataTransVia = byte(temInt); }
			if (Set.DataTransVia == 10) {
				if (Eth_connect_step == 255) {
					Eth_connect_step = 10;
					xTaskCreate(Eth_handle_connection, "Core1EthConnectHandle", 3072, NULL, 1, &taskHandle_Eth_connect);
					delay(500);
				}
			}
			if (Set.DataTransVia < 5) {//USB
				if (!USBDataTaskRunning) {
					xTaskCreate(getDataFromAOGUSB, "DataFromAOGHandleUSB", 5000, NULL, 1, &taskHandle_DataFromAOGUSB);
					delay(500);
				}
				if (EthDataTaskRunning) { vTaskDelete(taskHandle_DataFromAOGEth); delay(5); EthDataTaskRunning = false; }
			}
			else {
				if (Set.DataTransVia < 10) {//WiFi UDP
					if (USBDataTaskRunning) { vTaskDelete(taskHandle_DataFromAOGUSB); delay(5); USBDataTaskRunning = false; }
					if (EthDataTaskRunning) { vTaskDelete(taskHandle_DataFromAOGEth); delay(5); EthDataTaskRunning = false; }
				}
				else {
					if (Set.DataTransVia == 10) {//Ethernet UDP
						if (!EthDataTaskRunning) {
							xTaskCreate(getDataFromAOGEth, "DataFromAOGHandleEth", 5000, NULL, 1, &taskHandle_DataFromAOGEth);
							delay(500);
						}
						if (USBDataTaskRunning) { vTaskDelete(taskHandle_DataFromAOGUSB); delay(5); USBDataTaskRunning = false; }
					}
				}
			}
		}
		if (WiFi_Server.argName(n) == "OUTPUT_TYPE") { Set.output_type = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "invMotor") {
			if (WiFi_Server.arg(n) == "true") { Set.MotorDriveDirection = 1; }
			else { Set.MotorDriveDirection = 0; }
		}
		if (WiFi_Server.argName(n) == "PWMFreq") {
			argVal = int(WiFi_Server.arg(n).toInt());
			if ((argVal <= 20000) && (argVal >= 20)) { Set.PWMOutFrequ = int(argVal); }
		}
		if (WiFi_Server.argName(n) == "MotSlow") {
			argVal = WiFi_Server.arg(n).toInt();
			if ((argVal <= 20) && (argVal >= 1)) { Set.MotorSlowDriveDegrees = byte(argVal); }
		}

		if (WiFi_Server.argName(n) == "INPUT_TYPE") { Set.WASType = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "AckermFix") { Set.AckermanFix = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "invWAS") {
			if (WiFi_Server.arg(n) == "true") { Set.InvertWAS = 1; }
			else { Set.InvertWAS = 0; }
		}
		if (temInt == ACTION_SET_WAS_ZERO) {
			Set.WebIOSteerPosZero = actualSteerPosRAW; // >zero< Funktion Set Steer Angle to 0
			EEprom_write_all();
		}
		if (WiFi_Server.argName(n) == "IMU_TYPE") {
			Set.IMUType = byte(WiFi_Server.arg(n).toInt());
			assignGPIOs_start_extHardware();
		}
		if (WiFi_Server.argName(n) == "invRoll") {
			if (WiFi_Server.arg(n) == "true") { Set.InvertRoll = 1; }
			else { Set.InvertRoll = 0; }
		}
		if (temInt == ACTION_SET_INCL_ZERO) {
			int roll_avg = 0;
			for (int i = 0; i < 16; i++) {
				roll_avg += x_;
				delay(200);
			}
			if (Set.InvertRoll == 1) { roll_avg = 0 - roll_avg; }
			Set.roll_corr = roll_avg >> 4;
			EEprom_write_all();
		}
		if (WiFi_Server.argName(n) == "ENC_TYPE") { Set.ShaftEncoder = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "ENC_COUNTS") { Set.pulseCountMax = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "SSWITCH_TYPE") { 
			Set.SteerSwitchType = byte(WiFi_Server.arg(n).toInt());
			if (Set.SteerSwitchType == 4) {Set.SteerSwitchType = 255;}//none
		}
	//	if (WiFi_Server.argName(n) == "RSWITCH_TYPE") { Set.SteerRemoteSwitch = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "WSWITCH_TYPE") { Set.WorkSW_mode = byte(WiFi_Server.arg(n).toInt()); }
		if (WiFi_Server.argName(n) == "invWoSw") {
			if (WiFi_Server.arg(n) == "true") { Set.Invert_WorkSW = 1; }
			else { Set.Invert_WorkSW = 0; }
		}
		if (temInt == ACTION_SET_WS_THRESHOLD) {
			unsigned int WSThres_avg = 0;
			for (int i = 0; i < 8; i++) {
				WSThres_avg += analogRead(Set.WORKSW_PIN);
				delay(100);
			}
			Set.WorkSW_Threshold = WSThres_avg >> 3;
			EEprom_write_all();
		}
		if (WiFi_Server.argName(n) == "MinSpeed") {
			temDoub = WiFi_Server.arg(n).toDouble();
			if ((temDoub < 3) && (temDoub >= 0)) { Set.autoSteerMinSpeed = float(temDoub); }
		}
		if (WiFi_Server.argName(n) == "MaxSpeed") {
			temDoub = WiFi_Server.arg(n).toDouble();
			if ((temDoub <= 30) && (temDoub >= 5)) { Set.autoSteerMaxSpeed = float(temDoub); }
		}
		if (WiFi_Server.argName(n) == "WiFiIP0") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.WiFi_myip[0] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "WiFiIP1") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.WiFi_myip[1] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "WiFiIP2") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.WiFi_myip[2] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "WiFiIP3") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.WiFi_myip[3] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "WiFiIPDest") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.WiFi_ipDest_ending = byte(temInt);
		}

		if (WiFi_Server.argName(n) == "EthIP0") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_myip[0] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthIP1") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_myip[1] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthIP2") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_myip[2] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthIP3") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_myip[3] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthStatIP") {
			temInt = WiFi_Server.arg(n).toInt();
			if (temInt == 1) { Set.Eth_static_IP = true; }
			else { Set.Eth_static_IP = false; }
		}
		if (WiFi_Server.argName(n) == "EthIPDest") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_ipDest_ending = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac0") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[0] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac1") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[1] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac2") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[2] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac3") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[3] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac4") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[4] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "EthMac5") {
			temInt = WiFi_Server.arg(n).toInt();
			Set.Eth_mac[5] = byte(temInt);
		}
		if (WiFi_Server.argName(n) == "debugmode") {
			if (WiFi_Server.arg(n) == "true") { Set.debugmode = true; }
			else { Set.debugmode = false; }
		}
		if (WiFi_Server.argName(n) == "debugmodeDatFromAOG") {
			if (WiFi_Server.arg(n) == "true") { Set.debugmodeDataFromAOG = true; }
			else { Set.debugmodeDataFromAOG = false; }
		}
		

		if (temInt == ACTION_RESTART) {
			Serial.println("reboot ESP32: selected by webinterface");
			EEprom_block_restart();//prevents from restarting, when webpage is reloaded. Is set to 0, when other ACTION than restart is called
			delay(1000);
#if HardwarePlatform == 0
			WiFi.disconnect();
			delay(500);
			ESP.restart();
#endif
#if HardwarePlatform == 1
			WiFi.end();
			delay(2000);
			Serial.println("restarting WiFi");
			WiFi_Start_STA();
			delay(200);
			if (my_WiFi_Mode == 0) {// if failed start AP
				WiFi_Start_AP();
				delay(100);
			}
			delay(200);
#endif
		}
	}
}

//---------------------------------------------------------------------
// HTML Seite 01 aufbauen
//---------------------------------------------------------------------
void make_HTML01() {

	strcpy(HTML_String, "<!DOCTYPE html>");
	strcat(HTML_String, "<html>");

	strcat(HTML_String, "<head>");
	strcat(HTML_String, "<title>AG Autosteer ESP config</title>");
	strcat(HTML_String, "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0;\" />\r\n");
	//strcat( HTML_String, "<meta http-equiv=\"refresh\" content=\"10\">");
	strcat(HTML_String, "<style>divbox {background-color: lightgrey;width: 200px;border: 5px solid red;padding:10px;margin: 10px;}</style>");
	strcat(HTML_String, "</head>");
	strcat(HTML_String, "<body bgcolor=\"#ffcf00\">");//66b3ff
	strcat(HTML_String, "<font color=\"#000000\" face=\"VERDANA,ARIAL,HELVETICA\">");
	strcat(HTML_String, "<h1>AG Autosteer ESP config page</h1>");
	strcat(HTML_String, "Version ");
	strcati(HTML_String, vers_nr);
	strcat(HTML_String, VersionTXT);
	strcat(HTML_String, "<br><hr>");

	//---------------------------------------------------------------------------------------------  
	//load values of INO setup zone
	strcat(HTML_String, "<h2>Load default values of INO setup zone</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(270, 250, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"2\">Only loads default values, does NOT save them</td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_LoadDefaultVal);
	strcat(HTML_String, "')\" style= \"width:150px\" value=\"Load default values\"></button></td>");
	strcat(HTML_String, "</tr>");
	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// WiFi Client Access Data

	strcat(HTML_String, "<h2>WiFi Network Client Access Data</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "</b>If access to networks fails, an accesspoint will be created:<br>SSID: <b>");
	strcat(HTML_String, Set.ssid_ap);
	strcat(HTML_String, "</b>     with no password<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr><td><b>#1 Network SSID:</b></td>");
	strcat(HTML_String, "<td><input type=\"text\" onchange=\"sendVal('/?SSID_MY1='+this.value)\" style= \"width:200px\" name=\"SSID_MY1\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.ssid1);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td></tr>");

	strcat(HTML_String, "<tr><td><b>#1 Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY1='+this.value)\" style= \"width:200px\" name=\"Password_MY1\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.password1);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr><td><b>#2 Network SSID:</b></td>");
	strcat(HTML_String, "<td><input type=\"text\" onchange=\"sendVal('/?SSID_MY2='+this.value)\" style= \"width:200px\" name=\"SSID_MY2\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.ssid2);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<tr><td><b>#2 Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY2='+this.value)\" style= \"width:200px\" name=\"Password_MY2\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.password2);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr><td><b>#3 Network SSID:</b></td>");
	strcat(HTML_String, "<td><input type=\"text\" onchange=\"sendVal('/?SSID_MY3='+this.value)\" style= \"width:200px\" name=\"SSID_MY3\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.ssid3);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<tr><td><b>#3 Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY3='+this.value)\" style= \"width:200px\" name=\"Password_MY3\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.password3);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr><td><b>#4 Network SSID:</b></td>");
	strcat(HTML_String, "<td><input type=\"text\" onchange=\"sendVal('/?SSID_MY4='+this.value)\" style= \"width:200px\" name=\"SSID_MY4\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.ssid4);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<tr><td><b>#4 Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY4='+this.value)\" style= \"width:200px\" name=\"Password_MY4\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.password4);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr><td><b>#5 Network SSID:</b></td>");
	strcat(HTML_String, "<td><input type=\"text\" onchange=\"sendVal('/?SSID_MY5='+this.value)\" style= \"width:200px\" name=\"SSID_MY5\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.ssid5);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<tr><td><b>#5 Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY5='+this.value)\" style= \"width:200px\" name=\"Password_MY5\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, Set.password5);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">time, trying to connect to network</td></tr>");
	strcat(HTML_String, "<td colspan=\"3\">after time has passed access point is opened</td></tr>");
	strcat(HTML_String, "<tr><td><b>Timeout (s):</b></td><td><input type = \"number\" onchange=\"sendVal('/?timeoutRout='+this.value)\" name = \"timeoutRout\" min = \"20\" max = \"1000\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.timeoutRouterWiFi);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr><td colspan=\"2\"><b>Reboot ESP32 for changes to take effect</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_RESTART);
	strcat(HTML_String, "')\" style= \"width:120px\" value=\"Restart\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

//-----------------------------------------------------------------------------------------
// timeout webinterface

	strcat(HTML_String, "<h2>Webinterface timeout</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<b>Webinterface needs lots of calculation time, so if switched off, system runs better.</b><br>");
	strcat(HTML_String, "After this time (minutes) from restart, or last usage, webinterface is turned off.<br><br>");
	strcat(HTML_String, "Set to 255 to keep active.<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr><td><b>Webinterface timeout (min)</b></td><td><input type = \"number\"  onchange=\"sendVal('/?timeoutWebIO='+this.value)\" name = \"timeoutWebIO\" min = \"2\" max = \"255\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.timeoutWebIO);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");


	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

/*
	//-----------------------------------------------------------------------------------------
    // AOG Version

	strcat(HTML_String, "<h2>AOG Version number</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<b>For AgOpenGPS version 4.3 set 17, for V 4.6 and above set 20</b><br>");
	strcat(HTML_String, "AOG 4.3.10 = 4 + 3 + 10 = 17<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	//strcat(HTML_String, "<td colspan=\"3\">for 4.1 and before set 0</td></tr>");
	strcat(HTML_String, "<tr><td><b>AOG Version code</b></td><td><input type = \"number\"  onchange=\"sendVal('/?aogVer='+this.value)\" name = \"aogVer\" min = \"0\" max = \"255\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.aogVersion);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");


	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");
*/
	//---------------------------------------------------------------------------------------------  
   // Data transfer via USB/Wifi 
	strcat(HTML_String, "<h2>USB, WiFi or Ethernet data transfer</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 400, 150, 0, 0);

	//transfer data via 0 = USB / 7 = WiFi UDP / 8 = WiFi UDP 2x / 10 = Ethernet
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=0')\" name=\"DataTransfVia\" id=\"JZ\" value=\"0\"");
	if (Set.DataTransVia == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=7')\" name=\"DataTransfVia\" id=\"JZ\" value=\"7\"");
	if (Set.DataTransVia == 7)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">WiFi (UDP) (default)</label></td></tr>");

/*	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=8')\" name=\"DataTransfVia\" id=\"JZ\" value=\"8\"");
	if (Set.DataTransVia == 8)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">WiFi (UDP) send data 2x)</label></td></tr>");
*/
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td colspan=\"2\"><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=10')\" name=\"DataTransfVia\" id=\"JZ\" value=\"10\"");
	if (Set.DataTransVia == 10)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">Ethernet (UDP) Ethernet hardware needed!!</label></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//---------------------------------------------------------------------------------------------
	// Send AgIO heartbeat answer

	strcat(HTML_String, "<h2>AgIO heartbeat</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 400, 150, 0, 0);
	strcat(HTML_String, "<tr> <td colspan=\"3\">Send autosteer heartbeat to AgIO. Not recommended when using WiFi, the IP is always send.</td> </tr>");
	strcat(HTML_String, "<tr><td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?AgIOHeartBeat='+this.checked)\" name=\"AgIOHeartBeat\" id = \"Part\" value = \"1\" ");
	if (Set.AgIOHeartbeat_answer == 1) { strcat(HTML_String, "checked "); }
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> send autosteer heartbeat</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//---------------------------------------------------------------------------------------------
	//strcat(HTML_String, "<h1>Hardware setup</h1><hr>");

	//-----------------------------------------------------------------------------------------
	 // Steerswitch Type
	strcat(HTML_String, "<h2>Switch Types</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 4; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Steerswitch type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?SSWITCH_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\"name=\"SSWITCH_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.SteerSwitchType == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, steersw_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}
	strcat(HTML_String, "<tr><td> </td>");
	strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?SSWITCH_TYPE=255')\"name=\"SSWITCH_TYPE\" id=\"JZ255");
	strcat(HTML_String, "\" value=\"255\"");
	if (Set.SteerSwitchType == 255)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ255\">");
	strcat(HTML_String, steersw_type_tab[4]);
	strcat(HTML_String, "</label></td>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	for (int i = 0; i < 3; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Workswitch type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?WSWITCH_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"WSWITCH_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.WorkSW_mode == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, worksw_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0)  strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	}
	strcat(HTML_String, "</tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	//checkbox invert Workswitch
	strcat(HTML_String, "<tr><td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?invWoSw='+this.checked)\" name=\"invWoSw\" id = \"Part\" value = \"1\" ");
	if (Set.Invert_WorkSW == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> <b> Invert Workswitch</b></label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	//display Worksw value
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>Analog Workswitch Threshold value</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"><b>");
	strcati(HTML_String, (Set.WorkSW_Threshold));
	strcat(HTML_String, "</b></font></divbox>0-4095</td>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Set Threshold</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_SET_WS_THRESHOLD);
	strcat(HTML_String, "')\" style= \"width:200px\" value=\"Use Current\"></button></td>");
	strcat(HTML_String, "<td>Set Threshold value to current position</td>");


	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// Steering Wheel Encoder
	strcat(HTML_String, "<h2>Steering Wheel Encoder</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 2; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Encoder:</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?ENC_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"ENC_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.ShaftEncoder == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, encoder_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Counts to turn off Autosteer</b></td>");
	strcat(HTML_String, "<td><input type = \"number\" onchange=\"sendVal('/?ENC_COUNTS='+this.value)\" name = \"ENC_COUNTS\" min = \"0\" max = \"100\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.pulseCountMax);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//---------------------------------------------------------------------------------------------  
	// Output Driver
	strcat(HTML_String, "<h2>Output Driver</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 5; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Select your output type</b></td>");
		else if (i == 1) strcat(HTML_String, "<td>SWM: Steer Wheel Motor</td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?OUTPUT_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"OUTPUT_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.output_type == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, output_driver_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	//checkbox invert Motor
	strcat(HTML_String, "<tr><td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?invMotor='+this.checked)\" name=\"invMotor\" id = \"Part\" value = \"1\" ");
	if (Set.MotorDriveDirection == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> <b> Invert Motor direction</b></label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "</tr>");

	//PWM frequency + Motor slow drive range
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>PWM output frequency</b></td>");
	strcat(HTML_String, "<td><input type = \"number\" onchange=\"sendVal('/?PWMFreq='+this.value)\" name = \"PWMFreq\" min = \"20\" max = \"20000\" step = \"10\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.PWMOutFrequ);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");
	strcat(HTML_String, "</tr>");
	strcat(HTML_String, "<td colspan=\"3\">1000 Hz for low heat at PWM device</td></tr>");
	strcat(HTML_String, "<td colspan=\"3\">20000 Hz not hearable</td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\"><b>Motor/Valve is driving slower if steer angle error is less then x degrees:</b></td></tr>");
	strcat(HTML_String, "<td>slow drive range (degr):</td>");
	strcat(HTML_String, "<td><input type = \"number\" onchange=\"sendVal('/?MotSlow='+this.value)\" name = \"MotSlow\" min = \"1\" max = \"10\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.MotorSlowDriveDegrees);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// WAS  Input device
	strcat(HTML_String, "<h2>Wheel Angle Sensor (WAS)</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 3; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Select your Input type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?INPUT_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"INPUT_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.WASType == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, was_input_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>WAS RAW Data</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"> <b>");
	strcati(HTML_String, actualSteerPosRAW);
	strcat(HTML_String, "</b></font></divbox></td></tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>WAS corrected Data</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"> <b>");
	strcati(HTML_String, steeringPosition);
	strcat(HTML_String, "</b></font></divbox></td>");

	

	//Refresh button
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"location.reload()\" style= \"width:120px\" value=\"Refresh\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Center your Sensor to Zero</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_SET_WAS_ZERO);
	strcat(HTML_String, "')\" style= \"width:200px\" value=\"ZERO NOW\"></button></td>");
	strcat(HTML_String, "<td>Your Wheels should face straight ahead</td>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Ackerman fix factor (%)</b></td>");
	strcat(HTML_String, "<td><input type = \"number\" onchange=\"sendVal('/?AckermFix='+this.value)\" name = \"AckermFix\" min = \"35\" max = \"300\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, Set.AckermanFix);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr><td colspan=\"3\">if values for right and left side are the same: factor = 100 </td></tr>");
	strcat(HTML_String, "<tr><td colspan=\"3\">if values to the right are higher than to the left: 35 < factor < 100</td></tr>");
	strcat(HTML_String, "<tr><td colspan=\"3\">if values to the left are higher than to the right: 100 < factor < 300</td></tr>");

	// Checkbox invert WAS
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td colspan=\"2\"><input type=\"checkbox\" onclick=\"sendVal('/?invWAS='+this.checked)\" name=\"invWAS\" id = \"Part\" value = \"1\" ");
	if (Set.InvertWAS == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"><b> Invert wheel angle sensor</b></label>");
	strcat(HTML_String, "</td>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "<br><b>Steering to the left must be minus</b>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	//speed for autosteer

	strcat(HTML_String, "<h2>Speed range for using autosteer</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);
	//min speed
	strcat(HTML_String, "<tr><td><b>min speed (km/h)</b></td><td><input type = \"number\" onchange=\"sendVal('/?MinSpeed='+this.value)\" name = \"MinSpeed\" min = \"0\" max = \"2\" step = \"0.1\" style= \"width:200px\" value = \"");// placeholder = \"");
	if (Set.autoSteerMinSpeed < 10) { strcatf(HTML_String, Set.autoSteerMinSpeed, 3, 1); }
	else {strcatf(HTML_String, Set.autoSteerMinSpeed, 4, 1); }
	strcat(HTML_String, "\"></td>");
	//max speed
	strcat(HTML_String, "<tr><td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr><td><b>max speed (km/h)</b></td><td><input type = \"number\" onchange=\"sendVal('/?MaxSpeed='+this.value)\" name = \"MaxSpeed\" min = \"10\" max = \"30\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	if (Set.autoSteerMaxSpeed < 10) { strcatf(HTML_String, Set.autoSteerMaxSpeed, 3, 1); }
	else { strcatf(HTML_String, Set.autoSteerMaxSpeed, 4, 1); }
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// IMU Heading Unit

	strcat(HTML_String, "<h2>IMU Heading Unit (Compass)</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 3; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Select your IMU type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?IMU_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"IMU_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (Set.IMUType == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, imu_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\"  style= \"width:120px\" value=\"Save\"></button></td>");
		}
		strcat(HTML_String, "</tr>");
	}
	strcat(HTML_String, "<tr><td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>Heading");
	if (Set.IMUType == 2) { strcat(HTML_String, " from CPMS14"); }
	if (Set.IMUType == 3) { strcat(HTML_String, " from BNO085"); }
	strcat(HTML_String, "</td><td><divbox align=\"right\"><font size=\"+1\"><b>");
	if (heading < 10) { strcatf(HTML_String, heading, 3, 1); }
	else {
		if (heading < 100) { strcatf(HTML_String, heading, 4, 1); }
		else { strcatf(HTML_String, heading, 5, 1); }
	}
	strcat(HTML_String, "</b></font></divbox>degree</td>");

	//Refresh button
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"location.reload()\" style= \"width:120px\" value=\"Refresh\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr><td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr>");
	if (Set.IMUType > 1) {//CMPS BNO085
		if (Set.IMUType == 2) { strcat(HTML_String, "<td><br>Roll from CMPS14</td>"); }
		if (Set.IMUType == 3) { strcat(HTML_String, "<td><br>Roll from BNO085</td>"); }
		strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"><b>");
		if (roll < 10) { strcatf(HTML_String, roll, 3, 1); }
		else {
			if (roll < 100) { strcatf(HTML_String, roll, 4, 1); }
			else { strcatf(HTML_String, roll, 5, 1); }
		}
		strcat(HTML_String, "</b></font></divbox>degree</td>");
	}

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//---------------------------------------------------------------------------------------------

	//strcat(HTML_String, "<h1>Network setup</h1><hr>");

	//---------------------------------------------------------------------------------------------  
	// WiFi IP settings 
	strcat(HTML_String, "<h2>WiFi IP settings</h2>");
	strcat(HTML_String, "<form>");
	//IP
	strcat(HTML_String, "<b>IP address for WiFi</b><br>When using DHCP the last number is set as IP from here, the first 3 numbers are set by DHCP.<br>");
	strcat(HTML_String, "<b>Default for last number is 77, it's also the address of the Webinterface.</b>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 50, 50, 50, 50);
	strcat(HTML_String, "<tr><td>IP address</td><td><input type = \"number\"  onchange=\"sendVal('/?WiFiIP0='+this.value)\" name = \"WiFiIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.WiFi_myip[0]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?WiFiIP1='+this.value)\" name = \"WiFiIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.WiFi_myip[1]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?WiFiIP2='+this.value)\" name = \"WiFiIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.WiFi_myip[2]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?WiFiIP3='+this.value)\" name = \"WiFiIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.WiFi_myip[3]);
	strcat(HTML_String, "\"></td></table><br>");

	//IP destination    
	strcat(HTML_String, "<table>");
	set_colgroup(250, 300, 150, 0, 0);
	strcat(HTML_String, "<tr><td colspan=\"2\"><b>IP address of destination</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr></table><table>");
	strcat(HTML_String, "Destination's IP address, the first 3 numbers are set by DHCP, or as above.<br>");
	strcat(HTML_String, "<b>Use 255 to send to every device in network (default).</b> Use IP of your Computer, if you don't have a router and fixed IPs");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 50, 50, 50, 50);
	strcat(HTML_String, "<tr><td>IP address destination</td><td>xxx</td><td>xxx</td><td>xxx<td><input type = \"number\"  onchange=\"sendVal('/?WiFiIPDest='+this.value)\" name = \"WiFiIPDest\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.WiFi_ipDest_ending);
	strcat(HTML_String, "\"></td></table><br><hr>");

	//---------------------------------------------------------------------------------------------  
	// Ethernet settings 
	strcat(HTML_String, "<h2>Ethernet settings</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 400, 150, 0, 0);

	//use DHCP/static IP radio button
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?EthStatIP=0')\" name=\"EthStatIP\" id=\"JZ\" value=\"0\"");
	if (Set.Eth_static_IP == false)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">use DHCP (default)</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?EthStatIP=1')\" name=\"EthStatIP\" id=\"JZ\" value=\"1\"");
	if (Set.Eth_static_IP == true)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">use fixed IP from below</label></td></tr>");

	strcat(HTML_String, "</table><br>");

	//IP
	strcat(HTML_String, "<b>IP address for Ethernet</b><br>When using DHCP the last number is set as IP from here, the first 3 numbers are set by DHCP.<br>");
	strcat(HTML_String, "<b>Default for last number is 78.</b>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 50, 50, 50, 50);
	strcat(HTML_String, "<tr><td>IP address</td><td><input type = \"number\"  onchange=\"sendVal('/?EthIP0='+this.value)\" name = \"EthIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_myip[0]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthIP1='+this.value)\" name = \"EthIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_myip[1]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthIP2='+this.value)\" name = \"EthIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_myip[2]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthIP3='+this.value)\" name = \"EthIP\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_myip[3]);
	strcat(HTML_String, "\"></td></table><br>");

	//IP destination
	strcat(HTML_String, "<table>");
	set_colgroup(250, 300, 150, 0, 0);
	strcat(HTML_String, "<tr><td colspan=\"2\"><b>IP address of destination</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr></table><table>");
	strcat(HTML_String, "Destination's IP address, the first 3 numbers are set by DHCP, or as above.<br>");
	strcat(HTML_String, "<b>Use 255 to send to every device in network (default).</b> Use IP of your Computer, if you don't have a router and fixed IPs");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 50, 50, 50, 50);
	strcat(HTML_String, "<tr><td>IP address destination</td><td>xxx</td><td>xxx</td><td>xxx<td><input type = \"number\"  onchange=\"sendVal('/?EthIPDest='+this.value)\" name = \"EthIPDest\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_ipDest_ending);
	strcat(HTML_String, "\"></td></table><br>");

	//mac
	strcat(HTML_String, "<b>mac address of Ethernet hardware</b><br>Type in the mac address of you Ethernet shield.<br>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 50, 50, 50, 50, 50, 50);
	strcat(HTML_String, "<tr><td>mac address</td><td><input type = \"number\"  onchange=\"sendVal('/?EthMac0='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[0]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthMac1='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[1]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthMac2='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[2]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthMac3='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[3]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthMac4='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[4]);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><input type = \"number\"  onchange=\"sendVal('/?EthMac5='+this.value)\" name = \"EthMac\" min = \"0\" max = \"255\" step = \"1\" style= \"width:40px\" value = \"");
	strcati(HTML_String, Set.Eth_mac[5]);
	strcat(HTML_String, "\"></td></table>");


	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");


	//-------------------------------------------------------------
	// Checkbox debugmode
	strcat(HTML_String, "<h2>Debug</h2><hr>");

	//debug values
	strcat(HTML_String, "Settingsdata from AOG: Ackermann: ");
	strcati(HTML_String, Set.AckermanFix);
	strcat(HTML_String, " sensorCounts: ");
	strcati(HTML_String, Set.steerSensorCounts);
	//strcat(HTML_String, " WASoffset: ");
	//strcati(HTML_String, Set.wasOffset);
	strcat(HTML_String, "<br>minPWM: ");
	strcati(HTML_String, Set.minPWM);
	strcat(HTML_String, " lowPWM: ");
	strcati(HTML_String, Set.lowPWM);
	strcat(HTML_String, " high PWM: ");
	strcati(HTML_String, Set.highPWM);
	strcat(HTML_String, "<br><br>");

	strcat(HTML_String, "Steerdata from AOG: Guidance Status: ");
	strcati(HTML_String, guidanceStatus);
	strcat(HTML_String, " speed: ");
	strcati(HTML_String, gpsSpeed);
	strcat(HTML_String, "<br>SteerAngleSetPoint: ");
	strcati(HTML_String, steerAngleSetPoint);
	strcat(HTML_String, " SectGrFromAOG[0]: ");
	strcati(HTML_String, SectGrFromAOG[0]);
	strcat(HTML_String, " SectGrFromAOG[1]: ");
	strcati(HTML_String, SectGrFromAOG[1]);
	strcat(HTML_String, "<br><hr>");

	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(150, 400, 150, 0, 0);

	strcat(HTML_String, "<tr> <td colspan=\"3\"><b>Debugmode sends messages to USB serial</b></td> </tr>");
	strcat(HTML_String, "<tr><td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?debugmode='+this.checked)\" name=\"debugmode\" id = \"Part\" value = \"1\" ");
	if (Set.debugmode == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> debugmode on</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");

	strcat(HTML_String, "<tr><td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?debugmodeDatFromAOG='+this.checked)\" name=\"debugmodeDatFromAOG\" id = \"Part\" value = \"1\" ");
	if (Set.debugmodeDataFromAOG == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> debugmode Data From AOG on</label></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-------------------------------------------------------------
	// firmware update
	strcat(HTML_String, "<h2>Firmware Update for ESP32</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	strcat(HTML_String, "<tr> <td colspan=\"3\">build a new firmware with Arduino IDE selecting</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">Sketch -> Export compiled Binary</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\">upload this file via WiFi/Ethernet connection</td> </tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr><td></td>");
	//button
	strcat(HTML_String, "<td><input type='submit' onclick='openUpload(this.form)' value='Open Firmware uploader'></td></tr>");

	strcat(HTML_String, "<script>");
	strcat(HTML_String, "function openUpload(form)");
	strcat(HTML_String, "{");
	strcat(HTML_String, "window.open('/serverIndex')");
	strcat(HTML_String, "}");
	strcat(HTML_String, "</script>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-------------------------------------------------------------  
	strcat(HTML_String, "</font>");
	strcat(HTML_String, "</font>");
	strcat(HTML_String, "</body>");
	strcat(HTML_String, "</html>");


	//script to send values from webpage to ESP for process request
	strcat(HTML_String, "<script>");
	strcat(HTML_String, "function sendVal(ArgStr)");
	strcat(HTML_String, "{");
	strcat(HTML_String, "  var xhttp = new XMLHttpRequest();");
	strcat(HTML_String, "  xhttp.open(\"GET\",ArgStr, true);");
	strcat(HTML_String, "  xhttp.send();");
	strcat(HTML_String, " if ((ArgStr == '/?ACTION=");
	strcati(HTML_String, ACTION_LoadDefaultVal);	
	strcat(HTML_String, "') || (ArgStr == '/?ACTION=");	
	strcati(HTML_String, ACTION_SET_WAS_ZERO);
	strcat(HTML_String, "')) { window.setTimeout('location.reload()',300); }");
	strcat(HTML_String, " if ((ArgStr == '/?ACTION=");
	strcati(HTML_String, ACTION_SET_INCL_ZERO);
	strcat(HTML_String, "') || (ArgStr == '/?ACTION=");	
	strcati(HTML_String, ACTION_SET_WS_THRESHOLD);
	strcat(HTML_String, "')) { window.setTimeout('location.reload()',1800); }");
	strcat(HTML_String, "}");
	strcat(HTML_String, "</script>");

}


//-------------------------------------------------------------------------------------------------

void handleNotFound() {
	const char* notFound =
		"<!doctype html>"
		"<html lang = \"en\">"
		"<head>"""
		"<meta charset = \"utf - 8\">"
		"<meta http - equiv = \"x - ua - compatible\" content = \"ie = edge\">"
		"<meta name = \"viewport\" content = \"width = device - width, initial - scale = 1.0\">"
		"<title>Redirecting</title>"
		"</head>"
		"<body onload = \"redirect()\">"
		"<h1 style = \"text - align: center; padding - top: 50px; display: block; \"><br>404 not found<br><br>Redirecting to settings page in 3 secs ...</h1>"
		"<script>"
		"function redirect() {"
		"setTimeout(function() {"
		"    window.location.replace(\"/root\");"//new landing page
		"}"
		", 3000);"
		"}"
		"</script>"
		"</body>"
		"</html>";

	WiFi_Server.sendHeader("Connection", "close");
	WiFi_Server.send(200, "text/html", notFound);
	if (Set.debugmode) { Serial.println("redirecting from 404 not found to Webpage root"); }
}

//-------------------------------------------------------------------------------------------------

void set_colgroup(int w1, int w2, int w3, int w4, int w5) {
	strcat(HTML_String, "<colgroup>");
	set_colgroup1(w1);
	set_colgroup1(w2);
	set_colgroup1(w3);
	set_colgroup1(w4);
	set_colgroup1(w5);
	strcat(HTML_String, "</colgroup>");
}
void set_colgroup(int w1, int w2, int w3, int w4, int w5, int w6) {
	strcat(HTML_String, "<colgroup>");
	set_colgroup1(w1);
	set_colgroup1(w2);
	set_colgroup1(w3);
	set_colgroup1(w4);
	set_colgroup1(w5);
	set_colgroup1(w6);
	strcat(HTML_String, "</colgroup>");
}
void set_colgroup(int w1, int w2, int w3, int w4, int w5, int w6, int w7) {
	strcat(HTML_String, "<colgroup>");
	set_colgroup1(w1);
	set_colgroup1(w2);
	set_colgroup1(w3);
	set_colgroup1(w4);
	set_colgroup1(w5);
	set_colgroup1(w6);
	set_colgroup1(w7);
	strcat(HTML_String, "</colgroup>");
}
//------------------------------------------------------------------------------------------
void set_colgroup1(int ww) {
	if (ww == 0) return;
	strcat(HTML_String, "<col width=\"");
	strcati(HTML_String, ww);
	strcat(HTML_String, "\">");
}


//---------------------------------------------------------------------
void strcatf(char* tx, float f, byte leng, byte dezim) {
	char tmp[8];

	dtostrf(f, leng, dezim, tmp);//f,6,2,tmp
	strcat(tx, tmp);
}
//---------------------------------------------------------------------
void strcati(char* tx, int i) {
	char tmp[8];

	itoa(i, tmp, 10);
	strcat(tx, tmp);
}
