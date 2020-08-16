// definitions and variables for webinterface
char HTML_String[20000];
int action;

#define ACTION_LoadDefaultVal   1
#define ACTION_RESTART          2
#define ACTION_SET_WAS_ZERO     3
#define ACTION_SET_INCL_ZERO    4
#define ACTION_SET_WS_THRESHOLD 5

// Radiobutton output
char output_driver_tab[5][22] = { "None", "Cytron MD30 + SWM", "IBT_2 +SWM", "IBT_2 +PWM Valve", "IBT_2 +Danfoss Valve" };

// Radiobutton analog input
char was_input_tab[3][25] = { "Arduino/ESP direct", "ADS 1115 single", "ADS 1115 differential" };

// Radiobutton IMU Heading Unit
char imu_type_tab[2][10] = { "None", "BNO 055" };

// Radiobutton Inclinometer
char inclino_type_tab[3][50] = { "None", "MMA 8452 at address 1C (adr PIN open)" ,"MMA 8452 at address 1D (adr PIN to GND)"};

// Radiobutton Steerswitch
char steersw_type_tab[5][15] = { "Switch High", "Switch Low", "Toggle Button", "Analog Buttons","" };

// Radiobutton Workswitch
char worksw_type_tab[4][8] = { "None", "Digital", "Analog", "" };

// Radiobutton WorkSwitch Invert
char worksw_invert_tab[2][15] = { "not inverted", "inverted" };

// Radiobutton Encoder
char encoder_type_tab[2][11] = { "None", "Installed" };

//-------------------------------------------------------------------------------------------------
//10. Mai 2020

void StartServer() {

	/*return index page which is stored in serverIndex */
	server.on("/", HTTP_GET, []() {
		make_HTML01();
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", HTML_String);
		if (steerSet.debugmode) { Serial.println("Webpage root"); }
		process_Request();
		make_HTML01();
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", HTML_String);
		});
	server.on("/serverIndex", HTTP_GET, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", serverIndex);
		});
	/*handling uploading firmware file */
	server.on("/update", HTTP_POST, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
		}, []() {
			HTTPUpload& upload = server.upload();
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

	server.onNotFound(handleNotFound);

	server.begin();
}

//---------------------------------------------------------------------
// Process given values 10. Mai 2020
//---------------------------------------------------------------------
void process_Request()
{
	action = 0;
	if (steerSet.debugmode) { Serial.print("From webinterface: number of arguments: "); Serial.println(server.args()); }
	for (byte n = 0; n < server.args(); n++) {
		if (steerSet.debugmode) {
			Serial.print("argName "); Serial.print(server.argName(n));
			Serial.print(" val: "); Serial.println(server.arg(n));
		}
		if (server.argName(n) == "ACTION") {
			action = int(server.arg(n).toInt());
			if (steerSet.debugmode) { Serial.print("Action found: "); Serial.println(action); }
		}
		if (action != ACTION_RESTART) { EEprom_unblock_restart(); }
		if (action == ACTION_LoadDefaultVal) {
			if (steerSet.debugmode) { Serial.println("load default settings from EEPROM"); }
			EEprom_read_default();
			delay(2);
		}
		//save changes
		if (server.argName(n) == "Save") {
			if (steerSet.debugmode) { Serial.println("Save button pressed in webinterface"); }
			EEprom_write_all();
		}

		if (server.argName(n) == "SSID_MY") {
			for (int i = 0; i < 24; i++) steerSet.ssid[i] = 0x00;
			int tempInt = server.arg(n).length() + 1;
			server.arg(n).toCharArray(steerSet.ssid, tempInt);
		}
		if (server.argName(n) == "Password_MY") {
			for (int i = 0; i < 24; i++) steerSet.password[i] = 0x00;
			int tempInt = server.arg(n).length() + 1;
			server.arg(n).toCharArray(steerSet.password, tempInt);
		}
		if (server.argName(n) == "timeoutRout") {
			argVal = server.arg(n).toInt();
			if ((argVal >= 20) && (argVal <= 1000)) { steerSet.timeoutRouter = int(argVal); }
		}
		if (server.argName(n) == "aogVer") {
			argVal = server.arg(n).toInt();
			if ((argVal >= 0) && (argVal <= 255)) { steerSet.aogVersion = byte(argVal); }
		}
		if (server.argName(n) == "DataTransfVia") {
			steerSet.DataTransVia = byte(server.arg(n).toInt());
			/*
			argVal = server.arg(n).toInt();			
			if (argVal == 0) steerSet.DataTransVia = 0;
			else if (argVal == 1) steerSet.DataTransVia = 1;
			else if (argVal == 4) steerSet.DataTransVia = 4;*/
		}
		if (server.argName(n) == "OUTPUT_TYPE") { steerSet.output_type = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "invMotor") {
			if (server.arg(n) == "true") { steerSet.MotorDriveDirection = 1; }
			else { steerSet.MotorDriveDirection = 0; }
		}
		if (server.argName(n) == "PWMFreq") {
			argVal = int(server.arg(n).toInt());
			if ((argVal <= 20000) && (argVal >= 20)) { steerSet.PWMOutFrequ = int(argVal); }
		}
		if (server.argName(n) == "MotSlow") {
			argVal = server.arg(n).toInt();
			if ((argVal <= 20) && (argVal >= 1)) { steerSet.MotorSlowDriveDegrees = byte(argVal); }
		}

		if (server.argName(n) == "INPUT_TYPE") { steerSet.input_type = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "AckermFix") { steerSet.AckermanFix = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "invWAS") {
			if (server.arg(n) == "true") { steerSet.Invert_WAS = 1; }
			else { steerSet.Invert_WAS = 0; }
		}
		if (action == ACTION_SET_WAS_ZERO) {
			steerSet.SteerPosZero = actualSteerPos; // >zero< Funktion Set Steer Angle to 0
			steerSet.steeringPositionZero = actualSteerPos;
			EEprom_write_all();
		}
		if (server.argName(n) == "IMU_TYPE") {
			steerSet.BNOInstalled = byte(server.arg(n).toInt());
			if (!steerSet.BNOInstalled) imu_initialized = 0;
		}
		if (server.argName(n) == "INCLINO_TYPE") {
			steerSet.InclinometerInstalled = byte(server.arg(n).toInt());
			//init MMA
			if (steerSet.InclinometerInstalled == 1) {
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
			if (steerSet.InclinometerInstalled == 2) {
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
		if (server.argName(n) == "MMAAxis") { steerSet.UseMMA_X_Axis = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "rollMaxChan") {
			argVal = server.arg(n).toInt();
			if ((argVal >= 1) && (argVal <= 50)) { steerSet.roll_MAX_STEP = byte(argVal); }
		}
		if (server.argName(n) == "invRoll") {
			if (server.arg(n) == "true") { steerSet.InvertRoll = 1; }
			else { steerSet.InvertRoll = 0; }
		}
		if (action == ACTION_SET_INCL_ZERO) {
			int roll_avg = 0;
			for (int i = 0; i < 16; i++) {
				roll_avg += x_;
				delay(200);
			}
			if (steerSet.InvertRoll == 1) { roll_avg = 0 - roll_avg; }
			steerSet.roll_corr = roll_avg >> 4;
			EEprom_write_all();
		}
		if (server.argName(n) == "ENC_TYPE") { steerSet.ShaftEncoder = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "ENC_COUNTS") { steerSet.pulseCountMax = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "SSWITCH_TYPE") { steerSet.SteerSwitch = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "RSWITCH_TYPE") { steerSet.SteerRemoteSwitch = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "WSWITCH_TYPE") { steerSet.WorkSW_mode = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "invWoSw") { 
			if (server.arg(n) == "true") { steerSet.Invert_WorkSW = 1; }
			else { steerSet.Invert_WorkSW = 0; }
		}
		if (action == ACTION_SET_WS_THRESHOLD) {
			unsigned int WSThres_avg = 0;
			for (int i = 0; i < 8; i++) {
				WSThres_avg += analogRead(steerSet.WORKSW_PIN);
				delay(100);
			}
			steerSet.WorkSW_Threshold = WSThres_avg >> 3;
			EEprom_write_all();
		}
		if (server.argName(n) == "MinSpeed") { steerSet.autoSteerMinSpeed4 = byte(server.arg(n).toInt()); }
		if (server.argName(n) == "MaxSpeed") {
			argVal = byte(server.arg(n).toInt());
			if ((argVal >= 10) && (argVal <= 30)) {
				steerSet.autosteerMaxSpeed4 = byte(argVal * 4);
			}
		}
		if (server.argName(n) == "debugmode") {
			if (server.arg(n) == "true") { steerSet.debugmode = true; }
			else { steerSet.debugmode = false; }
		}


		if (action == ACTION_RESTART) {
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
	strcat(HTML_String, "<body bgcolor=\"#66b3ff\">");
	strcat(HTML_String, "<font color=\"#000000\" face=\"VERDANA,ARIAL,HELVETICA\">");
	strcat(HTML_String, "<h1>AG Autosteer ESP config page</h1>");
	strcat(HTML_String, "by WEder/coffeetrac and MTZ8302<br>");
	strcat(HTML_String, "ver 4.3  -  16. Juni 2020 with OTA firmware update + send 2x<br><br><hr>");

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
	strcat(HTML_String, steerSet.ssid_ap);
	strcat(HTML_String, "</b>     with no password<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Network SSID:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?SSID_MY='+this.value)\" style= \"width:200px\" name=\"SSID_MY\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, steerSet.ssid);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" onchange=\"sendVal('/?Password_MY='+this.value)\" style= \"width:200px\" name=\"Password_MY\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, steerSet.password);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">time, trying to connect to network</td></tr>");
	strcat(HTML_String, "<td colspan=\"3\">after time has passed access point is opened</td></tr>");
	strcat(HTML_String, "<tr><td><b>Timeout (s):</b></td><td><input type = \"number\" onchange=\"sendVal('/?timeoutRout='+this.value)\" name = \"timeoutRout\" min = \"20\" max = \"1000\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.timeoutRouter);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr><td colspan=\"2\"><b>Restart NTRIP client for changes to take effect</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_RESTART);
	strcat(HTML_String, "')\" style= \"width:120px\" value=\"Restart\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
    // AOG Version

	strcat(HTML_String, "<h2>AOG Version number</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<b>If version number send by autosteer doesn't fit to AOG, AOG won't start.</b><br>");
	strcat(HTML_String, "AOG 4.3.10 = 4 + 3 + 10 = 17<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">for 4.1 and before set 0</td></tr>");
	strcat(HTML_String, "<tr><td><b>AOG Version code</b></td><td><input type = \"number\"  onchange=\"sendVal('/?aogVer='+this.value)\" name = \"aogVer\" min = \"0\" max = \"255\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.aogVersion);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");


	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------		
	// Data transfer via USB/Wifi 
	strcat(HTML_String, "<h2>USB or WiFi data transfer</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);
	//0 = USB 10byte / 1 = USB 10 byte 2x / 4 = USB 8 byte / 7 = UDP / 8 = UDP 2x
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>AOG 2019 and before</td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=4')\" name=\"DataTransfVia\" id=\"JZ\" value=\"4\"");
	if (steerSet.DataTransVia == 4)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB 8 byte sentence </label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>AOG V4</td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=0')\" name=\"DataTransfVia\" id=\"JZ\" value=\"0\"");
	if (steerSet.DataTransVia == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB 10 byte sentence </label></td></tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>AOG V4</td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=1')\" name=\"DataTransfVia\" id=\"JZ\" value=\"1\"");
	if (steerSet.DataTransVia == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB 10 byte send 2x </label></td></tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=7')\" name=\"DataTransfVia\" id=\"JZ\" value=\"7\"");
	if (steerSet.DataTransVia == 7)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">WiFi (UDP) (default)</label></td></tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?DataTransfVia=8')\" name=\"DataTransfVia\" id=\"JZ\" value=\"8\"");
	if (steerSet.DataTransVia == 8)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">WiFi (UDP) send 2x</label></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

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
		if (steerSet.SteerSwitch == i)strcat(HTML_String, " CHECKED");
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

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Remote button for</b></td>");
	strcat(HTML_String, "<td><input onclick=\"sendVal('/?RSWITCH_TYPE=0')\" type = \"radio\" name=\"RSWITCH_TYPE\" id=\"JZ0\" value=\"0\"");
	if (steerSet.SteerRemoteSwitch == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ0\">unused</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>AOG autosteer</b></td>");
	strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?RSWITCH_TYPE=1')\" name=\"RSWITCH_TYPE\" id=\"JZ1\" value=\"1\"");
	if (steerSet.SteerRemoteSwitch == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ1\">Switch to GND</label></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr><td>Remote switch linked to SteerSW</td>");
	strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?RSWITCH_TYPE=2')\" name=\"RSWITCH_TYPE\" id=\"JZ2\" value=\"2\"");
	if (steerSet.SteerRemoteSwitch == 2)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ2\"></label>ONLY with steer switch</td>");
	strcat(HTML_String, "</tr>");

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
		if (steerSet.WorkSW_mode == i)strcat(HTML_String, " CHECKED");
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
	if (steerSet.Invert_WorkSW == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> <b> Invert Workswitch</b></label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	//display Worksw value
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>Analog Workswitch Threshold value</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"><b>");
	strcati(HTML_String, (steerSet.WorkSW_Threshold));
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
		if (steerSet.ShaftEncoder == i)strcat(HTML_String, " CHECKED");
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
	strcati(HTML_String, steerSet.pulseCountMax);
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
		if (steerSet.output_type == i)strcat(HTML_String, " CHECKED");
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
	if (steerSet.MotorDriveDirection == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> <b> Invert Motor direction</b></label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "</tr>");

	//PWM frequency + Motor slow drive range
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>PWM output frequency</b></td>");
	strcat(HTML_String, "<td><input type = \"number\" onchange=\"sendVal('/?PWMFreq='+this.value)\" name = \"PWMFreq\" min = \"20\" max = \"20000\" step = \"10\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.PWMOutFrequ);
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
	strcati(HTML_String, steerSet.MotorSlowDriveDegrees);
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
		if (steerSet.input_type == i)strcat(HTML_String, " CHECKED");
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
	strcati(HTML_String, actualSteerPos);
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
	strcati(HTML_String, steerSet.AckermanFix);
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
	if (steerSet.Invert_WAS == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"><b> Invert wheel angle sensor</b></label>");
	strcat(HTML_String, "</td>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "<br><b>Steering to the left must be minus</b>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// min speed selction
	strcat(HTML_String, "<h2>Speed range for using autosteer</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);
	for (int i = 4; i > 0; i--) {
		strcat(HTML_String, "<tr>");
		if (i == 4)  strcat(HTML_String, "<td><b>min speed (km/h)</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?MinSpeed=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"MinSpeed\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (steerSet.autoSteerMinSpeed4 == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"> ");
		strcatf(HTML_String, float(i) / 4);
		strcat(HTML_String, "</label></td>");
		//strcat(HTML_String, "></td>");
		if (i == 4) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
		}
		strcat(HTML_String, "</tr>");
	}
	//max speed
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr><td><b>max speed (km/h)</b></td><td><input type = \"number\" onchange=\"sendVal('/?MaxSpeed='+this.value)\" name = \"MaxSpeed\" min = \"10\" max = \"30\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, int((steerSet.autosteerMaxSpeed4 * 0.25) + 0.3));
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// Inclinometer
	strcat(HTML_String, "<h2>Inclinometer Unit (Roll)</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 3; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Select your Inclinometer type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" onclick=\"sendVal('/?INCLINO_TYPE=");
		strcati(HTML_String, i);
		strcat(HTML_String, "')\" name=\"INCLINO_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (steerSet.InclinometerInstalled == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, inclino_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "</tr><tr><td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>Tilt Angle</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"><b>");
	strcatf(HTML_String, (XeRoll / 16));
	strcat(HTML_String, "</b></font></divbox>degree</td>");

	//Refresh button
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"location.reload()\" style= \"width:120px\" value=\"Refresh\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Calibrate Inclinometer</b></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?ACTION=");
	strcati(HTML_String, ACTION_SET_INCL_ZERO);
	strcat(HTML_String, "')\" style= \"width:200px\" value=\"ZERO NOW\"></button></td>");
	strcat(HTML_String, "<td>Tilt Calibration takes place on a flat area with no slope</td></tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	// Checkbox invert roll
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?invRoll='+this.checked)\" name=\"invRoll\" id = \"Part\" value = \"1\" ");
	if (steerSet.InvertRoll == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"><b> Invert roll</b></label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");

	// radio button use x/y axis
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\"><b>MMA orientation:</b></td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>MMA uses</td><td><input type = \"radio\" onclick=\"sendVal('/?MMAAxis=1')\" name=\"MMAAxis\" id=\"JZ\" value=\"1\"");
	if (steerSet.UseMMA_X_Axis == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">X axis (default)</label></td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" onclick=\"sendVal('/?MMAAxis=0')\" name=\"MMAAxis\" id=\"JZ\" value=\"0\"");
	if (steerSet.UseMMA_X_Axis == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">Y axis</label></td></tr>");

	//roll max change
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr><td><b>MMA Filter setting:</b></td></tr>");
	strcat(HTML_String, "<tr><td>maximum roll change per 100ms:</td><td><input type = \"number\" onchange=\"sendVal('/?rollMaxChan='+this.value)\" name = \"rollMaxChan\" min = \"1\" max = \"40\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.roll_MAX_STEP);
	strcat(HTML_String, "\"></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	// IMU Heading Unit

	strcat(HTML_String, "<h2>IMU Heading Unit (Compass)</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 2; i++) {
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
		if (steerSet.BNOInstalled == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, imu_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\"  style= \"width:120px\" value=\"Save\"></button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br>Heading</td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+1\"><b>");
	strcatf(HTML_String, (heading));
	strcat(HTML_String, "</b></font></divbox>degree</td>");

	//Refresh button
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"location.reload()\" style= \"width:120px\" value=\"Refresh\"></button></td>");
	strcat(HTML_String, "</tr>");


	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-------------------------------------------------------------
	// Checkbox debugmode
	strcat(HTML_String, "<h2>Debugmode</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	strcat(HTML_String, "<tr> <td colspan=\"3\">debugmode sends messages to USB serial</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type=\"checkbox\" onclick=\"sendVal('/?debugmode='+this.checked)\" name=\"debugmode\" id = \"Part\" value = \"1\" ");
	if (steerSet.debugmode == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> debugmode on</label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "<td><input type= \"button\" onclick= \"sendVal('/?Save=true')\" style= \"width:120px\" value=\"Save\"></button></td>");

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
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for (uint8_t i = 0; i < server.args(); i++) {
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
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
//------------------------------------------------------------------------------------------
void set_colgroup1(int ww) {
	if (ww == 0) return;
	strcat(HTML_String, "<col width=\"");
	strcati(HTML_String, ww);
	strcat(HTML_String, "\">");
}


//---------------------------------------------------------------------
void strcatf(char* tx, float f) {
	char tmp[8];

	dtostrf(f, 6, 2, tmp);
	strcat(tx, tmp);
}
//---------------------------------------------------------------------
void strcati(char* tx, int i) {
	char tmp[8];

	itoa(i, tmp, 10);
	strcat(tx, tmp);
}
