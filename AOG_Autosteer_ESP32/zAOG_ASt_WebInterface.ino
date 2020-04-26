
//-------------------------------------------------------------------------------------------------
// definitions and variables for webinterface
#define MAX_PACKAGE_SIZE 2048
char HTML_String[20000];
const char* host = "192.168.2.77";// "esp32";
bool firmwareUpdFileSel = false;
char HTTP_Header[150];
int Aufruf_Zaehler = 0;


#define ACTION_SET_SSID        1  
#define ACTION_SET_OUTPUT_TYPE 2  // also adress at EEPROM
#define ACTION_SET_WAS_TYPE    3
#define ACTION_SET_WAS_ZERO    4
//#define ACTION_SET_WAS_INVERT  5
#define ACTION_SET_IMU_TYPE    6
#define ACTION_SET_INCLINO     7
#define ACTION_SET_INCL_ZERO   8
#define ACTION_SET_ENCODER     9
#define ACTION_SET_SWITCHES    10
#define ACTION_SET_THRESHOLD   11
#define ACTION_SET_loadDefault 12
#define ACTION_SET_RESTART     13
#define ACTION_SET_DataTransfVia 14
#define ACTION_SET_debugmode   15
#define ACTION_SET_MinSpeed	   16
#define ACTION_SET_AOGVer	   17

int action;

// Radiobutton output
char output_driver_tab[5][22] = { "None", "Cytron MD30 + SWM", "IBT_2 +SWM", "IBT_2 +PWM Valve", "IBT_2 +Danfoss Valve" };

// Radiobutton analog input
char was_input_tab[3][25] = { "Arduino/ESP direct", "ADS 1115 single", "ADS 1115 differential" };

// Radiobutton WAS Invert
//char was_invert_tab[2][15] = { "not inverted", "inverted" };

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

char tmp_string[20];

//-------------------------------------------------------------------------------------------------
// 26. April 2020

void doWebInterface() {

	unsigned long my_timeout;

	// Check if a client has connected
	client_page = server.available();

	if (!client_page)  return;

	Serial.println("New Client:");           // print a message out the serial port

	my_timeout = millis() + 250L;
	while (!client_page.available() && (millis() < my_timeout)) { delay(10); }
	delay(10);
	if (millis() > my_timeout)
	{
		Serial.println("Client connection timeout!");
		client_page.flush();
		client_page.stop();
		return;
	}

	//---------------------------------------------------------------------
	//htmlPtr = 0;
	char c;
	if (client_page) {                        // if you get a client,
	  //Serial.print("New Client.\n");                   // print a message out the serial port
		String currentLine = "";                // make a String to hold incoming data from the client
		while (client_page.connected()) {       // loop while the client's connected
			delay(0);
			if (client_page.available()) {        // if there's bytes to read from the client,
				char c = client_page.read();        // read a byte, then
				Serial.print(c);                             // print it out the serial monitor
				if (c == '\n') {                    // if the byte is a newline character

				  // if the current line is blank, you got two newline characters in a row.
				  // that's the end of the client HTTP request, so send a response:
					if (currentLine.length() == 0) {

						if (firmwareUpdFileSel) {
							Serial.println("firmwareupdate page");
							client_page.print(serverIndex);
							Serial.println("root");
						}
						else {
							make_HTML01();   // create Page array
						   //---------------------------------------------------------------------
						   // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
						   // and a content-type so the client knows what's coming, then a blank line:
							strcpy(HTTP_Header, "HTTP/1.1 200 OK\r\n");
							strcat(HTTP_Header, "Content-Length: ");
							strcati(HTTP_Header, strlen(HTML_String));
							strcat(HTTP_Header, "\r\n");
							strcat(HTTP_Header, "Content-Type: text/html\r\n");
							strcat(HTTP_Header, "Connection: close\r\n");
							strcat(HTTP_Header, "\r\n");

							client_page.print(HTTP_Header);
							delay(20);
							send_HTML();
							// break out of the while loop:
							break;
						}
					}
					else {    // if you got a newline, then clear currentLine:
						currentLine = "";
					}
				}
				else if (c != '\r')
				{ // if you got anything else but a carriage return character,
					currentLine += c;      // add it to the end of the currentLine
					if (currentLine.endsWith("HTTP"))
					{
						if (currentLine.startsWith("GET "))
						{
							currentLine.toCharArray(HTML_String, currentLine.length());
							Serial.println(); //NL
							exhibit("Request : ", HTML_String);
							process_Request();
						}
					}
				}//end else
			} //end client available
		} //end while client.connected
		// close the connection:
		client_page.stop();
		Serial.print("Pagelength : ");
		Serial.print((long)strlen(HTML_String));
		Serial.print("   --> Client Disconnected\n");
	}// end if client 
}

//---------------------------------------------------------------------
// Process given values
//---------------------------------------------------------------------
void process_Request()  //OTA
{
	int myIndex;

	if (Find_Start("/serverIndex", HTML_String) > 0) {
		Serial.println(); Serial.println("firmwareupdater detected");
		firmwareUpdFileSel = true;
	}
	if (Find_Start("/?", HTML_String) < 0 && Find_Start("GET / HTTP", HTML_String) < 0)
	{
		//nothing to process
		return;
	}
	firmwareUpdFileSel = false;
	Serial.println("request seen");
	action = Pick_Parameter_Zahl("ACTION=", HTML_String);

	if (action != ACTION_SET_RESTART) { EEprom_unblock_restart(); }
	if (action == ACTION_SET_loadDefault) {
		EEprom_read_default();
		delay(5);
	}

	if (action == ACTION_SET_SSID) {// WiFi access data
		myIndex = Find_End("SSID_MY=", HTML_String);
		if (myIndex >= 0) {
			for (int i = 0; i < 24; i++) steerSet.ssid[i] = 0x00;
			Pick_Text(steerSet.ssid, &HTML_String[myIndex], 24);
			exhibit("SSID  : ", steerSet.ssid);
		}
		myIndex = Find_End("Password_MY=", HTML_String);
		if (myIndex >= 0) {
			for (int i = 0; i < 24; i++) steerSet.password[i] = 0x00;
			Pick_Text(steerSet.password, &HTML_String[myIndex], 24);
			exhibit("Password  : ", steerSet.password);
		}
		int tempint = Pick_Parameter_Zahl("timeoutRout=", HTML_String);
		if ((tempint >= 20) && (tempint <= 1000)) { steerSet.timeoutRouter = tempint; }
		EEprom_write_all();
	}

	if (action == ACTION_SET_AOGVer) {
		int tempint = Pick_Parameter_Zahl("aogVer=", HTML_String);
		if ((tempint >= 0) && (tempint <= 255)) { steerSet.aogVersion = tempint; }
		EEprom_write_all();
	}

	if (action == ACTION_SET_DataTransfVia) {
		//int temp = Pick_Parameter_Zahl("AOGNTRIP=", URI_answ_char);
		if (Pick_Parameter_Zahl("DataTransfVia=", HTML_String) == 0) steerSet.DataTransVia = 0;
		if (Pick_Parameter_Zahl("DataTransfVia=", HTML_String) == 1) steerSet.DataTransVia = 1;
		if (Pick_Parameter_Zahl("DataTransfVia=", HTML_String) == 4) steerSet.DataTransVia = 4;
		EEprom_write_all();
	}

	if (action == ACTION_SET_OUTPUT_TYPE) {
		steerSet.output_type = Pick_Parameter_Zahl("OUTPUT_TYPE=", HTML_String);
		int tempint = Pick_Parameter_Zahl("PWMFreq=", HTML_String);
		if ((tempint <= 20000) && (tempint >= 20)) { steerSet.PWMOutFrequ = tempint; }
		tempint = Pick_Parameter_Zahl("MotSlow=", HTML_String);
		if ((tempint <= 20) && (tempint >= 1)) { steerSet.MotorSlowDriveDegrees = tempint; }
		EEprom_write_all();
	}

	if (action == ACTION_SET_WAS_TYPE) {
		steerSet.input_type = Pick_Parameter_Zahl("INPUT_TYPE=", HTML_String);
		steerSet.AckermanFix = Pick_Parameter_Zahl("rightMul=", HTML_String);
		byte tempby = Pick_Parameter_Zahl("invWAS=", HTML_String);
		if (tempby == 1) {
			steerSet.Invert_WAS = tempby;
		}
		else { steerSet.Invert_WAS = 0; }//no value back from page = 0
		EEprom_write_all();
	}

	if (action == ACTION_SET_WAS_ZERO) {
		steerSet.SteerPosZero = actualSteerPos; // >zero< Funktion Set Steer Angle to 0
		steerSet.steeringPositionZero = actualSteerPos;
		EEprom_write_all();
	}
	if (action == ACTION_SET_IMU_TYPE)
	{
		steerSet.BNOInstalled = Pick_Parameter_Zahl("IMU_TYPE=", HTML_String);
		if (!steerSet.BNOInstalled) imu_initialized = 0;
		EEprom_write_all();
	}

	if (action == ACTION_SET_INCLINO)
	{
		steerSet.InclinometerInstalled = Pick_Parameter_Zahl("INCLINO_TYPE=", HTML_String);
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
		byte tempby = Pick_Parameter_Zahl("invRoll=", HTML_String);
		if (tempby == 1) {
			steerSet.InvertRoll = tempby;
		}
		else { steerSet.InvertRoll = 0; }//no value back from page = 0
		if (Pick_Parameter_Zahl("MMAAxis=", HTML_String) == 1) steerSet.UseMMA_X_Axis = 1;
		if (Pick_Parameter_Zahl("MMAAxis=", HTML_String) == 0) steerSet.UseMMA_X_Axis = 0;
		
		tempby = Pick_Parameter_Zahl("rollMaxChan=", HTML_String);
		if ((tempby >= 1) && (tempby <= 50)) {
			steerSet.roll_MAX_STEP = tempby;
		}
		EEprom_write_all();
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

	if (action == ACTION_SET_ENCODER) {
		steerSet.ShaftEncoder = Pick_Parameter_Zahl("ENC_TYPE=", HTML_String);
		steerSet.pulseCountMax = Pick_Parameter_Zahl("ENC_COUNTS=", HTML_String);
		EEprom_write_all();
	}
	if (action == ACTION_SET_SWITCHES) {
		steerSet.SteerSwitch = Pick_Parameter_Zahl("SSWITCH_TYPE=", HTML_String);
		steerSet.SteerRemoteSwitch = Pick_Parameter_Zahl("RSWITCH_TYPE=", HTML_String);
		steerSet.WorkSW_mode = Pick_Parameter_Zahl("WSWITCH_TYPE=", HTML_String);
		steerSet.Invert_WorkSW = Pick_Parameter_Zahl("IWSWITCH_TYPE=", HTML_String);
		EEprom_write_all();
	}
	if (action == ACTION_SET_THRESHOLD) {
		unsigned int WSThres_avg = 0;
		for (int i = 0; i < 8; i++) {
			WSThres_avg += analogRead(steerSet.WORKSW_PIN);
			delay(100);
		}
		steerSet.WorkSW_Threshold = WSThres_avg >> 3;
		EEprom_write_all();
	}
	if (action == ACTION_SET_MinSpeed) {
		steerSet.autoSteerMinSpeed4 = Pick_Parameter_Zahl("MinSpeed=", HTML_String);
		byte tempby = Pick_Parameter_Zahl("MaxSpeed=", HTML_String);
		if ((tempby >= 10) && (tempby <= 65)) {
			if (tempby == 64) { steerSet.autosteerMaxSpeed4 = 255; }
			else { steerSet.autosteerMaxSpeed4 = tempby * 4; }
		}
		EEprom_write_all();
	}
	if (action == ACTION_SET_debugmode)
	{
		byte tempby = Pick_Parameter_Zahl("debugmode=", HTML_String);
		if (tempby == 1) {
			steerSet.debugmode = true;
		}
		else { steerSet.debugmode = false; }//no value back from page = 0
		EEprom_write_all();
	}
	if (action == ACTION_SET_RESTART) {
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
	strcat(HTML_String, "ver 4.2.2  26. Apr. 2020<br><br><hr>");

	//---------------------------------------------------------------------------------------------  
	//load values of INO setup zone
	strcat(HTML_String, "<h2>Load default values of INO setup zone</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(270, 250, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"2\">Only load default values, does NOT save them</td>");
	strcat(HTML_String, "<td><button style= \"width:150px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_loadDefault);
	strcat(HTML_String, "\">Load default values</button></td>");
	strcat(HTML_String, "</tr>");
	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br>");

	//-----------------------------------------------------------------------------------------
	// WiFi Client Access Data

	strcat(HTML_String, "<hr><h2>WiFi Network Client Access Data</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "</b>If access to networks fails, an accesspoint will be created:<br>SSID: <b>");
	strcat(HTML_String, steerSet.ssid_ap);
	strcat(HTML_String, "</b>     with no password<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Network SSID:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"SSID_MY\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, steerSet.ssid);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_SSID);
	strcat(HTML_String, "\">Apply and Save</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Password:</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"Password_MY\" maxlength=\"22\" Value =\"");
	strcat(HTML_String, steerSet.password);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">time, trying to connect to network</td></tr>");
	strcat(HTML_String, "<td colspan=\"3\">after time has passed access point is opened</td></tr>");
	strcat(HTML_String, "<tr><td><b>Timeout (s):</b></td><td><input type = \"number\"  name = \"timeoutRout\" min = \"20\" max = \"1000\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.timeoutRouter);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr><td colspan=\"2\"><b>Restart NTRIP client for changes to take effect</b></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_RESTART);
	strcat(HTML_String, "\">Restart</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
// AOG Version

	strcat(HTML_String, "<h2>AOG Version number</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<b>If version number send by autosteer doesn't fit to AOG, AOG won't start.</b><br>");
	strcat(HTML_String, "AOG 4.2.01 = 4 + 2 +1 = 7<br><br><table>");
	set_colgroup(250, 300, 150, 0, 0);

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">for 4.1 and before set 0</td></tr>");
	strcat(HTML_String, "<tr><td><b>AOG Version code</b></td><td><input type = \"number\"  name = \"aogVer\" min = \"0\" max = \"255\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.aogVersion);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_AOGVer);
	strcat(HTML_String, "\">Apply and Save</button></td>");
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

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>AOG 2019 and before</td><td><input type = \"radio\" name=\"DataTransfVia\" id=\"JZ\" value=\"0\"");
	if (steerSet.DataTransVia == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB 8 byte sentence </label></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_DataTransfVia);
	strcat(HTML_String, "\">Apply and Save</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>AOG V4</td><td><input type = \"radio\" name=\"DataTransfVia\" id=\"JZ\" value=\"4\"");
	if (steerSet.DataTransVia == 4)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">USB 10 byte sentence </label></td></tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" name=\"DataTransfVia\" id=\"JZ\" value=\"1\"");
	if (steerSet.DataTransVia == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">WiFi (UDP) (default)</label></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");

	//-----------------------------------------------------------------------------------------
	 // Steerswitch Type
	strcat(HTML_String, "<h2>Switch Types</h2>");
	strcat(HTML_String, "<form>");
	strcat(HTML_String, "<table>");
	set_colgroup(300, 250, 150, 0, 0);

	for (int i = 0; i < 5; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Steerswitch type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" name=\"SSWITCH_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (steerSet.SteerSwitch == i)strcat(HTML_String, " CHECKED");
		if (i == 4) strcat(HTML_String, " disabled");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, steersw_type_tab[i]);
		strcat(HTML_String, "</label></td>");
		if (i == 0) {
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_SWITCHES);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Remote button for</b></td>");
	strcat(HTML_String, "<td><input type = \"radio\" name=\"RSWITCH_TYPE\" id=\"JZ0\" value=\"0\"");
	if (steerSet.SteerRemoteSwitch == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ0\">unused</label></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_SWITCHES);
	strcat(HTML_String, "\">Apply and Save</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>AOG autosteer</b></td>");
	strcat(HTML_String, "<td><input type = \"radio\" name=\"RSWITCH_TYPE\" id=\"JZ1\" value=\"1\"");
	if (steerSet.SteerRemoteSwitch == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ1\">Switch to GND</label></td>");
	strcat(HTML_String, "</tr>");
	
	if (steerSet.SteerSwitch < 2) {
		strcat(HTML_String, "<tr><td></td>");
		strcat(HTML_String, "<td><input type = \"radio\" name=\"RSWITCH_TYPE\" id=\"JZ2\" value=\"2\"");
		if (steerSet.SteerRemoteSwitch == 2)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ2\">Remote switch linked to SteerSW</label></td>");
		strcat(HTML_String, "</tr>");
	}
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	for (int i = 0; i < 4; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Workswitch type</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" name=\"WSWITCH_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (steerSet.WorkSW_mode == i)strcat(HTML_String, " CHECKED");
		if (i == 3) strcat(HTML_String, " disabled");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, worksw_type_tab[i]);
		strcat(HTML_String, "</label></td>");
	}

	for (int i = 0; i < 2; i++) {
		strcat(HTML_String, "<tr>");
		if (i == 0)  strcat(HTML_String, "<td><b>Invert Workswitch</b></td>");
		else strcat(HTML_String, "<td> </td>");
		strcat(HTML_String, "<td><input type = \"radio\" name=\"IWSWITCH_TYPE\" id=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\" value=\"");
		strcati(HTML_String, i);
		strcat(HTML_String, "\"");
		if (steerSet.Invert_WorkSW == i)strcat(HTML_String, " CHECKED");
		strcat(HTML_String, "><label for=\"JZ");
		strcati(HTML_String, i);
		strcat(HTML_String, "\">");
		strcat(HTML_String, worksw_invert_tab[i]);
		strcat(HTML_String, "</label></td>");
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br><font size=\"+1\">Analog Workswitch Threshold value</font></td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
	strcati(HTML_String, (steerSet.WorkSW_Threshold));
	strcat(HTML_String, "</b></font></divbox>0-4095</td>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Set Threshold</b></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_THRESHOLD);
	strcat(HTML_String, "\">Use Current</button></td>");
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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"ENC_TYPE\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_ENCODER);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Counts to turn off Autosteer</b></td>");
	strcat(HTML_String, "<td>");
	strcat(HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"ENC_COUNTS\" maxlength=\"3\" Value =\"");
	strcati(HTML_String, steerSet.pulseCountMax);
	strcat(HTML_String, "\"></td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_ENCODER);
	strcat(HTML_String, "\">Apply and Save</button></td>");
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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"OUTPUT_TYPE\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_OUTPUT_TYPE);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	//PWM frequency + Motor slow drive range
	if (steerSet.output_type > 0) {
		strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
		strcat(HTML_String, "<tr>");
		strcat(HTML_String, "<td><b>PWM output frequency</b></td>");
		strcat(HTML_String, "<td><input type = \"number\"  name = \"PWMFreq\" min = \"20\" max = \"20000\" step = \"10\" style= \"width:200px\" value = \"");// placeholder = \"");
		strcati(HTML_String, steerSet.PWMOutFrequ);
		strcat(HTML_String, "\"></td>");
		strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
		strcati(HTML_String, ACTION_SET_OUTPUT_TYPE);
		strcat(HTML_String, "\">Apply and Save</button></td>");
		strcat(HTML_String, "</tr>");
		strcat(HTML_String, "</tr>");
		strcat(HTML_String, "<td colspan=\"3\">1000 Hz for low heat at PWM device</td></tr>");
		strcat(HTML_String, "<td colspan=\"3\">20000 Hz not hearable</td></tr>");

		strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
		strcat(HTML_String, "<tr>");
		strcat(HTML_String, "<td colspan=\"3\"><b>Motor/Valve is driving slower if steer angle error is less then x degrees:</b></td></tr>");
		strcat(HTML_String, "<td>slow drive range (degr):</td>");
		strcat(HTML_String, "<td><input type = \"number\"  name = \"MotSlow\" min = \"1\" max = \"10\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
		strcati(HTML_String, steerSet.MotorSlowDriveDegrees);
		strcat(HTML_String, "\"></td>");
		strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
		strcati(HTML_String, ACTION_SET_OUTPUT_TYPE);
		strcat(HTML_String, "\">Apply and Save</button></td>");
		strcat(HTML_String, "</tr>");
		strcat(HTML_String, "</tr>");


	}
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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"INPUT_TYPE\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_WAS_TYPE);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br><font size=\"+1\">WAS RAW Data</font></td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
	strcati(HTML_String, steeringPosition);
	strcat(HTML_String, "</b></font></divbox></td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_WAS_TYPE);
	strcat(HTML_String, "\">Refresh</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Center your Sensor to Zero</b></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_WAS_ZERO);
	strcat(HTML_String, "\">ZERO NOW</button></td>");
	strcat(HTML_String, "<td>Your Wheels should face straight ahead</td>");


	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Ackerman fix factor (%)</b></td>");
	strcat(HTML_String, "<td><input type = \"number\"  name = \"rightMul\" min = \"35\" max = \"300\" step = \"1\" style= \"width:120px\" value = \"");// placeholder = \"");
	strcati(HTML_String, steerSet.AckermanFix);
	strcat(HTML_String, "\"></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_WAS_TYPE);
	strcat(HTML_String, "\">Apply and Save</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr><td colspan=\"3\">if values for right and left side are the same: factor = 100 </td></tr>");
	strcat(HTML_String, "<tr><td colspan=\"3\">if values to the right are higher than to the left: 35 < factor < 100</td></tr>");
	strcat(HTML_String, "<tr><td colspan=\"3\">if values to the left are higher than to the right: 100 < factor < 300</td></tr>");

	// Checkbox invert WAS
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type=\"checkbox\" name=\"invWAS\" id = \"Part\" value = \"1\" ");
	if (steerSet.Invert_WAS == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> invert wheel angle sensor</label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_WAS_TYPE);
	strcat(HTML_String, "\">Apply and Save</button></td></tr>");

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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"MinSpeed\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_MinSpeed);
			strcat(HTML_String, "\">Apply and Save</button></td>");

		}
		strcat(HTML_String, "</tr>");
	}
	//max speed
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr><td colspan=\"3\">Set to 64 if you whant to use autosteer backwards, but this is NOT SAVE!!</td></tr>");
	strcat(HTML_String, "<tr><td><b>max speed (km/h)</b></td><td><input type = \"number\"  name = \"MaxSpeed\" min = \"10\" max = \"64\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"INCLINO_TYPE\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_INCLINO);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br><font size=\"+1\">Tilt Angle</font></td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
	strcatf(HTML_String, (XeRoll / 16));
	strcat(HTML_String, "</b></font></divbox>degree</td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_INCLINO);
	strcat(HTML_String, "\">Refresh</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><b>Calibrate Inclinometer</b></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_INCL_ZERO);
	strcat(HTML_String, "\">ZERO NOW</button></td>");
	strcat(HTML_String, "<td>Tilt Calibration takes place on a flat area with no slope</td></tr>");

	// Checkbox invert roll
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type=\"checkbox\" name=\"invRoll\" id = \"Part\" value = \"1\" ");
	if (steerSet.InvertRoll == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> invert roll</label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_INCLINO);
	strcat(HTML_String, "\">Apply and Save</button></td></tr>");

	// radio button use x/y axis
	strcat(HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
	strcat(HTML_String, "<tr> <td colspan=\"3\"><b>MMA orientation:</b></td> </tr>");
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td>MMA uses</td><td><input type = \"radio\" name=\"MMAAxis\" id=\"JZ\" value=\"1\"");
	if (steerSet.UseMMA_X_Axis == 1)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">X axis (default)</label></td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_INCLINO);
	strcat(HTML_String, "\">Apply and Save</button></td>");
	strcat(HTML_String, "</tr>");

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td></td><td><input type = \"radio\" name=\"MMAAxis\" id=\"JZ\" value=\"0\"");
	if (steerSet.UseMMA_X_Axis == 0)strcat(HTML_String, " CHECKED");
	strcat(HTML_String, "><label for=\"JZ\">Y axis</label></td></tr>");

	//roll max change
	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td colspan=\"3\">&nbsp;</td></tr>");
	strcat(HTML_String, "<tr><td><b>MMA Filter setting:</b></td></tr>");
	strcat(HTML_String, "<tr><td>maximum roll change per 100ms:</td><td><input type = \"number\"  name = \"rollMaxChan\" min = \"1\" max = \"40\" step = \"1\" style= \"width:200px\" value = \"");// placeholder = \"");
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
		strcat(HTML_String, "<td><input type = \"radio\" name=\"IMU_TYPE\" id=\"JZ");
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
			strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
			strcati(HTML_String, ACTION_SET_IMU_TYPE);
			strcat(HTML_String, "\">Apply and Save</button></td>");
			strcat(HTML_String, "</tr>");
		}
	}

	strcat(HTML_String, "<tr>");
	strcat(HTML_String, "<td><br><font size=\"+1\">Heading</font></td>");
	strcat(HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
	strcatf(HTML_String, (heading));
	strcat(HTML_String, "</b></font></divbox>degree</td>");

	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_IMU_TYPE);
	strcat(HTML_String, "\">Refresh</button></td>");
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
	strcat(HTML_String, "<td></td><td><input type=\"checkbox\" name=\"debugmode\" id = \"Part\" value = \"1\" ");
	if (steerSet.debugmode == 1) strcat(HTML_String, "checked ");
	strcat(HTML_String, "> ");
	strcat(HTML_String, "<label for =\"Part\"> debugmode on</label>");
	strcat(HTML_String, "</td>");
	strcat(HTML_String, "<td><button style= \"width:120px\" name=\"ACTION\" value=\"");
	strcati(HTML_String, ACTION_SET_debugmode);
	strcat(HTML_String, "\">Apply and Save</button></td></tr>");

	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");
/*
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

	strcat(HTML_String, "<td><input type='submit' onclick='openUpload(this.form)' value='Open Firmware uploader'></td></tr>");
		
	strcat(HTML_String,"<script>" );
	strcat(HTML_String,"function openUpload(form)" );
	strcat(HTML_String, "{");
	strcat(HTML_String,"window.open('/serverIndex')" );
	strcat(HTML_String, "}");
	strcat(HTML_String,"</script>" );
	
	strcat(HTML_String, "</table>");
	strcat(HTML_String, "</form>");
	strcat(HTML_String, "<br><hr>");
*/
	//-------------------------------------------------------------  
	strcat(HTML_String, "</font>");
	strcat(HTML_String, "</font>");
	strcat(HTML_String, "</body>");
	strcat(HTML_String, "</html>");
}

//--------------------------------------------------------------------------

void send_not_found() {

	Serial.println("Send Not Found");

	client_page.print("HTTP/1.1 404 Not Found\r\n\r\n");
	delay(20);
	//client.stop();
}

//--------------------------------------------------------------------------
void send_HTML() {
	char my_char;
	int  my_len = strlen(HTML_String);
	int  my_ptr = 0;
	int  my_send = 0;

	//--------------------------------------------------------------------------
	// in Portionen senden
	while ((my_len - my_send) > 0) {
		my_send = my_ptr + MAX_PACKAGE_SIZE;
		if (my_send > my_len) {
			client_page.print(&HTML_String[my_ptr]);
			delay(20);

			//Serial.println(&HTML_String[my_ptr]);

			my_send = my_len;
		}
		else {
			my_char = HTML_String[my_send];
			// Auf Anfang eines Tags positionieren
			while (my_char != '<') my_char = HTML_String[--my_send];
			HTML_String[my_send] = 0;
			client_page.print(&HTML_String[my_ptr]);
			delay(20);

			//Serial.println(&HTML_String[my_ptr]);

			HTML_String[my_send] = my_char;
			my_ptr = my_send;
		}
	}
	//client.stop();
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

//---------------------------------------------------------------------
void strcati2(char* tx, int i) {
	char tmp[8];

	itoa(i, tmp, 10);
	if (strlen(tmp) < 2) strcat(tx, "0");
	strcat(tx, tmp);
}

//---------------------------------------------------------------------
int Pick_Parameter_Zahl(const char* par, char* str) {
	int myIdx = Find_End(par, str);

	if (myIdx >= 0) return  Pick_Dec(str, myIdx);
	else return -1;
}
//---------------------------------------------------------------------
int Find_End(const char* such, const char* str) {
	int tmp = Find_Start(such, str);
	if (tmp >= 0)tmp += strlen(such);
	return tmp;
}

//---------------------------------------------------------------------
int Find_Start(const char* such, const char* str) {
	int tmp = -1;
	int ww = strlen(str) - strlen(such);
	int ll = strlen(such);

	for (int i = 0; i <= ww && tmp == -1; i++) {
		if (strncmp(such, &str[i], ll) == 0) tmp = i;
	}
	return tmp;
}
//---------------------------------------------------------------------
int Pick_Dec(const char* tx, int idx) {
	int tmp = 0;

	for (int p = idx; p < idx + 5 && (tx[p] >= '0' && tx[p] <= '9'); p++) {
		tmp = 10 * tmp + tx[p] - '0';
	}
	return tmp;
}
//----------------------------------------------------------------------------
int Pick_N_Zahl(const char* tx, char separator, byte n) {

	int ll = strlen(tx);
	int tmp = -1;
	byte anz = 1;
	byte i = 0;
	while (i < ll && anz < n) {
		if (tx[i] == separator)anz++;
		i++;
	}
	if (i < ll) return Pick_Dec(tx, i);
	else return -1;
}

//---------------------------------------------------------------------
int Pick_Hex(const char* tx, int idx) {
	int tmp = 0;

	for (int p = idx; p < idx + 5 && ((tx[p] >= '0' && tx[p] <= '9') || (tx[p] >= 'A' && tx[p] <= 'F')); p++) {
		if (tx[p] <= '9')tmp = 16 * tmp + tx[p] - '0';
		else tmp = 16 * tmp + tx[p] - 55;
	}

	return tmp;
}

//---------------------------------------------------------------------
void Pick_Text(char* tx_ziel, char* tx_quelle, int max_ziel) {

	int p_ziel = 0;
	int p_quelle = 0;
	int len_quelle = strlen(tx_quelle);

	while (p_ziel < max_ziel && p_quelle < len_quelle && tx_quelle[p_quelle] && tx_quelle[p_quelle] != ' ' && tx_quelle[p_quelle] != '&') {
		if (tx_quelle[p_quelle] == '%') {
			tx_ziel[p_ziel] = (HexChar_to_NumChar(tx_quelle[p_quelle + 1]) << 4) + HexChar_to_NumChar(tx_quelle[p_quelle + 2]);
			p_quelle += 2;
		}
		else if (tx_quelle[p_quelle] == '+') {
			tx_ziel[p_ziel] = ' ';
		}
		else {
			tx_ziel[p_ziel] = tx_quelle[p_quelle];
		}
		p_ziel++;
		p_quelle++;
	}

	tx_ziel[p_ziel] = 0;
}
//---------------------------------------------------------------------
char HexChar_to_NumChar(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 55;
	return 0;
}
//---------------------------------------------------------------------
void exhibit(const char* tx, int v) {
	Serial.print(tx);
	Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char* tx, unsigned int v) {
	Serial.print(tx);
	Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char* tx, unsigned long v) {
	Serial.print(tx);
	Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char* tx, const char* v) {
	Serial.print(tx);
	Serial.println(v);
}
