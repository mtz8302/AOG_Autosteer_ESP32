//--------------------------------------------------------------
//  EEPROM Data Handling 19. August 2021
//--------------------------------------------------------------
#define EEPROM_SIZE 1024 //16. April 2021: 2x 250 needed
#define EE_ident1 0xED  // Marker Byte 0 + 1
#define START_OF_FRIST_SET 4
//--------------------------------------------------------------
//  Restore EEprom Data
//--------------------------------------------------------------
void restoreEEprom() {
	Serial.println("read values from EEPROM");
	byte ECheck = EEprom_empty_check();
	if (ECheck == 3) { //first start?, breaking version up or downgrade
    EEprom_write_versionAndIdent();
		EEprom_write_FirstSet();  //write normal and default data
    EEprom_write_SecondSet();
		Serial.println("breaking Up or Downgrade - EEPROM write all");
	}
	else if (ECheck == 2) { //New not braking version change - data available
    EEprom_write_versionAndIdent();
		EEprom_read_all();
		Serial.println("New not braking version change - EEPROM read all");
	}
  else if (ECheck == 1) { //data available - ident version
    EEprom_read_all();
    Serial.println("Ident version - EEPROM read all");
  }
  else {
    Serial.println("ERROR with EEPROM !");
  }
	if (Set.debugmode) { EEprom_show_memory(); }
}
//--------------------------------------------------------------
int VersionCompare (int oldNr, int newNr, int NoChangeSinceNr){
  if (oldNr == newNr){
    return 0; // equal
  }
  else if (oldNr < newNr){ //increase
    if (oldNr >= NoChangeSinceNr){ //increase but NoChange
      return 1;
    }
    else { //increase and Change required
      return 2; 
    }
  }
  else if (oldNr > newNr){ //decrease
    return -1;
  }
}
//--------------------------------------------------------------
byte EEprom_empty_check() {
	if (!EEPROM.begin(EEPROM_SIZE))	{
		Serial.println("init EEPROM failed"); delay(1000);
		return 0; //error;
	}
  if (EEPROM.read(0) == EE_ident1) { //EEPROM ID Korrect     
    int major = VersionCompare (EEPROM.read(1), major_ver_nr, noChangeSince_major_ver_nr);
    int minor = VersionCompare (EEPROM.read(2), minor_ver_nr, noChangeSince_minor_ver_nr);
    int patchLevel = VersionCompare (EEPROM.read(3), patch_level_ver_nr, noChangeSince_patch_level_ver_nr);

    if (major == 2 || minor == 2 || patchLevel == 2) { //Breaking Version Upgrade - complete EEPROM rewrite
      Serial.println("Breaking Version Upgrade - complete EEPROM rewrite"); 
      delay(100);
      return 3; //flash all
    }
    else if (major == -1 || minor == -1 || patchLevel == -1) { //Version Downgrade - complete EEPROM rewrite
      Serial.println("Breaking Version Downgrade - complete EEPROM rewrite"); 
      delay(100);
      return 3; //flash all
    }
    else if (major == 1 || minor == 1 || patchLevel == 1) { //Not breaking Version Upgrade - version nr. EEPROM rewrite
      Serial.println("Not breaking Version Upgrade - version nr. EEPROM rewrite"); 
      delay(100);
      return 2; //flash at least new version nr
    }
	  else if (major == 0 && minor == 0 && patchLevel == 0) { //Same Version Nr, Not first boot or not new flashed
      Serial.println("EEPROM ident Version-Nr. found"); 
      delay(100);
      return 1; //do nothing
	  }
    Serial.println("This position should not be reached - error EEPROM 2"); 
    delay(100);
    return 0; //error
  }
  else {
    Serial.println("EEPROM no Id found - no data avaliable"); 
    delay(1000);
    return 0; //EEPROM no Id found
  }
  Serial.println("This position should not be reached - error EEPROM 3"); 
   delay(100);
  return 0; //error;
}
//-------------------------------------------------------------------------------------------------
void EEprom_write_versionAndIdent() {
  delay(50);
  EEPROM.write(0, EE_ident1); delay(2);
  EEPROM.write(1, major_ver_nr); delay(2);
  EEPROM.write(2, minor_ver_nr); delay(2);
  EEPROM.write(3, patch_level_ver_nr); delay(2);
  EEPROM.commit();
  delay(50);
}
//-------------------------------------------------------------------------------------------------
void EEprom_write_FirstSet() {
  int leng = sizeof(Set);
  if (2*leng+4 > EEPROM_SIZE){
    Serial.println("Settings are to long for EEPROM !");
    leng = (EEPROM_SIZE - START_OF_FRIST_SET) / 2;  // to prevent further damage to data
  }
  Serial.print("rewriting EEPROM + write 1. set at #");Serial.println(START_OF_FRIST_SET);
  for (int n = 0; n < leng; n++) {
    EEPROM.write(n + START_OF_FRIST_SET, ((unsigned char*)(&TempSet))[n]);
    delay(2);
  }
  delay(50);
  EEPROM.commit();
  delay(50);
}
//-------------------------------------------------------------------------------------------------
void EEprom_write_SecondSet() {
  int leng = sizeof(Set);
  uint16_t startOfSecondSet = ((EEPROM_SIZE - START_OF_FRIST_SET) / 2) +1 +START_OF_FRIST_SET;
  if (2*leng+4 > EEPROM_SIZE){
    Serial.println("Settings are to long for EEPROM !");
    leng = (EEPROM_SIZE - START_OF_FRIST_SET) / 2;  // to prevent further damage to data
  }
  Serial.print("rewriting EEPROM + write 2. set at #");Serial.println(startOfSecondSet);
  for (int n = 0; n < leng; n++) {
    EEPROM.write(n + startOfSecondSet, ((unsigned char*)(&TempSet))[n]);
    delay(2);
  }
  delay(50);
  EEPROM.commit();
  delay(50);
}
//-------------------------------------------------------------------------------------------------
void EEprom_write_TempSet() {
  int leng = sizeof(TempSet);
  for (int n = 0; n < leng; n++) {
    EEPROM.write(n + START_OF_FRIST_SET, ((unsigned char*)(&TempSet))[n]);
    delay(2);
  }
  delay(50);
  EEPROM.commit();
  delay(50);
}
//--------------------------------------------------------------
void EEprom_read_all() {
	int leng = sizeof(Set);
	Serial.print(leng);
	Serial.println(" Bytes reading from EEPROM ");
	for (int n = 0; n < leng; n++) {
		((unsigned char*)(&Set))[n] = EEPROM.read(n + START_OF_FRIST_SET);
	}
}
//--------------------------------------------------------------
void EEprom_read_default() {
	int leng = sizeof(Set);
  uint16_t startOfSecondSet = ((EEPROM_SIZE - START_OF_FRIST_SET) / 2) +1 +START_OF_FRIST_SET;
	for (int n = 0; n < leng; n++) {
		((unsigned char*)(&Set))[n] = EEPROM.read(n + startOfSecondSet);
	}
	Serial.print("load default value from EEPROM at #"); Serial.println(4 + sizeof(Set));
}

//--------------------------------------------------------------
void EEprom_block_restart() {
	if (EEPROM.read(2) == 0) {//prevents from restarting, when webpage is reloaded. Is set to 0, when other ACTION than restart is called
		EEPROM.write(2, 1);
		delay(2);
		EEPROM.commit();
		delay(50);
	}
}

//--------------------------------------------------------------
void EEprom_unblock_restart() {
	if (EEPROM.read(2) != 0) {
		EEPROM.write(2, 0); // reset Restart blocker
		delay(2);
		EEPROM.commit();
		delay(50);
	}
}

//--------------------------------------------------------------
void EEprom_show_memory() {
	byte c2 = 0, data_;
	Serial.print(EEPROM_SIZE, 1);
	Serial.println(" bytes read from Flash. Values are:");
	for (int i = 0; i < EEPROM_SIZE; i++)
	{
		data_ = byte(EEPROM.read(i));
		if (data_ < 0x10) Serial.print("0");
		Serial.print(data_, HEX);
		if (c2 == 15) {
			Serial.print(" ");
		}
		else if (c2 >= 31) {
			Serial.println(); //NL
			c2 = -1;
		}
		else Serial.print(" ");
		c2++;
	}
}
