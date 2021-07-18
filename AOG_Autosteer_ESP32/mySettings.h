#ifdef USE_CUSTOM_SETTINGS
  //general settings
  #define EEPROM_CLEAR true                     //set to true when changing settings to write them as default values: true -> flash -> boot -> false -> flash again

  #define AOG_VERSION 20                        // Version number for version check 4.3.10 = 4+3+10 = 17  
  #define DATA_TRANS_VIA 7                      // transfer data via 0 = USB / 7 = WiFi UDP / 10 = Ethernet UDP

  #define DEBUG_MODE false
  #define DEBUG_MODE_DATA_FROM_AOG false
  
  #define USE_LED_BUILTIN 0                     // some ESP board have a build in LED, some not. Here it's the same funtion as the WiFi LED

//Features
  //MOTOR/OUTPUT  
  #define OUTPUT_TYPE 5                         // set to 1  if you want to use Stering Motor + Cytron MD30C Driver
                                                // set to 2  if you want to use Stering Motor + IBT 2  Driver
                                                // set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
                                                // set to 4  if you want to use  IBT 2  Driver + Danfoss Valve PVE A/H/M
                                                // set to 5  if you want to use an (CLOSED LOOP) STEPPER DRIVER
  #define MOTOR_DRIVE_DIRECTION 1               // 0 = normal, 1 = inverted
  #define MOTOR_SLOW_DRIVE_DEGREES 5            // How many degrees before decreasing Max PWM
  #define PWM_OUT_FREQU 20000                   // PWM frequency for motordriver: 1000Hz:for low heat at PWM device 20000Hz: not hearable
  #define STEPPER_KP_TO_DEGREES_FACTOR 10       // when setting Kp by WebIO or AGO the stepper-steps per degree WAS will be recalculated "stepPerPositionDegree = Kp * STEPPER_KP_TO_DEGREES_FACTOR" to make it adjustable the common and easy way
  #define STEPPER_HIGHPWM_TO_MAXSPEED_FACTOR 50 // when setting highRPM by WebIO or AGO the stepper-maxSpeed will be recalculated "maxSpeed = highRPM * STEPPER_HIGHRPM_TO_MAXSPEED_FACTOR" to make it adjustable the common and easy way
  #define STEPPER_LOWPWM_TO_ACCELERATION_FACTOR 50 // when setting lowRPM by WebIO or AGO the stepper-acceleration will be recalculated "acceleration = lowRPM * STEPPER_LOWRPM_TO_ACCELERATION_FACTOR" to make it adjustable the common and easy way

  //WAS
  #define WAS_TYPE 2                            // 0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 36 (pin set below) (attention 3,3V only)
                                                // 1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
                                                // 2 = Differential Mode - Connect Sensor GND to A1, Signal to A0
  #define INVERT_WAS 1                          // set to 1 to Change Direction of Wheel Angle Sensor - to +   
  #define ACKERMAN_FIX 78                       // if values for left and right are the same: 100                                            
  
  //SWITCHES
  #define STEER_SWITCH_TYPE 1                   // 0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
                                                // 3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V
                                                // 255 = no steer switch, allways on if AOG steering is active
                                                
  #define WORSK_SW_MODE 1                       // 0 = disabled // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)
  #define INVERT_WORK_SW 0                      // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low
  #define WORK_SW_THRESHOLD 1600                // Value for analog hitch level to switch workswitch  (0-4096)

  //AUTOSTEER
  #define AUTOTSTEER_MIN_SPEED 0.2               // Min speed to use autosteer km/h
  #define AUTOTSTEER_MAX_SPEED 30                // Max speed to use autosteer km/h

  //IMU/COMPAS
  #define IMU_TYPE 0                             // 0: none, 1: BNO055 IMU, 2: CMPS14, 3: BNO080 + BNO085 
  #define INVERT_ROLL 0                          // 0: no, set to 1 to change roll direction
    //CMPS14  
    #define I2C_CMPS14_ADDRESS 0x60               // Address of CMPS14 shifted right one bit for arduino wire library
    #define CMPS14_HEADING_CORRECTION 0.0         // not used at the moment
    #define CMPS14_ROLL_CORRECTION 0.0            // not used at the moment
    #define CMPS14_USED_AXIS 0                    // not used at the moment
  
    //BNO08x
    #define I2C_BNO08X_ADDRESS {0x4A,0x4B}        // BNO08x address variables to check where it is
    #define BNO_HEADING_CORRECTION 0.0            // not used at the moment
    #define BNO_ROLL_CORRECTION 0.0               // not used at the moment
    #define BNO_USED_AXIS 0                       // not used at the moment
  
    //MMA
    #define MMA_INSTALLED 0                       // set to 1 if MMA8452 is installed at address 1C (Adr PIN to GND) set to 2 at address 1D (Adr PIN open)
    #define USE_MMA_X_AXIS 1                      // 1: use X axis (default) 0: use Y axis
    #define MMA_ROLL_MAX_STEP 10                  // max roll step per loop (5-20) higher = filter less

  //ENCODER  
  #define SHAFT_ENCODER 0                       // Steering Wheel ENCODER Installed
  #define PULSE_COUNT_MAX 3                     // Switch off Autosteer after x Pulses from Steering wheel encoder

  //CURRENT SENSOR
  #define CURRENT_SENSOR 0                      // (not supported at the moment)
  
  //PRESSURE_SENSOR
  #define PRESSURE_SENSOR 0                     // (not supported at the moment)

//NETWORK 
  //PORTS
  #define PORT_AUTOST_TO_AOG 5577               // this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
  #define PORT_FROM_AOG 8888                    // port to listen for AOG
  #define PORT_DESTINATION 9999                 // port of AOG that listens
  
  //WiFi
  #define WIFI_SSID "TS115_WLAN"                // WiFi network Client name
  #define WIFI_PASSWORD "TS115_WLAN"                      // WiFi network password
  #define WIFI_SSID_AP "Autosteer_Unit_TS115"   // name of Access point, if no WiFi found, NO password!!
  #define WIFI_TIMEOUT_ROUTER 30                // time (seconds) to wait for WIFI access, after that own Access Point starts
  #define WIFI_TIMEOUT_TIMEOUT_WEBIO 255        // time (min) afterwards webinterface is switched off
  #define WIFI_MYIP {192, 168, 99, 77}          // autosteer module 
  #define WIFI_GWIP {192, 168, 99, 1}           // Gateway IP only used if Accesspoint created
  #define WIFI_IPDEST_ENDING 255                // ending of IP address to send UDP data to
  #define WIFI_MASK {255, 255, 255, 0}
  #define WIFI_MYDNS {8, 8, 8, 8}               //optional

  //Ethernet
  #define ETHERNET_MYIP {192, 168, 1, 78}       // autosteer module 
  #define ETHERNET_IPDEST_ENDING 255            // ending of IP address to send UDP data to
  #define ETHERNET_MAC {0x70,0x69,0x69,0x2D,0x30,0x31}
  #define ETHERNET_STATIC_IP false              // false = use DHPC and set last number to 80 (x.x.x.80) / true = use IP as set above

  //WEBIO
  #define WEBIO_STEER_POS_ZERO 7650             // first value for steer zero position ADS: 11000 for EPS32 AD PIN: 2048

// IO pins ------------------------------------------------------------------
  // set to 255 for unused !!!!! ==> PIN_UNDEFINED
  
  //I2C
  #define SDA_PIN 21;                            // I2C Pins
  #define SCL_PIN 22;

  //LEDS
  #define AUTOSTEER_LED_PIN 33                  // light on active autosteer and IBT2
  #define LED_WIFI_PIN 32                       // light on WiFi connected, flashes on searching Network. If GPIO 0 is used LED must be activ LOW otherwise ESP won't boot
  #define LED_WIFI_ON_LEVEL HIGH                // HIGH = LED on high, LOW = LED on low

  //RELAYS
  #define RELAY_PINS {PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED} // (not supported at the moment) relais for section control
  #define TRAM_PINS {PIN_UNDEFINED,PIN_UNDEFINED,PIN_UNDEFINED} // (not supported at the moment) relais for tramline control
  #define RELAYS_ON HIGH                        // HIGH = Relay on high, LOW = Relay on low

  //WAS
  #define LOCAL_WAS_PIN PIN_UNDEFINED           // PIN for Wheel Angle Sensor (none, if ADS used)
  #define LOCAL_WAS_DIFF_GND_PIN PIN_UNDEFINED
  
  //SWITCHES
  #define WORK_SW_PIN 34                        // PIN for workswitch (can be analog or on/off switch see WorkSW_mode)
  #define STEER_SW_PIN 35                       // Pin for steer button or switch (see SteerSwitchType)

  //ENCODER
  #define ENC_A_PIN PIN_UNDEFINED               // Pin for steer encoder, to turn off autosteer if steering wheel is used
  #define ENC_B_PIN PIN_UNDEFINED               // Pin for steer encoder, to turn off autosteer if steering wheel is used

  //SERVO
  #define SERVO_PIN PIN_UNDEFINED               // (not supported at the moment) Pin for servo to pull motor to steering wheel

  //MOTOR
  #define LOCAL_PWM_PIN PIN_UNDEFINED           // PWM Output to motor controller (IBT2 or cytron)
  #define LOCAL_DIR_PIN PIN_UNDEFINED           // direction output to motor controller (IBT2 or cytron)
  #define STEPPER_DIR_PIN  26 
  #define STEPPER_STEP_PULSES_PIN  27 
  #define STEPPER_ENABLE_PIN 13
  #define STEPPER_ENABLE_SAFETY_PIN 25
  
  //CURRENT SENSOR
  #define CURRENT_SENSE_PIN PIN_UNDEFINED       // (not supported at the moment) current sensor for IBT2 to read the force needed to turn steering wheel

  //ETHERNET
  #define ETHERNET_CS_PIN PIN_UNDEFINED         // CS PIN with SPI Ethernet hardware  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5

  //CAN BUS
  #define LOCAL_CAN_RX_PIN PIN_UNDEFINED        // (not supported at the moment) CAN bus 
  #define LOCAL_CAN_TX_PIN PIN_UNDEFINED        // (not supported at the moment)

  //##########################################################################################################
  //### End of Setup Zone ####################################################################################
  //##########################################################################################################
  //filter variables set by AOG via PGN settings sentence
  #define KO 0.05f                              //overall gain  
  #define KP 20.0f                              //proportional gain  
  #define KI 0.001f                             //integral gain
  #define KD 1.0f                               //derivative gain 
  #define AOG_STEER_POSITION_ZERO 0             // keep value to 0, needed to keep WEBIO and AGIO zero function working
  #define STEER_SENSOR_COUNTS 100
  #define ROLL_CORR 200
  #define MIN_PWM 40
  #define HIGH_PWM 150
  #define LOW_PWM 60
#endif
