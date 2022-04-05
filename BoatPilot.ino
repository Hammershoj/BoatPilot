/*****************************************************************************
TERMS AND CONDITIONS

THE USER OF THIS SOFTWARE, WHICH IS MADE AVAILABLE FREE, ACCEPTS ALL RESPOSIBILTY ASSOCIATED WITH ITS USE.
SOFTWRE ERRORS OR HARWARE PROBLEMS CAN CAUSE THE BOAT TO MAKE SUDDEN TURNS OR FAIL TO MAKE EXPECTED TURNS.
THE BOAT OPERATOR MUST HAVE A POSITIVE MEANS TO REGAIN MANUAL CONTROL IF A PROBLEM OCCURS.   UNDER CONDITIONS 
WHERE A SUDDEN TURN OR LACK THERE COULD CAUSE A HAZARD  THE SOFTWARE SHOULD NOT BE 
LEFT TO CONTROL THE BOAT WITHOUT THE OPERATOR BEING PRESENT. 

This software wa developed as an experiment by the author to provide an auto pilot for his boat.  It is made available
to others who want to develop a Do It Yourself autopilot.  The USER acepts all responsibilty for the safe operation of
the boat recognizing that hardware and software erros can occur.  The User also acknowledges that it is their responsibily to
safely wire and install the autopilot components in accordance wire appropriate codes and standards.

THIS AUTOPILOT SOFTWARE AND ASSOCIATED HARDWARE DESCRIPTIONS IS MADE AVAILABE UNDER CREATIVE
COMMONS LICENSING

*****************************************************************************************************************************/

#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include <SPI.h>
#include <Adafruit_GFX.h>      // Graphics library used with OLED
#include <Adafruit_SSD1306.h>  // OLED

#include <ClickEncoder.h>
#include <TimerOne.h>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/******       USER INPUT       ********/

#define Compass 1 //  0 for Pololu, 1 for BNO055, 2 for BNO080
#define IMU 52 // Used to select Pololu IMUs Versions and calibration set. 
    //Allowed values: 2 IMU9 V2; 93 (jacksIMU9V3); 103 (jacks IMU10V3; 51 (Jacks IMU9V5 #1); 52 (Jacks IMU9 V5 #2)
    //determines which set of calibration data is used these extra versions were added to code 7/11/17 J14.4
    // see code lines this tab about 390 to 438 to enter your calibration data
#define GPS_Used 1 // 1 to include GPS code, 0 to exclude it
#define Motor_Controller 4  // 1 for Pololu Qik dual controller, 2 for Pololu Trex dual controller, 3 for Pololu Simple Controller, cfh 09.06.2019 mode 4 for dual relay controller + solenoid clutch
#define Clutch_Solenoid 1 // 1 a clutch solenoid is used, 0 is not used
int RUDDER_MODE = 0; // 0 uses rudder position, 1 does not   // cfh 15.06.2019 changed to variable and not predefined const
boolean Change_rudder_mode = false;  // cfh 15.06.2019 added to allow for user input to change RUDDER_MODE
#define RF24_Attached 0 // 0 if RF 24 radio modules are not attached, 1 if they are used
#define Wind_Input 1 // 1 to use NMEA wind data. 0 to not use wind data

#define RUDDER_OFFSET 1 // 1 uses rudder offset, 0 does not
#define BEARINGRATE_OFFSET 1 // 1 to use 0 to not use
 // float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output;
 float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
 #define PID_MODE 0 // See description PID tab.
 boolean Accept_Terms = 0; //  1 user must accept terms to run.  0 user accepts terms by setting this value to 0
 boolean just_started = 0; // to do a second setup so get a better gyro startup
// cfh 09.07.19 uncommented
 //float Kxte[3] = {0, 0, 0}; // used for XTE PID, use this to zero out XTE tracking
 float Kxte[3] = {.2, 0, 0}; // {.2, 4, .0004} baseline; {.05, .5, .0005}last used;  0 is proportional, 1 is differential, 2 is integral error, see GPS_Steer() in PID
                        // .36 will give 45 deg correction at 120 ft XTE to emulate my Garmin GPSMAP 740s see PID tab, voidActual_Gps_Steering()
 float K_course_avg = .999; //used to smooth gps course in PID/ void Actual_GPS_Steering().999 will smooth over 1000 points
 
 //---------------------------------------------- Rudder values and variables ------------------------------------------------------------------------------------------------
  
 float Maximum_Rudder = 18; // Maximum rudder angle in degrees
// set rudder variables as global
 float counts_at_zero = 0;
 float counts_min = counts_at_zero -100;  //Right
 float counts_max = counts_at_zero +100;  // Left   from calibration in print statement
 float counts;
 // rotary encoder direction
 boolean rudder_right = false;
 boolean rudder_left = false;
 //boolean middle = false;

ClickEncoder *rudder_encoder;
int16_t last_rudder_encoder_value, rudder_encoder_value;

//----------------------------------------------- Connection pins for motor and solenoid steering on Midtholm Maxi 120
// cfh 09.06.2019 defined motor control output pins added
int relay_Turn_rudder_left = 10;  // pin 9 or pin 12 open relay engaging motor to the left
int relay_Turn_rudder_right = 9; // pin 8 or pin 11 open relay engaging motor to the right
int relay_Engage_solenoid = 8; // pin 10 or pin 10 open relay engaginng solenoid engaging gear
 // end cfh
 
 // User set motorspeedMIN around lines 359, 360, 371 for your controller type and rudder steering motor Use crtl-F to find motorspeedMIN 
 float Tack_Angle = 100;  // angle throug which boat will tack when tack L or R pressed (keys 4 and 6 in TACK mode(3)
 int Tack_rudder_MAX = 32;// limits rudder so it doesn't slow boat too much,  need to tune
 float Tack_rudder_speed = .5; // rudder speed during tack , value * full speed, will use min of tack speed and regular speed, user adjust
 float Rudder_Offset = 0; // see notes 10.21.16
 float bearingrate_Offset = 0;
 float MagVar_default = 0.5;// 2.5 Avg west coast NO STVG  User should keep up to date for location.  Pgm will use GPS value if available + east, - west

 boolean Serial_Remote_Is_Connected = 0  ; // 1 remote connected, 0 remote not connected Pgm hangs up if remote data is sent and remote is not connected
 boolean GPS_Auto_Steer = 1; // 0 will cause GPS steering to go to compass mode and maintain heading if waypoint reached. 
                             //  1 will cause GPS to automatically steer to next waypoint.  NOTE THIS CAN BE DANGEROUS
 boolean Use_CTS = 0  ;// 0 no, 1 yes;  if the GPS generates a GPS course to steer and you want to use it instead of calculating course to steer from BOD and XTE                            
 //float bearingrate_correction = 0.0; //use to get avg stationary bearing rate to read 0 on screen 2

 #define SW2_Used 0 // 0 if SW not used. 1 if SW2 used, must be 1 to use SW2 on wired remote
 #define UseBarometer 0 // 1 to use, 0 to not use, this works with Pololu Alt IMU-10 v3 and v5 Prints barometric pressure and temperature on screen 4, measured at the compass 
 #define Wind_Steer_Direct 0 /* 0 should be used for default which uses wind-error or GPS_error to compute heading_error but then all steering is based on compass steering.
                              If set to 1 the PID will use the Wind error or the GPS course error directly without using the compass (compass still needed for bearing rate)
                             see lines around PID 45 for implementation.  This change implemented 8/18/17 version J14.4 barometer 
                             */
 #define GPS_Steer_Direct 0 // same as above
 #define TFT_Used 0 //  1 to use tft
 //#define Screen 0
 
 //--------------------------------------- debug print settings --------------------------------------------------------------------------------------------------------------------
 int print_level_max = 3;  //  0 to 4, used to set how much serial monitor detail to print
 // 0 = none, 1=PID Output, 2 Adds parsed GPS data, 3 adds raw GPS input, 4 adds checksum results
 int print_time =5;  // print interval in sec
 boolean Print_ETdata = 0; //prints GPS incoming data turn this off to see actual loop time
 boolean Print_ETtime = 0;  // prints Easy Transfer loop time 1 on 0 off
 boolean Print_heading  = 0 ; // diagnostic serial print heading, interval set in A_P_loop
 boolean Print_integral = 0 ; // diagnostic serial print debug info for integral steering  (PID tab)
 boolean Print_LCD_IMU9 = 0;  //prints Head, Pitch, Roll, Yaw on LCD conflicts with other LCD Prints
 boolean Print_LCD_AP = 1; // prints main A/P LCD output data and Menus, only do one of these at a time
 boolean Print_Gyro = 0; //  Prints LCD and Serial scaled X(roll), Y(pitch), Z(Yaw) deg/sec
 boolean Print_PID = 0; // 
 boolean Print_UTC = 0;
 boolean print_Nav_Data = 0; // Print_1 Tab
 boolean Print_Motor_Commands = 0;  // prints rudder commands in PID tab
 boolean Print_Rudder_Commands = 0;  // prints rudder commands in PID tab
 boolean Print_Anticpate_Turn = 0;  // prints data from void Actual_GPS_Steering to evaluate Anticipate turn function
 int print_level=print_level_max;
//  print modes for MinIMU9
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1
#define PRINT_DATA 0 // 1 to print serial data or run python display
#define PRINT_EULER 0  //Will print the Euler angles Roll, Pitch and Yaw, needed for python display
//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data

 //*************  GPS USERS INPUT  *******************

#if GPS_Used == 1  
  #define GPS_source 1  //  1 means GPS is done onboard main AP board, 2 means a separate board is used and data read in with Easy Transfer 
  #define Serial_GPS Serial1 
  static const uint32_t GPSBaud = 9600; // cfh 15.06.2019 
 
  int Input_Source = 4; 
     // 1 = Garmin GPS60CSX(has $GPRMC & $GPAPB // 
     // 2 = Nobeltec (missing Last term of $GPRMC and $GPAPB for GPS status. note seems to work with GPS 60CSX in Source 2
     // 3 = same as 1 may need to change 198 c does not have APB but does have BOD
     // 4 = TinyGPS  added by cfh 15.06,2019
   #define LatLon_decimals 0 // Use print_NEMA to determine number of decimals NOTE IF SET TO ZERO CODE WILL AUTO DETECT SET VALUE MANUALLY IF HAVING TROUBLE

   float  Turn_distance = 200;  //  Anticipate turn distance in feet. for compute from how long to turn 360 deg and boat speed  =  traveled/ 2 Pi
   boolean Anticipate_Turns = 1; // 1 yes, 0 no turns off turn anticpation
  //   boolean Use_CTS = 0;  //  1 to use the CTS (course to steer) from GPS sentence $GPAPB. turn Anticipate Turns off if GPS has this feature see video
   boolean print_NEMA = 0;  //1 = print, 0 = not print, printing byte by byte input
   boolean print_GPS_buffer = 0;  //1 = print, 0 = not print, looks the same as NEMA but is the parsed incomimg sentence by sentence
   boolean print_RMC = 0 ;  //1 = print, 0 = not print
   boolean print_APB = 0; //1 = print, 0 = not print
   boolean print_RMB = 0; //1 = print, 0 = not print
   boolean print_BOD = 0; //1 = print, 0 = not print
   boolean print_WPL = 0;  //1 = print, 0 = not print
   boolean print_RTE = 0;  //1 = print, 0 = not print
   boolean print_timing = 0;
   boolean print_checksum = 0;
   boolean print_ETdata = 0;

//**********  GPS VARIABLES  ************
   int oldtime = 0;
   int newtime= 0;
   int deltatime=0;
   int byteGPS=-1;
   const int buffer_length = 84;//chnged from 200 oct-15-15
   char gps_buffer[buffer_length] = "";
   int bufpos = 0;
   String data_APB[17];
   String data_RMB[17];
   String data_RMC[17];
   String data_BOD[7];
   String data_WPL[6];
   String data_RTE[17];          
   int GPS_WPT_index = 0;
   boolean Anticipated_Turn_Active = 0;
   String Route[20];
   double Waypoint_Lat [20];  //note intend to store waypoint data here with same index as Route[20]
   double Waypoint_Lon [20];
   double Waypoint_Range_From[20]; // range from previous waypoint to this waypoint 
   long Number_of_sentences = 0;
   int Current_sentence_number = 0;
   int Number_of_waypoints;
   int word_count; // this is used in A_GPS
   int word_count_temp;
   int Word_count= 0; //This word count used in RTE
   boolean Route_Completeness = 0; //GPRTE reports data as c - complete, or w for working see NMEA standard
         // for c all waypoints are sent.  For w the previous waypoint, the destination wpl and the rest of the wpls is reported
         //  code only currently treats complete routes
   unsigned long RTE_timer;
   boolean RTE_Active = 0;   
   unsigned  long temp = 0;  
   unsigned long temp2 = 0;  ; 
   String temp4 = "";
   String temp5 ="";
   char char1;
   char char2;
   char char3;
   String string3;
   unsigned long long2;
   unsigned long long3;
   int int1;
   //boolean GPS_available= false;
  String GPScase = "";
  String GPSheader = "";
  boolean NEMA_sentence = false;
  String GPRMC_fix_status = "";
  boolean GPAPB_fix = false;
  String GPAPB_fix_status = "";
  boolean GPRMB_fix= false;
  String GPRMB_fix_status = "";
  boolean NewData = false;
  unsigned  long Waypoint_Age; // Waypoint_Age_Old;
  String Lat_Lon;
  double Lat_Lon_Deg;
  String CTS_MorT;
  float SOG;
  String XTE_LR;
  String XTE_unit;
  String Nullstring;
  String Waypoint_next; 
  double Lat_current;
  double Lon_current;
  double Lat_destination;
  double Lon_destination;
  String Origin_Waypoint;
  String Waypoint_name;
  double Lat_Waypoint;
  double Lon_Waypoint;
  double Current_Waypoint_Lat;
  double Current_Waypoint_Lon;
  long Waypoint= 0;
  long Latitude = 0;
  String BOD_MorT = "";
  double Active_Bearing_to_destination;
  String BTD_MorT = "";
  double Range_Destination_by_LatLon;
  double Active_Range_Destination;
  float Velocity_towards_destination;
  String UTC_string;
  long Date;
  double Range_and_Bearing[2];
  boolean Detect_LatLon_decimals = 0; // used in Sub void To_Degrees(String Lat_Lon) to only auto detect number of decimals once
  float Time_decimal;  // supports wind instrument
  float Time_decimal_old;
  float Time_decimal_delta;
  const float degrees_to_feet = 364566; // feet per degree of lat (lon at equator)
#endif  // endif GPS_USed == 1

// GPS parameters needed elsewhere (wind and LCD)
int PT_old = 0; // for pritnt timer
String GPS_status= "NO GPS";
float Avg_course; 
float CTS_GPS2;
float GPS_course_to_steer;
float course_error;
float tracking_error;
float AVG_tracking_error;
float XTE_integral_error;
float XTE_course_correction;
unsigned long UTC_timer;
unsigned long UTC_timer_old;
boolean GPS_Available = 1;
boolean GPS_Was_Available = 0;
int MSG = 0; // message to send number for message display on serial remote
 int j_MAX = 0;
double float1;
double float2;
double float3;
char *brkb, *pEnd;
String string1;
String string2;
unsigned  long long1;
char char_buf[16];
float course; 
long checksum_received = 0; // or static byte
int checksum = 0;
boolean checksum_status = false;
String data_IN[17];
String Active_waypoint;
float XTE;
//float XTE_differential_error;
boolean GPRMC_fix = false;
double Bearing_to_destination = 0;
float course_to_steer;
double Bearing_origin_to_destination = 0; 
unsigned long UTC; 
float NEXT_TURN;
float Next_Turn[20];
double Range_Destination;
double Bearing_to_destination_by_LatLon = 0;
int WPT_index = 0;
double Waypoint_Bearing_From[20]; // beraing from previous waypoint to this waypoint 
float MagVar; //Magnetic Variation E is plus, W is minus 
  
//******  COMPASS  ***************

boolean compass_connected = true;
#if Compass == 0
 #include <L3G.h>  // get library from Pololu http://www.pololu.com/product/1268
 #include <LSM303.h>  // get library from Pololu, use version that corresponds to your version of the IMU
 compass_connected = false;
#endif

#if Compass == 1  // BNO055 compass
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 Adafruit_BNO055 bno055 = Adafruit_BNO055(55);
 #define BNO055_SAMPLERATE_DELAY_MS (100)
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  bool DataStored = false;
#endif

#if Compass ==2  // BNO08x compass using BNO08x.ino file
/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the parts of the magnetometer.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 bno08x;

#endif

int bnoCAL_status =0;  //used to send 4 digit cal status to RF remotes

#if UseBarometer
#include <LPS.h>
LPS ps;
  float pressure;
  float altitude;
  float temperature;
#endif

// LCD library code:
//LiquidCrystal lcd(27,28,29,30,31,32); // for traditional LCD wiring
#include <LiquidCrystal_I2C.h> // for LCD-I2c Serial
LiquidCrystal_I2C lcd(0x27,20,4); //Addr: 0x27, 20 chars & 4 lines, for serial LCD

//  KEYPAD SETUP
const byte ROWS = 4; //four rows
const byte COLS = 3; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {23,25,27,29}; //Use these with Fritz Diagram 6C and 6d
byte colPins[COLS] = {31,33,35}; //Use these with Fritz Diagram 6C and 6D
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
char key = 0;
boolean toggle = false;

// MOTOR / RUDDER CONTROL SETUP PARAMETERS 
#define Serial_MotorControl Serial2
 
 //const int motorspeed = 32; // 0 t0 127  // use with pololu motor controller

 //Servo myservo;  // used to operate a servo to simulate the rudder
 float Magnetic_Variation; 
 float heading_error =0;
 float differential_error = 0;
 float integral_error = 0; 
 float deadband;  // steering deadband
 float rudder_position = 0;
 float rudder_command = 0;
 float rudder_error;
 int motorspeed;  // sets rudder motor speed in Pololu controller can be constant or variable
 int rudder_MAX; //allows rudder to be changed for various maneuvers like TACK
 boolean TACK_ON = false;  // goes on for a tack and off when within 10 deg final course
// float rudder_total_time;
 float bearingrate=0;
 float bearingrate_smoothed;
 float bearingrate2;
 unsigned long bearingrate2_time;
 float heading_old;
 float delta_heading;
 long delta_compass_time; // computed in compass tab, used in PID for integral error
 float PID_output = 0; 
// float GPS_PID = 0;
 boolean GPS_Steering = false;
 int Steering_Mode = 0;
 String Mode = "OFF";
 String Previous_Mode;
 boolean Steering = false;  // boolean to automatic steering is on or off independant of Steering mode
 boolean sw1_turned_on = false;
 boolean sw1 = false;
 boolean sw2 = false;
 boolean sw1Status = 1;
 boolean DODGE_MODE = false;
 int Screen=0;
 boolean rudder_on;
 boolean rudder_was_off;
 unsigned long rudder_time_old;
  
  #if Motor_Controller == 1  //  Motor controll parameters, 1 for Pololu Qik Dual controller
   int Motor_0_fwd = 137;
   int Motor_0_rev = 139;
   int Motor_1_fwd = 141; //change these to reverse left and right rudder
   int Motor_1_rev = 143;
   int motorspeedMIN = 30; // this value is the minimum speed sent to controller if left or right rudder is commanded
                            //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                            //  moves at a noticable but slow speed.  Higher values will be more responsive.  
     int  motorspeedMAX = 127;
 #endif
 
 #if Motor_Controller == 2  //  Motor controll parameters, 2 for Pololu Trex Dual controller
   int Motor_0_fwd = 202;
   int Motor_0_rev = 201; 
   int Motor_1_fwd = 194; //change these to reverse left and right rudder
   int Motor_1_rev = 193;
   int motorspeedMIN = 30; // this value is the minimum speed sent to controller if left or right rudder is commanded
                            //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                            //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 127;
 #endif
 
  #if Motor_Controller == 3 // Pololu Simple controller
  // int Motor_0_fwd = 202; // motor 0 is the clutch solenoid
  // int Motor_0_rev = 201; 
   int Motor_1_fwd = 133; // change these to reverse left and right rudder
   int Motor_1_rev = 134;
   int motorspeedMIN = 555; // this value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 3200;
 #endif

 // cfh 09.06.2019 added this code for relay motor controller
 #if Motor_Controller == 4// Pololu Simple controller
  // int Motor_0_fwd = 202; // motor 0 is the clutch solenoid
  // int Motor_0_rev = 201; 
   int Motor_1_fwd = 133; // change these to reverse left and right rudder
   int Motor_1_rev = 134;
   int motorspeedMIN = 555; // this value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 3200;
   // NEw IBT-2 setup 19.04.2021
   int MOTORSPEED_PIN = 0; // center pin of the potentiometer
 
   int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
   int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
 #endif
// end cfh
#define SoftSerial1 Serial2 
#if Wind_Input == 1
  //#define SoftSerial1 Serial2 
  static const uint32_t NMEABaud = 4800; // cfh 15.06.2019 
 //#include <SoftwareSerial.h>
 //SoftwareSerial SoftSerial1 =  SoftwareSerial(17, 16); // RX, TX 
 int SoftSerial1_Bytes;
 byte byteWind;
 int count_b;
 String Windheader;
 const int wind_buffer_length = 32;
 char Wind_buffer[wind_buffer_length]; 
 String data_MWV[6];
#endif
 // wind variables defined outside #if Wind_Input to compile without complexity in LCD tab
 float Wind_Dir;
 float Wind_Speed;
 float wind_to_steer;
 float wind_error;
 float Wind_Avg;
 float Wind_MAX;
 float Depth = 0;
 
/***********  COMPASS paramters SET UP  **************************************/
float heading;
float heading_to_steer=0; //  see PID
float MAG_Heading_Degrees; //LCD_compass tab

//float system_cal; float gyro_cal;  float accel_cal; float mag_cal;
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
unsigned int counter=0;
unsigned int counter2=0;
unsigned long counter3=0; // used for 10 minute print interval in A_P_Loop
float roll;
float pitch;
float yaw;

 //*******  SETUP    SETUP   SETUP   ***** 

void setup() {
  //delay(1000); // give chip some warmup on powering up
  
  // **** Initialize serial communication and set baud rate ****************/
  Serial.begin(9600); // Serial conection to Serial Monitor
  //Serial1.begin(57600); //Communication to Serial Remote
  Serial_GPS.begin(9600);
  Serial_MotorControl.begin(19200); //Serial output for the Pololu Motor Controller 
 
  pinMode(48, INPUT); //SW1
  pinMode(46, INPUT); //SW2
  // cfh 09.06.2019 added changed motor controller using relays
  pinMode(relay_Turn_rudder_left, OUTPUT);
  pinMode(relay_Turn_rudder_right, OUTPUT);
  pinMode(relay_Engage_solenoid, OUTPUT);
  digitalWrite(relay_Turn_rudder_left, HIGH);
  digitalWrite(relay_Turn_rudder_right, HIGH);
  digitalWrite(relay_Engage_solenoid, HIGH);
  Serial.print("Rudder mode: ");Serial.println(RUDDER_MODE);
  //float Stored_rudder_mode; 
  //EEPROM.get(1,Stored_rudder_mode);
  //Serial.print("Stored rudder mode: "); Serial.println(Stored_rudder_mode);
  //#if (Stored_rudder_mode == 0 || Stored_rudder_mode == 1)
  //  RUDDER_MODE = Stored_rudder_mode;
  //  Serial.print("rudder mode: "); Serial.println(Stored_rudder_mode);
  //#endif
  // end cfh add
  // rudder rotary encoder init
  
  rudder_encoder = new ClickEncoder(A3, A4, A5);
  rudder_encoder->setAccelerationEnabled(false);
   
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  last_rudder_encoder_value = rudder_encoder->getValue();
  
  // IBT-2 motorcontroller setup 20.04.2021
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
     
 #if Wind_Input == 1
   pinMode(16,INPUT);
   SoftSerial1.begin(NMEABaud);
   if (SoftSerial1.available()) {
     Serial.print("Wind: "); Serial.println(INPUT);
   }
   else {
     Serial.println("No NMEA serial connection available ");
   }
 #endif
  //lcd.begin(20,4); // regular LCD
  lcd.begin(); // for LCD_I2C
  lcd.setBacklight(1); // for LCD_I2C
  lcd.setCursor(0, 0);

  keypad.addEventListener(keypadEvent); //add an event listener for this keypad 

  Steering_Mode = 0; // Steering mode is default off
  Mode = "OFF"; 
  Steering = false;  // default automatic is off at start up and reset
  

  if(Motor_Controller == 1) Serial_MotorControl.write(170); // sends packet that is detected for auto baud rate detection and starts normal operation not need for Trex
  if(Motor_Controller == 3)  // ditto for Pololu Simple controller
  {
  Serial_MotorControl.write(170);
  Serial_MotorControl.write(131);
  }
  if(Motor_Controller == 4) Serial_MotorControl.write(170); // cfh 09.06.2019 added, but not in use
 
  #if Compass > 1
    Init_Compass();
  #endif  // Compass == 1


  #if UseBarometer
  Init_Barometer();
  #endif

  // Initialize OLED display 
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  else {
    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds
    // Clear the buffer
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.print("BNO055 detected");
    display.display();
    delay(500);  
  }
  // Clear LCD start up info
  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print("                    ");

  // read a byte from the current address of the EEPROM
  byte value = EEPROM.read(eeAddress);

  Serial.print(eeAddress);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();
 }  // end setup
  
/*********************************************/
/* main loop */
/*********************************************/

void loop()
{
   A_P_Loop(); // Autopilot Loop
   KEYPAD();
 }    

/*********************************************/
/* end main loop */
/*********************************************/

/***********************************************/
   void Terms_and_Conditions()
   { lcd.setCursor(0,0);
     lcd.print("I Accept the Terms  ");
     lcd.setCursor(0,1);
     lcd.print("and Conditions      ");
     lcd.setCursor(0,2);
     lcd.print("Press 0             ");
     lcd.setCursor(0,4);
     lcd.print("to Accept           ");
     delay (100);
   }  // end accept terms and conditions

/********************************************/
/* NMEA parser                              */
/********************************************/
void ParseNMEA()
{
  
}
