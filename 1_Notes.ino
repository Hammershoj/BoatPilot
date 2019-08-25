/*
 * NOTES  This tab added to J13.2 10.21.16
 * 
 * OLD NOTES MOVED FROM MAIN TAB
 * 
 * // V29 started at 25 to experiment with ways to get stable operation.  This version seems to be working,  It has the print routines
// commented out, not sure why that works,  must be a timig issue.
// V29B added compass to display on LCD
// V29C, try to handle GPS not available or route go to waypoint not available. 
//Auto pilot 4_0 is working model.
//Autopilot 4_1 adds keypad and event handler
// Autopilot 4_2 adds rudder signals from PID to operate rudder relays for n seconds less than +/- 5 from center
// A/P 4_3 will addXTE for integral error and compass rate for differential error
// A/P 4_6 Starting at 4_3, 4_6 will change  control from rudder motion to rudder position See Void PID 
// A/P 4_7 based on 4_6 update to Arduino 1.0.  elimnate type byte replace with int or char
//A/P 4_9 based on 4_7 to integrate the MinIMU9 gyro compass, also moved subroutines to TABS.
// A/P 4_10 based on 4_9 willput GPS input to separate Arduino because gps was slowing down MinIMU9.
//  testing shows separate processing will transfer needed data in 1 milli()
// A/P 10 to 13 incorporated bearing rate from gyro vertical component and reworked PID
// A/P 14 add Simulation
// AP 15 Change rudder control to eliminate use of delays. instead sets rudder command turns rudder on then updates position 
      //AP 15 (cont'd)   each cycle until rudder position  = rudder command.
// AP 16 Revise the GPS Steering mode and Simulator to test GPS steering
// AP 17 screwed up
// AP 18 -  AP 16 is working on Compass and GPS simulation but LCD display has conflicts AP is to fix that
// AP 19 working version after AP 16 
// AP 20 update keypad to switch modes 4/3/2012
// AP 21 pull out the verbose keypad with Heirarchial screens in v20 and use AutopilotKeypad v1 with each key having a specific function
// AP 22  21 was working 22 is trying to rudder control working with dodge keys
// AP 23 add Tack as Mode 3 change HTS by tack angle.  Add Magnetic variation so HDGs are True
// AP 24 makes simulation mode available from keypad (HOLD key 5)
// AP 25  add limits to rudder during TACK maneuver;
// AP 26 added UTC seconds print to LCD comes from GPS should count to 60 shows if GPS data is updating. moved get_GPS to fast loop.
// AP 26 con't also move the Easy Transfer "get_GPS() from slow loop to fast loop to read data more frequently
// AP 4_27 commented out waypoint next in Data Structure and Data recieve to stopp chrashes while looking for alternative data transfer
// AP 5_0 fixed data structure crashes and 5_0 is the go to the boat Test Model.
// AP 5_1 added heading_error_max to limit rudder for large course changes. added sw2 function as helm steering permissive switch.
// AP 5_2 Last version tested on the boat in August 2012, don't know what changes from 5_1.
// AP 5_3 Editorial Changes back home 11/2012.
// AP 5_3_1 Editorial changes
// Ap 5_3_2 Switched over to rudder position indicator for feed back instead of tracking rudder on times
// AP 5_3_3 modified rudder command structure
// AP 5_3_4 further mods to rudder command coding
// AP 5_3_5 Added data structure to receive new data for GPRMB
// AP 5_4_2 Working version with GPRMB,and good data ET Transfer, Added screen 0/1 on key 5 
// AP 5_4_4 Added UTC Timer, void Date_Time To track on LCD running time and max Dt between ETdata updates
// AP 5_4_5 added XTE control to Steer GPS and new ETdata XTE_differential _error. Compute DXTE/DT in GPS_JNE
// AP 5_4_7 Started revising simulation in 5_4_6 and 5_4_6_1.  this version is revising simulation anew from 5_4_5 
// AP 5_4_7_1 modified KeyPad 5/19/13  ** This is starting point on boat June 2013 **
// AP 5_4_7_2 installed rudder indicator on boat aand rewrote Rudder_Position() calibration
// AP 5_4_7_3  put tracking erro in separate subroutine and update it continuously.
// AP 5_4_7_4 rearranged A_P_Loop, added logic to turn rudder off after n ms to slow down response, testedin compass mode worked great
// AP 5_5 New baseline, same as 5_4_7_4 tested working version, compiled in the Knob gain for this version to lock in this working version
// AP 5_5_1 play with rudder deadband and timing
// AP 6_0 same as 5_5_1 but includes new Pololu MinIMU9 v1.2.1 software to support MinIMU9 V2 chip
// AP 6_1 same as 6_0 but change commands from Wagerner motor controller to Pololu controller i.e. digital.write(12, low) becomes Serial2.write(0xc2) etc
// AP 6_2 6_1 had a constant motor speed and rudder off after duration. 6_2 has variible rudder speed and no off time rudder modes added
// AP 6_2_1 integrating rudder control fro PWM_3 into AP and revising to accommodate input from Garmin GPSmap 198C (has no GPAPB sentence 10/6/13
// AP 6_2_2 10/10/13 restructuring GPS steering for course to steer and heading to steer
// AP 6_2_3 10/24/13 contiuning with 6..2.2 changes just want to sae 622 as a go back point.  Added smoothed bearing rate
// Note SW2 is not being used I am tking out the actuating code but there maybe some comment references I didn't get removed
// AP 6_3 add knob steering
// AP 6_4 minor tweaks
// AP-6_5 add second startup execution to reduce IMU9 gyro biasing when powered up
// AP 6_6_5_1 Modified Print NAV DATA for GPS Steering analysis, added maximum rudder angle and speed for tacking, user input, keypad4/6
// AP 6_6_5_2 changed rudder indication positive direction
// AP 6_7 renamed 6_6_5_2.  Did sea trials with reinstalled equipment and all modes working including GPS with cross track error correction. 11/22/13
// AP 6_7_1 same as above but add code to limit cross track error correction for a minute when steering to new waypoint to eliminate oversteer with big course change See GPS_Steer()
  // this version sea tested Nov 2013.All modes working.  GPS is rough in the turns goes through the waypoint then has to double back
// AP 6_7_3 same as 671 added limit on newwaypoint rudder max, modified Tack rudder max implementation.
// AP 6_7_4  changed the rudder controls so user can select between Pololu Trex Controller or Pololu Qik controller
// AP 6_7_5 added line 145 PID to set rudder position to zero in RUDDER_MODE 1
// AP 6_7_6 Added Terms and conditions shown above and added start up screen requiring acceptance
// AP 6_7_6_1  Added Serial Keypad remote
// AP 6_7_6_2 Added Easy Transfer send data to the remote,  Can now send an receive data between remote and Main AP Mega.
// AP 6_7_6_3 corrrected errors in Dodge Mode
// AP 6_7_6_4 deleted
// AP 6_7_6_5 Added the key released function to the Remote Serial KeyPad 1/7/2014
// AP 6_7_6_6  Added Print Motor Commands to print motor controller command open/ close solenoid/ L/R rudder and Rudder stop,ported Lat and Lon over 
             //  Must be used with GPS_JNE_5_5_7_2 or later
// AP B1_0  Same as 6_7_6_6 plus a switch to turn off serial remote since it screws up AP if not plugged in.  
// the B series in both AB and GPS are adopted because they must go together for easy transfer to work after I added
// transfer of Lat and Lon
// AP B1_2 started from B1_0 update for Pololu latest libraries and IMU code 2/2/14 for library nov 2013. 
    // Changed LCD print screen 3 from MAGVAR to Magnetic_Variation. (uses default or GPS input if GPRMC Fix is valid) 
// AP B1_2b added Key "0" function to include reset screen to 0  Added Next turn, deleted LAT/LON (too much data)
// AP B1_2d  add data fo easy transfer to serial remote
// AP C1 Use with C series of GPS and Serial Remote got the serial remote TFT working.  Changed how I get Easy Transfer data
      // use if(ET.receiveData()) before trying to read data.
// AP C3 Took out the stop rudder every loop to cut down inductive kickback load on motor controller
// AP C4 In GPS Steer make default on waypoint arrival to continue on course.  Auto turning to new course  is an option.
// AP D extension of C4. Added in MSG to eztransfer to send a message ID compactly to display messages on TFT
// AP D4, D5 updated MSG handling for GPS available or lost. 
// AP E same as D5 numbering to keep uniform with Serial Remote
// AP F Jack's experimental with hooks for wind steering
// Fa tested well for GPS steering. turn anticipation turned off in GPS F3
// Used with GPS F3 which uses Delta lat, Delta lon to compute XTE more accurately which cut out the lumpyness caused by XTE changing by 15 ft
// Fa2 Added Kxte integral .0005. Fa tested well with GPS G0 comming down from swinomish, now try XTE integral correction. Serial remote baud to 19200
// Fa3 same as Fa2 but testing variousKxte coefficients
// Fa4 change XTE to be based on BOD instead of BRG
// Fa5 set Xte_correction and XTE_integral_error to zero when key 0 pressed
// AP_JNE_G  Same as Fa5
// G1 constrain integral error +/- 15, varying Kxte, Kxte differentila, sign XTE differential = sign Dxte/Dt, change the Key 0 reset to Key 2 reset of XTE integral
// G1 final version summer 2014 tested in heavy weather in GPS mode crossing wind and current
// IMUv3 G1 updated code so it could use IMU V3 lbrary.  Use the New I2C tab, other tabs are the same.
// Note  This version needs to use the  L3G and LSM303 libraries from IMU_LIBRARY_2015 
// IMUv3 G2 added Bearingrate_correction as user input
// RF version attempt to transmitt the eazy transfer data via wifi
// Version IMUv3_RF24_G2_3 has the RF remote working to send data and receive keypad input
// IMUv3_RF24_G3 Saved previous working version of RF24 and added code to smooth GPS course and use in GPS steering
// G4 saved working version making changes to improve GPS, changed MSG structure to alwaways print see print1 screen 0, adding second RFDATA2
// G4c saved working RF version
// G4d deleted the serial remote call and reference to see if it was causing instability
// G4e brought in course to steer from APB and called it GPS_course_to_steer and will evaluate the effect of using it instead of course to steer based
//   my lengthy derivation of course to steer based on bearing and cross track error
// this worked fabuously with the Garmin 740s whichgenerates a cross track error corrected course to steer 9/21/15
// H1 saved the previous G4e as default version 
// H3 got Anticipated turning working
// H4 same as H3 now trying to add #define code turn turn various features on at the compile stage
// H4B  added #define IMU to set which dcalibration is used
// H4C add code for Pololu Simple Controller modify rudder control to be more general in term max min rudder speed Added parameter Clutch_Solenoid to turn off trying to 
//    write to the solenoid if it was not attached because it could have unforseen issues to motor controller if not specifically attached
// H4D  took out waypoint timer, dead reckon stuff which was replaced with anticipated turns, deleted a lot of old commented out stuff
// H4E going to add steering mode GPS2 which will be like compass steer but will capture current COG as CTS and maintain it use key hold to activate
//   In GPS2 mode the CTS is set = to the current COG it can then be incremented up and down with keys 4, 6, 7 and 9 like compass course
// J versions put #define COMPASS in to select between Pololu IMU9 and Bosch BNO055.  So far Test J2 has BNO055 heading working but does not yet have bearing rate.
// J3 added BNO055 calibration restore from EEPROM
// J6 changed RF inherited printf statements to regular print statements. Took out test designator and posted to DropBox a current version. 
// J7  House cleaning
// J8 J9 rearranged some of the user input to get important things up front, Add PWM LCD contrast control to eliminate 10K pot for contrast,  added user option to not use SW2
   //  Added #define GPS_Used 0/1 0 leaves out the easy transfer library and GPS functions though GPS variables are still defined
// J9a 3.20.16 moved BNO campass cal status to Screen 3(fourth screen) had conflict on screen 0 with rudder position
//   put in a user note to set motorspeedMIN
//  J9a saved as working model tested 3.25.16 with GPS H6.  tested GPS and compass using Pololu IMU9v3 and using CTS from Garmin GPS 740 saved as working version.
// J10 starting with J9a to work on BNO055 compass.
//  Note BNO055 compass not working it did not update rapidly and bearing rate juped around between 0 and twice the rate from Pololu IMU9v3
// J10 was tested in anticipated turn mode and worked well.
// J11 same as J10.  starting J11 so J10 is archived as the working tested version
// J11 Increase frequency of reading the compass (line 57 A_P_Loop) if counter > 8 changed to if Counter > 1 (reads every other fast loop) 
// J11 RF remote Update frequency is slow need to check. J11 Notes revised 5/14/16 prior to posting.
// J12 added Wind direction and Speed From Raymarine wind instr via MINIPlex Seatalk converter  
// ET transfer did not work with added wind data. restructure ET to convert float to int to save space.  I find ET does not work with >64 bytes, library says it will do 255. 
// J12.2 evolution of reading wind data
// J12.3 revise RF data structure from float to int with float/int conversion to provide compact data transfer with 32 byte limit
// J12.6 Wind input.  Got wind and Depth input woring from Seatalk and MiniPlex.  Added AT ANCHOR SCREEN Wind, heading, depth, wind max , Screen 4
// J12.7 First attempts with wind involved using the GPS but later went straight to AP. Had converted ETdata to integer and had some /10 to convert to float, had to remove these.
// J12.7 tested successfully with Use_CTS = 1 use with GPS H6. 
// J12.8 adding code to PID to implement wind steering as Mode 4
// J12.9 modifed GPS available line 43 A_P_Loop eliminate && GPS_was_Available, tweaked LCD Rudder display, tweaked GPS message (AP_Loop 46)
// J12.9 With CTS 0 and GPS_6 CTS 0, Anticipate Rurns on, this version worked beautifully in GPS mode As well as the Garmin GPS 740 with CTS on.
// Autopilot_JNE_J13.0 resaved the above J12.9 as an archived baseline of a good working model 8/7/16 Left the RF24 and bn055 out of title. Use with GPS_H6.
// j13.2  Added Radio.powerup to RF24 for added reliabilty of RF remote skipped J13.1 
 * Changed if PID_MODE == to #define, #if, #endif structure main tab and PID tab, saved J13.2.A
 * J13.2A Added RUDDER_OFFSET 10.21.16
 *     Rudder offset - When sailing to windward with significant weather helm when shifting to AP steering a large heading error is needed to keep 
 *     the weather helm.  Using rudder offset should enable user to steer a course and have the AP hold that course when key 1,3 pressed without
 *     needing to key in a large course offset to hold that amount of rudder. Using RUDDER_OFFSET 1 will enable this feature, 0 will not. When used 
 *     keys 1 and 3 (maybe 2) will capture rudder position as Rudder_Offset  and add it to PID_Output to obtain Rudder_Command.  This is implemented
 *     this way to facilitate testing. Also if PID_MODE == 3 initial integral error == rudder offset. Use of rudder offset controlled in user input
 *     
 * J13.2A2  Commented out main tab #if Wind_Input ... #endif because if set to zero there were  a bunch of undefined variables.
 * J13.2.A3  Implemented Proportional rudder force. At bigger rudder positions it takes more force to increase rudder.  see PID void Rudder_Control
 * J14.2  Modified code to allow use of Pololu IMU V5  swapped out I2C tab and added user input for V5 and use of #define IMU_V5
 * J14.2B changed Gyro fullscale from 2000dps to 245 dps and gain from .07 dps/digit to .00875 see main tab lines 366- 370 and I2C. 4/27/17
 * J14.2C added multiple calls to receive RF data to improve reliability
 * J14.2D Starting with J14.2B  added intrrupt structure to RF to receive data based on an interrupt as opposed to multiple insertions of calling recvdata
 * J14.2E Added radio.setPALevel(RF_24_MAX), radio.setDataRate(RF24_ 250_KBPS), radio.setChannel(108)
 * This is buiid 5/14/17
 * J14.3 Saved as archived version 6-4-17 settings worked well sailing and yeterday Bellingham to Sucia GPS steering faultlessly complex route 18 miles
 * J14.3A and B are variations of gyro full scal and sensitivity settings for testing
 * J14.4 Starting from J 14.3. Change #define IMU from 2, 3 and 5 to 93 IMU9V3, 103 IMU10 V3, 51 IMU9 V5 #1 and 52 IMU( V5 #2 allows testing of various compasses
 *   and variations on calibration data sets.
 * J14.4barometer_test added code to support Pololu barometer and temperature sensor for AltMinImu-10 v3 and v5 boards. press and temp readout on sceeen 4 (wind screen)  
 * J14.5barometer added Wind_Steer_Direct and GPS_Steer_Direct see user input
 * J14.7  9.22.17 Version 14.6 added code to allow GPS2 steering when waypoint = no waypoint, one line added, see PID GPS Steering.
 *    14.6 was my end of summer  August 17 tested code but doing some testing today I inadvertently saved some changes to 14.6 i did not want to keep
 *    and I was not certain I got them all deleted.  So i went back to 14.5barometer and added the gps changes from 14.6 to give me what i think is 
 *    the same as the 14.6 I tested this summer.  Next I added two lines of code to the top of PID to read rudder position and stop rudder if
 *    greater than Maxuimum_Rudder. Kevin found in dodge mode the rudder position was not updating and would run into the stops.
 *    
 *    NEW CODE Starting with J14.7 modified code to run on one board either Arduino or Teensy.  To do this the GPS GET_Sentence was restructured to eliminate the 
 *    while(){ funtion and replace with a Serial_GPS read each cycle until carriage return detected. This eliminates the wait time for incoming NMEA data and runs 
 *    fast enough to include on main AP Mega.   Same code can then run on a Teensy
 *    Rewired my KEYPAD to agree with Fritz 6C and D
 *    
 *    AP_JNE_Teensy_3.5 works on a single Mega with RF24.  Works on Teensy 3.6 But with RF24 off
 *    3.7 works with RF if I pull the TFT MISO pin.  Does not accept key input from TFT or Remote.
 *    3.8 saved 3.7 Removed the TFT and modified code to allow TFT_Used parameter.  Added LCD-I2C and got it to work including RF24 remote key functions LCD works 
 *    same as on  Mega model.  I am getting some lock up and will try LCD without the IIC module poor wire connections seem to be a problem also
 *    3.9 Deleted dummy GPS data saved to preserve 3.8 before changes
 *      Got Teensy Keypad working 
 *      3.10 added lcd.init() and LCD() to RF24 Recv_data to refresh Serial LCD which somtimes gets scrambled when RF interrupt receives data
 *      3.11 added LCD traditionl wiring to TEensy with Pin declarations.
 *      3.12 Experimenting with XTE calc
 *      3.13
 *      start back at 3.11 added Additional coding from GPS H6Bv5 to bring in anticipated turn and try to get this version to include GPS functionallity of old
 *      two board GPS system. changed feet per mile from 6000 to 60761. and degrees_to_feet = 364566.
 *   3.14 rework XTE
 *   found errors in accuracy of Lat and Lon. found pased lat lon had round off with less precision than 4 decimals of NMEA lat lon input.
 *   For the Teensy fixed by using double instead of float. not an option for Arduino Mega 
 *   3.15 Changed Index test for current waypoint in XTE - Destination_Bearing() 4/24/18. Deleted differentiation of RF data for teensy or Arduino
 *   3.16 Exrensive mod to GPS steering to delete PID GPS and go to GArmin style Course to steer and correction. Disabled GPS2 see GPS2 in keypad
 *   3.16d revised how antivipate turns is indexed tested and seems to work Waypoint_next on LCD does not increment when anticipate turns indexes.
 *   3.16e kept Waypoint_next as the GPS read in data but made Active_waypoint the go to waypoint that is indexed or set to NO WPT etc for display and send to RF24
 *   3.16.f 3.16e is working wrt GPS XTE and Anticipate turns. 3.16f to preserve 3.16.e. in f modifying dta sent to RF remote trying to get it work correctly
 *   3.17 Tested 3.16f on Arduino and Mega working on both. New baseline
 *   3.17a Uncommented Wind and cleared all obvious variable redefinitions.  Code working wind not tested until i get to the boat set wind_buffer size to 32
 *   Changed name to Autopilot_JNE_15.0 5/12/2018
 *   JNE_15.1, 2 and 3 experimental with wind ended up messing up GPS will delete
 *   JNE_15.0A new thread to work on wind Tested 15.0A 6/5/18 for GPS steering (not CTS) seemed to work fine but Use CTS did not, GPS_course_to_steer
 *    was not modified for single board.  
 *   JNE_15.0B Changed GAPB to use varible GPS_course_to_steer. 
 *   15.0C added wind code from 15.3 and added sw2 can be used to reset Wind_MAX
 *   15.0D Made code so it would compile and run with GPS_Used = 0.  need to move a number of variables outside the #if GPS_Used ... #endif loop
 *   15.0E implementing and testing integral error PID coefficients float PID_Ks[4] = {2, .4, 2, .0005}; worked pretty well using 
 *   .01 for integral was a lot of overshooting. tested with GPS steering using CTS 1.  Tested in COMP mode but not sailing with weather helm Also Wind was off
 *   as it is causing some data problems which result in spurious steering action.
 *   15.0F Saved 15.0E as archived version. added if (!Use_CTS) to LCD code so it prints waypoint next for CTS = 1 or Active Waypoint for CTS = 0
 *   also if Use_CTS = 0 XTE not displayed.
 *   15.0F 6/19/2018 tested lasst couple days PID integral term used set at .0005. things are working great GPS steering with CTS 1 and CTS 0 both 
 *   working great.  Both modes held cross track error generally less than 10 feet.  Tested compass mode sailing and it would pull the HDG and HTS
 *   right together.  Still have a data issue with the wind so this testing is with wind off. 
 *   15.0G to archive 15.0F tweak Rudder Offset add bearingrate offset.
 *   bearingrate_Offset captured with key 1, 3, and 2 reset to 0 with key 0. applied to bearingrate in void Bearing_Rate()
 *   15.0H Wind steering code had been commented out to get one Mega code working, 15.0H is reimplementing it.
 *   15.0J no I code, revise wind serial read to be like GPS to eliminate while loop. also revise how active waypoint handled for 
 *   CTS 1, #if Use_ CTS == 0 Sub G_XTE and Active_waypoint indexing code in AP_Loop 
 *   15.0K  In CTS 0 AP steers opposite direction for a momentworking to fix. had two calls to Get XTE. deleted extraneous comments
 *   15.0L  Changed the way the wind data is processed so it is like the GPS data instead of the way GPS was down when two boards used.
 *   15.0L2 added line in AP_Loop to not Get_Wind when GPS steering trying to eliminate wind code intermittent reset of CTS when GPS steering.
 */
