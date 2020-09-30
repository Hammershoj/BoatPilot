 /*******************************

PID TAB is the PID calculator and it computes and sends rudder control signals

*******************************/
/******************************
PID_MODE
 MODE 0: rudder_command = PID_output with no integral_error term    
 MODE 1: rudder_command = rudder_command + PID_output.  This is a form of integral control. Was used successfully summer 2012
 MODE 3: rudder_command = PID_output and includes integral error.   x
RUDDER_MODE 
  Rudder mode determines which mode of PID control and rudder control are used.
  MODE 0: Rudder controlled to specified rudder_position     x
  MODE 1: Rudder keeps moving until rudder_command < deadband


***********************************/


        void Steer_PID()
        {  
          deadband = 5.0;  //cfh org value 2.0

         RUDDER_POSITION(); // 9.22.17 added to update rudder position and stop rudder in Dodge Mode but good for all modes in V14.7
         if(abs(rudder_position) > Maximum_Rudder) {
            Rudder_Stop();
            Key0_Pressed();   // turning off rudder to avoid damage !
         }
          
        if(!DODGE_MODE)
        { 
           // if keypad "1" was pushed Steering_Mode = 1 (compass steer) and heading_to_steer was set to the then current heading  
           MSG = 0; // null
     #if GPS_Used      
           if (Steering_Mode == 2 || Steering_Mode == 22) GPS_Steer();  // get compass heading to steer based on GPS Route and cross track error    
               // adjusts gyro heading_to_steer so GPS course = GPS course_to_steer, 
     #endif      
  
     #if Wind_Input == 1
         if(Steering_Mode == 4){  //  wind steering,  actually steers compass course where heading error = wind error         
            wind_error = wind_to_steer - Wind_Avg; // Wind_Avg calculated in Tab A_Wind
            if (abs(wind_error) > 180) // this limits error to < 180 and makes turn short way on compass + right, - left
              {
                 if(wind_to_steer > Wind_Avg)  wind_error = wind_error - 360;
                 if(wind_to_steer < Wind_Avg) wind_error = 360 + wind_error;
              }            
           heading_to_steer = -wind_error + heading;  // Wind Error = - Heading Error        
           } // end Steering_Mode == 4
    #endif  // Wind_Input
       
        
        // --------------------------------------------------------- Main compass steering -------------------------------------------------------------
        
        //  heading_error = heading_to_steer - heading;  // This is the main PID proportional term for compass based steering           
         heading_error = heading - heading_to_steer;  // This is the main PID proportional term for compass based steering   cfh 11.07.19 changed direction !        
        // heading_error = course - heading_to_steer;  // cfh testing steering based GPS COG
          if (Wind_Steer_Direct == 1 && Steering_Mode == 4) heading_error = -wind_error; // base case wind steer uses wind indicator to dervie a compass course, this mode will just steer wind error
          if (GPS_Steer_Direct == 1 && (Steering_Mode == 2 || Steering_Mode == 22)) heading_error = course_error;// similarly gps steering is compass steering where the error is based on COG - CTS, this mode leaves out the compass
           
           // Serial.print("Wind Error "); Serial.print(wind_error); Serial.print("; heading to steer "); Serial.println(heading_to_steer);            
           if (abs(heading_error) > 180) // this limits error to < 180 and makes turn short way on compass + right, - left
            {
               if(heading_to_steer > heading)  heading_error = heading_error - 360;
               if(heading_to_steer < heading) heading_error = 360 + heading_error;
               //if(heading_to_steer > course)  heading_error = heading_error - 360; // cfh testing steering based GPS COG
               //if(heading_to_steer < course) heading_error = 360 + heading_error; // cfh testing steering based GPS COG
            }

          /*
          lcd.setCursor(0,1);
          lcd.print("BRT       ");
          lcd.setCursor(5,1);
          lcd.print(bearingrate);
          */
          //if(abs(bearingrate) < 0.5 ) bearingrate = 0; // try to cut out bearing rate noise
      
           
          #if Compass == 0
           differential_error = bearingrate;
          #endif
          //NOTE if Compass == 1 (BNO055) bearing rate is calculated in that tab.  if other compasses added need to be sure bearing rate set by this point
          // Serial.print(heading_error);
          // Serial.print("  ");
          // Serial.println(differential_error);          
                        
          #if PID_MODE == 0          
            PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error); 
            rudder_command = PID_output + Rudder_Offset; // rudder offset set when key 1 or 3 pressed captures rudderoffset only used when User input RUDDER_OFFSET == 1
          #endif
          
          #if PID_MODE == 1   
           PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error); 
           rudder_command = rudder_command + PID_output + Rudder_Offset;  // this is a form of integral control was used summer of 2012 with estimated rudder position
          #endif
          
          #if PID_MODE == 3
           if(abs(heading_error)> deadband) // if heading error < deadband integral error stops accumulating
           {integral_error = integral_error + PID_Ks[3] * heading_error; // integral error used to control droop
           }
           if (!Steering) integral_error = 0;
           /*
           When sailing to windward a non zero rudder is needed to keep head into the wind with no integral term this requires a course error signal
           this is called droop and integral term will steer correct heading with non-zero rudder angles
           */
           integral_error = constrain(integral_error,-10,10);  // constrain intergral error to 10 degrees correction
           PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error + integral_error); // see void Key1_Pressed() for rudder offset
           rudder_command = PID_output;
          
          // Use this for debugging Integral error steering 
          if(Print_integral)
             {
              lcd.setCursor(0,0);
              lcd.print("CMD ");
              lcd.print(rudder_command,1);// diagnostic
              lcd.setCursor(11,0);
              lcd.print("IE ");
              lcd.print(integral_error *PID_Ks[0],1);// diagnostic note THis is effective integral error because IE multiplied by PID_Ks[0] above.
             }
           // if(abs(heading_error) < 10) TACK_ON = false;  // used to limit tack rate probably not needed
          //  if(TACK_ON) rudder_MAX = Tack_rudder_MAX; // 
          #endif
          
          if (Steering_Mode == 5)
         {
           Knob_Steering();          
         }  // end if(Steering_Mode == 5)
         
          rudder_MAX = Maximum_Rudder;
          if(TACK_ON)   rudder_MAX = Tack_rudder_MAX;
  //        if(new_waypoint) rudder_MAX = new_waypoint_rudder_MAX;
          rudder_command=constrain(rudder_command,-rudder_MAX,rudder_MAX);
        }   // end if  not DODGE_MODE
        
        if(!Steering)
           {
             rudder_command = 0;
             heading_to_steer = 0;
           }
    
        
         Rudder_Control(); // call Rudder for actual turning rudder
         
         if(Print_PID)
         {
           Serial.print("Course : "); Serial.print(course,1);
           Serial.print(" | GPS course to steer : "); Serial.print(GPS_course_to_steer,1); 
//           Serial.print(" | Heading Average: "); Serial.print(Cavg,1);
           Serial.print(" | Heading : "); Serial.print(heading);
           Serial.print(" | heading to steer: "); Serial.print(heading_to_steer,1);
           Serial.print(" | Heading Error: "); Serial.print(heading_error,1);
         //  Serial.print(" | compass delta T in sec: "); Serial.print(compass_delta_T);
          // Serial.print(" | delta heading: "); Serial.print(delta_heading);
           Serial.print(" | Bearing Rate: ");Serial.print(bearingrate);
           Serial.print(" | Integral error: "); Serial.print(integral_error);  
           Serial.print(" | PID Output: "); Serial.print(PID_output);
           Serial.print(" | Rudder: "); Serial.print(rudder_error);
          Serial.println("   ***Print_PID***"); 
         }   // end if Print PID       
       }  // End Void Steer_PID()  

/**********************************************************************/
        void Rudder_Control()
        {
         // int motorspeed_min = 30;
//         float Rudder_Power_coeff = 0;  //  Set to 0 to not use. Use .5 for default starting point.applies more motor speed proportional to rudder position to have more force to increase rudder at
             // bigger rudder angles to counter weather helm. At bigger rudder positions it takes more force to increase rudder
              
          RUDDER_POSITION();// update rudder position  
          if (RUDDER_MODE ==1) rudder_position = 0; // rudder feed back RFB not availabl  
          rudder_error = rudder_command - rudder_position;
          if(Print_Rudder_Commands) {
                Serial.print(" Rudder error: "); Serial.print(rudder_error); 
                Serial.print(" | = | Rudder command: "); Serial.print(rudder_error);
                Serial.print(" | + | Rudder position: "); Serial.print(rudder_position);
                Serial.print(" | deadband: "); Serial.print(deadband);
                 Serial.print(" | counts: "); Serial.print(counts); Serial.println(" ; ");
          }

          if(Steering_Mode == 0 || !sw1 || !sw2)  // sw1 and sw2 need to be on for automated steering
         {
            Steering = false;
            Rudder_Stop();
           #if Clutch_Solenoid  == 1
            Open_Solenoid();  // open solenoid to enable manual steering
          #endif  
         }
        #if Clutch_Solenoid  == 1 
        if(Steering_Mode != 0 && sw1 && sw2) Close_Solenoid();   // this closes solenoid to enable hydraulic steering, hardware dependent
        #endif
        
       if(Steering_Mode == 1 || Steering_Mode == 2 || Steering_Mode ==3 || Steering_Mode == 5) Steering = true; // could use if Steering_Mode >0
       // if(DODGE_MODE) Steering = false; //  if keypad LEFT or RIGHT RUDDER skip PID rudder control
            
        if(!DODGE_MODE) // do not steer if in dodge mode
        {
       //   if(rudder_on)  RUDDER_POSITION(); //if rudder on up date position      
          if(Steering)
          {      
             if(abs(rudder_error) < deadband) 
               {
                Rudder_Stop();
               }                    
         
            if(rudder_error > deadband)   
               { 
                 Right_Rudder();
               }
               
            if(rudder_error < - deadband)
               {
                 Left_Rudder();
               }  
          } // end  if Steering

        }  // end if(!DODGE_MODE)          
        }  // void rudder control

      
  //------------------------  RUDDER POSITION  -----------------------

  void RUDDER_POSITION()
  {
     float rudder_position_max = 45;  // cfh 29.06.2019  org 45
     float rudder_position_min = -45; // cfh 29.06.2019 org -45
     //float counts_max = 820;  // from calibration in print statement
     //float counts_at_zero = 440;
     //float counts_min = 64;

     
     counts = analogRead(4);
     //Serial.print("Rudder = "); // use these print lines to get counts for calibration
     //Serial.println(counts);

     // cfh 10.07.2019  This is the rudder position formula calculating the rudder position based on calibration setttings in encoder unita (counts) and the max/min rudder position values
      if(counts >= counts_at_zero) // linear calibration from zero
      {
          rudder_position = rudder_position_max *(counts - counts_at_zero) / (counts_max - counts_at_zero);
      }
      else
      {
          rudder_position = rudder_position_min * (counts - counts_at_zero) / (counts_min - counts_at_zero);
      }
      //rudder_position = - rudder_position;  // teporary reverse direction of positive rudder position
    
    // rudder_position =map(rudder_position, 187,910,-45,45); 
     
    // Serial.print("Rudder: "); Serial.print(rudder_position);
    
  }  // END VOID RUDDER POSITION
    
   /*  OLD RUDDER POSITION USING TIMING 
    float rudder_rate = .015; // deg/millisec
    int rudder_delta_time;
    int rudder_time;
    static float rudder_change; 
    
    if(rudder_on)
    {
      rudder_time = millis();
      rudder_delta_time = rudder_time - rudder_time_old;
      rudder_total_time = rudder_total_time + float(rudder_delta_time)/1000;  // diagnostic to see how much rudder motor is on can be commented out
      rudder_change = float(rudder_delta_time) * rudder_rate * rudder_direction; //time in milliseconds, change in deg 
      rudder_time_old = rudder_time;
      rudder_position = rudder_position + rudder_change;
    }
     if(abs(rudder_position) > rudder_MAX) Rudder_Stop();
   */  // END OLD RUDDER ROSITION USING TIMING  
    
   
  // ----------------------  END RUDDER POSITION --------------------
    
     
        /************************** 
         //Servo routine for rudder simulation
         rudder = 90+3 - PID_output; // 0 rudder = 90 deg position of servo so it will go 45 L and R the 3 is a trim on servo I used
          myservo.attach(38);  
          delay(50);
          myservo.write(rudder);
          delay(100);
          myservo.detach();  
         ****************************/
 
 //------------- RUDDER CONTROLS --------------------------------------------------
#if Clutch_Solenoid  == 1
 void Open_Solenoid()
   {
     #if Motor_Controller != 4 
          
          Serial_MotorControl.write(Motor_0_fwd);  // set amotor 0 forward, 137 for Qik, 201 for Trex
          Serial_MotorControl.write(0);  // speed = 0
            if(Print_Motor_Commands)
       {  Serial.print ("Motor Code, motorspeed ");
          Serial.print(Motor_0_fwd);Serial.print(", "); Serial.println(0);
       }
     #endif
     #if Motor_Controller == 4
        //Serial.println("Open solenoid Midtholm");
        digitalWrite(relay_Engage_solenoid, HIGH); 
     #endif
        
   }  // End Void Open Solenoid
 

 void Close_Solenoid()
   {
    #if Motor_Controller != 4 
         Serial_MotorControl.write(Motor_0_fwd); //  set amotor 0 forward, 137 for Qik, 201 for Trex
         Serial_MotorControl.write(127); // speed = full 
           if(Print_Motor_Commands)
       {  Serial.print ("Motor Code, motorspeed ");
          Serial.print(Motor_0_fwd);Serial.print(", "); Serial.println(127);
       }
     #endif
     #if Motor_Controller == 4
        //Serial.println("Close solenoid Midtholm");
        digitalWrite(relay_Engage_solenoid, LOW); 
     #endif
     //digitalWrite(10, LOW);   // close solenoid to enable autopilot steering steering 
   }  // end void Close-Solenoid
# endif

  void Rudder_Stop()
 {
    #if Motor_Controller < 3  
      Serial_MotorControl.write(Motor_1_fwd); //  for Qik 141.  set motor 1 forward for Trex(193)
      Serial_MotorControl.write(0); // set speed = 0
   #endif

   #if Motor_Controller == 3 
       motorspeed = 0;
       Serial_MotorControl.write(Motor_1_fwd);
       Serial_MotorControl.write(motorspeed & 0x1F);
       Serial_MotorControl.write(motorspeed >> 5);
   #endif  
   // cfh 09.06.2019
   #if Motor_Controller == 4
      //Serial.println("Stop Midtholm rudder");
      digitalWrite(relay_Turn_rudder_left, HIGH);
      digitalWrite(relay_Turn_rudder_right, HIGH); 
   #endif
   // end cfh
  //  rudder_stop_time = millis();
      rudder_on = false; 
      rudder_was_off = true;    
     if(Print_Motor_Commands)
     {  Serial.print ("Motor Code, motorspeed ");
        Serial.print(Motor_1_fwd);Serial.print(", "); Serial.println(0); 
     }
 }  // end Rudder_Stop
 
 
  void Left_Rudder()
  {
     #if Motor_Controller < 3
      Serial_MotorControl.write(Motor_1_rev);  //   set motor 1 in reverse, 143 for Qik, 194 for TREX
      Serial_MotorControl.write(motorspeed); // set speed = motorspeed 0 to 127 is 0% to 100%
     #endif
     
     #if Motor_Controller == 3
       Serial_MotorControl.write(Motor_1_rev);
       Serial_MotorControl.write(motorspeed & 0x1F);
       Serial_MotorControl.write(motorspeed >> 5);
     #endif
     // cfh 09.06.2019
     #if Motor_Controller == 4
        //Serial.println("Turn Midtholm left");
        digitalWrite(relay_Turn_rudder_right, HIGH); // always turn off right relay before engaging left
        digitalWrite(relay_Turn_rudder_left, LOW); 
     #endif
     // end cfh
    if(Print_Motor_Commands)
     {  Serial.print ("Rudder Command Motor Code, motorspeed ");
        Serial.print(rudder_command); Serial.print(", "); Serial.print(Motor_1_rev);Serial.print(", "); Serial.println(motorspeed);
     }
      rudder_on = true; //used in rudder position
      if (rudder_was_off)
          {
            rudder_time_old = millis();
            rudder_was_off = false;
          }
   // } 
  } // end Left_Rudder()
// --------------------------------------- 

  void Right_Rudder()
  {  
    
     #if Motor_Controller < 3
      Serial_MotorControl.write(Motor_1_fwd);  //   set motor 1 in reverse, 143 for Qik, 194 for TREX
      Serial_MotorControl.write(motorspeed); // set speed = motorspeed 0 to 127 is 0% to 100%
     #endif
     
     #if Motor_Controller == 3
       Serial_MotorControl.write(Motor_1_fwd);
       Serial_MotorControl.write(motorspeed & 0x1F);
       Serial_MotorControl.write(motorspeed >> 5);
     #endif
     // cfh 09.06.2019 
     #if Motor_Controller == 4
        //Serial.println("Turn Midtholm right");
        digitalWrite(relay_Turn_rudder_left, HIGH); // always turn off left relay before engaging right
        digitalWrite(relay_Turn_rudder_right, LOW); 
     #endif
     // end cfh
        rudder_on = true; //used in rudder position
        if (rudder_was_off)
          {
            rudder_time_old = millis();
            rudder_was_off = false;
          }
        if(Print_Motor_Commands)
         {   Serial.print ("Rudder Command Motor Code, motorspeed ");
        Serial.print(rudder_command); Serial.print(", "); Serial.print(Motor_1_fwd);Serial.print(", "); Serial.println(motorspeed);
         }  
    //}  // end if rudder < rudder MAX  
  } // end Right_Rudder
  

    /***********************************************************************/    
 #if GPS_Used
    void GPS_Steer()
    { 
      static int waypoint_error_count;
      
      if(!GPS_Available)
        {
          if(GPS_Was_Available) // GPS_Was_Available set true in void Is GPS Available in GPS based on UTC counting,  GPS_Was_Available set true there also
            {
              heading_to_steer = heading;
              GPS_Was_Available = false; // captures the heading to steer only once,  
            }  // end If(GPS_Was_Available
         GPS_Steering = false; // HTS is unchanged and boat will steer current compass HTS
         MSG = 2; //  NO GPS Steering HTS
        }  // end if GPS not available
        
     if(GPS_Available)
     {
      MSG = 0; // null message  
      GPS_Steering = true;
       if(Mode == "GPS2") Actual_GPS_Steering(); // allows GPS2 mode in case waypoint = NO WPT 8/20/17 JNE 14.6
        if ( Active_waypoint != "NO WPT")   //if you don't have valid fixes don't steer the boat, probably need an alarm. Needs to be more than just GPRMC 
       {      
           Actual_GPS_Steering();
           if (MSG != 4) MSG = 0; // NullMsg 4 turning to new waypoint durng time delay then null
       }  // end if Active_waypoint != "NO WPT"
     }  // End if GPS available
    } // end GPS_Steer() 
        
 /*************************************************************************************/
void Actual_GPS_Steering() // modified 4/25/18 to use Garmin approach for course correction
{  const float XTE_to_45_correction = 200; //  this is XTE in feet where full 45 deg correction will be applied Seems to be how GArmin GPS map 720 computes course to steer
   static long CTStime; 
  //CORRECTION FOR CROSS TRACK ERROR
 /* Course correction using Cross Track Error  */ 
    if(!Use_CTS){                  
    // #if GPS_source != 2 // GPS_source 1 is a single board,  2 is separate GPS board and ET.data transfer
    //   Get_Cross_Track_Error(); // redundant, Rev 15.0K 7-12/18
    // #endif                              
       XTE_course_correction = 45.0 * XTE / XTE_to_45_correction;  // See excel analysis of garmin GPS 760 course correction defined above 200 ft                   
       XTE_course_correction = constrain(XTE_course_correction,-45.0,45.0); 
       //course_to_steer = Bearing_origin_to_destination - XTE_course_correction;  // if XTE is positive(right of track) need to steer left hence negative term    
       course_to_steer = Waypoint_Bearing_From[WPT_index] - XTE_course_correction; // Rev 15.0K 7-12/18
    }   // end if(!USe_CTS) 
   /* This is couse corretion using chart plotter Course to steer  */
       if(Use_CTS) course_to_steer = GPS_course_to_steer;  // this negates all of the XTE calculations and uses CTS from NEMA GPAPB word 13         
        if (course_to_steer > 360) course_to_steer = course_to_steer - 360;
        if (course_to_steer < 0) course_to_steer = course_to_steer + 360;
        course_error = course_to_steer - course;
        //course_error = course_to_steer - Avg_course; // if key 2 pressed Avg_course = course, Avg interval set in USER INPUT
        heading_to_steer = heading + course_error;
            // NOTE heading_error = heading_to_steer - heading = course_error
        if (heading_to_steer > 360) heading_to_steer = heading_to_steer - 360;
        if (heading_to_steer < 0) heading_to_steer = heading_to_steer + 360;

     if(Print_Anticpate_Turn == 1) // should be off except for diagnostic analysis
     {
       if (millis() - CTStime >1000){  // should print every n/1000 sec
         CTStime= millis();
       Serial.println();
       Serial.println("WPT,     WPTix, BOD,   BRG, XTE, RNG,   COG,   CTS,   HDG,    CE,   HTS ");  
       Serial.print(Active_waypoint); Serial.print(", ");
       Serial.print(WPT_index); Serial.print(",   ");
       Serial.print(Waypoint_Bearing_From[WPT_index]); Serial.print(", ");
       Serial.print(Bearing_to_destination_by_LatLon,0); Serial.print(", ");
       Serial.print(XTE,0); Serial.print(", ");
       Serial.print(Range_Destination_by_LatLon,0); Serial.print(", ");
       lcd.setCursor(0,3);
       lcd.print("RNG         ");
       lcd.setCursor(4,3); 
       lcd.print(Range_Destination_by_LatLon,0);
       Serial.print(course); Serial.print(", ");
       Serial.print(course_to_steer,0); Serial.print(", ");
      // Serial.print(Avg_course); Serial.print(", ");
       Serial.print(heading); Serial.print(", ");
       Serial.print(course_error); Serial.print(", ");
       Serial.print(heading_to_steer); Serial.print(", ");
      // Serial.print(heading_error); Serial.print(", ");
       } 
     } 
  
}// end actual gps steering
#endif
    
 /*************************************************************************************/   
 void Knob_Steering()
 {
   float Knob;
    Knob =   analogRead(2);
    if (Knob > 600 || Knob < 400) {
        rudder_command =  2*Maximum_Rudder*(float(Knob/1000)) - Maximum_Rudder ; // rudder command +/- Maximum_Rudder using 10Kohm values ranging from 0 to 1000
      }
    else {
        rudder_command = 0;
      }
 
    //Serial.print("Knob: "); Serial.print(Knob); 
    //Serial.print("  CMD: "); Serial.print(rudder_command);  
   
   /*
   motorspeed = 255/40*(rudder_command); // + = right, - = left
   // sets motor speed 0 to 1023 for command = 0 to 40 degrees
   if( motorspeed > 255) motorspeed = 255;
   if ( motorspeed < -1023) motorspeed = -255;
   motorspeed= abs(motorspeed);
  */
  
  
   lcd.setCursor(0, 0);
  // lcd.print("                  ");
  // lcd.setCursor(0, 0);
   lcd.print("CMD ");
   lcd.print(rudder_command);
   
     lcd.setCursor(5,3);
     lcd.print("Rud                "); // extra spaces clear old data
     lcd.setCursor(9,3);
     lcd.print(rudder_position,0);
   
   
 
   /*
   lcd.setCursor(0,1);
   lcd.print("speed      ");
   lcd.setCursor(7,1);
   lcd.print("     ");
   lcd.setCursor(7,1);
   lcd.print(motorspeed);
   */
   
   
 } // end void knob steering
 
 
 
 /*************************************************************************************/
 /*
     void Tracking_Error()
  {
    /*  tracking error is difference between course over ground and compass heading.
        It's purpose is to compensate for lack of accuracy in heading sensor. It also
        corrects drift or anything else that causes a deviation betweeen the direction the boat 
        is travelling and the direction the compass says it is pointing.  The results are low pass
        filtered to provide a time averaged result that will not change abruptly wwhen going to a new course
        but recompute on a new course.
     */
     

/*  
           float tracking_error_LPF = .999; // tracking error updates when GPS updates about  once per second
           if(GPRMC_fix && SOG > 1)// only updates if valid fix  may want to add a lower speed limit
           {
             tracking_error = course - heading; // would like to use an average heading but need to figure out how so the 1  and 359 doesn't avg to 180
             
                if (abs(tracking_error) > 180) // this limits err0r to < 180 and makes turn short way on compass + right, - left
                  {
                     if(course > heading)  tracking_error = tracking_error - 360;
                     if(course < heading) tracking_error = 360 + tracking_error;
                  }
             
             AVG_tracking_error = tracking_error_LPF * AVG_tracking_error + (1 - tracking_error_LPF) * tracking_error;
           } // end if
         } // end Tracking_Error
   
 */ 
/********************************************************************************/
