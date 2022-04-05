
  /**********************************
  
  KEYPAD TAB
  
  ***************************************/
 // based on eventkeypad by Alexander Brevig 
 
/* AutoPilotKeypad V1 (same as EventKeypad_and_LCD_v6) is set up to use keypad to get input for Autopilot by Jack Edwards
  it does the following:
  
  KEY TYPE  ACTION
   0 press - sets steering mode to 0 OFF
   1 press - sets steering mode to 1 COMPASS
   2 press - set steering mode to 2 GPS
   3 press - Tack
   4 press - decrease course b 10 deg, 90 in Tack
   5  press - none,  hold - none
   6 press - increase course 10 deg, 90 in Tack
   7 press - decrease course by 1 deg
   8 LCD screen increment then back to zero
   9 press - increase course by 1 deg
   * press/release - Left Rudder ON until released then rudder OFF
   0 press - sets steering mode to 0 OFF
   # press/release - Right Rudder ON until released then OFF
*/
void KEYPAD()
{
  char key = keypad.getKey();
  /*
    if (key) {
        Serial.println(key);
    }
  */
 
}

//take care of some special events
void keypadEvent(KeypadEvent key){
  switch (keypad.getState()){
    case PRESSED:
      KeyPressed(key);
    break;
  
    case RELEASED:
     KeyReleased(key);
    break;
   
    case HOLD:
     KeyHeld(key);
    break;
    } // end switch(ad.getstate)
  }  //  end void keypadEvent(KeypadEvent key)
  
 /****************************************************/
  
void KeyPressed(char keyin)
{  
     Serial.print("keyin = "); Serial.println(keyin);
        switch (keyin){
        case '0': 
          lcd.clear();
          Key0_Pressed();
        break; // end case 0
   
        case '1': 
         // cfh 15.06.2019 added rudder settings code
         //Serial.print("Case 3  Screen "); Serial.println(Screen); 
         if (Screen == 5) {
              Serial.print("Change rudder mode "); Serial.println(RUDDER_MODE); 
              Change_rudder_mode = true;
              break;     
         }
        Key1_Pressed();
        break;
        
        case '2':
        // cfh 15.06.2019 added rudder settings code for screen 5
        if (Screen == 5) {
              Rudder_Store_at_left();
              Serial.print("Store left max"); Serial.println(counts_min);
              break;     
        } // cfh      
        #if GPS_Used == 1
          if(!GPS_Available) {
            Mode = "No GPS";
          }  
          else {
           Steering_Mode = 2;
            Mode = "GPS";   
            UTC_timer_old = millis();
            Date_Time();
            Time_decimal_old = Time_decimal;
            GPS_Steering = true; 
            XTE_course_correction = 0;
            XTE_integral_error = 0; // allows zeroing out integral error by re-pressing key 2
            Avg_course = course; 
            lcd.setCursor(0,3);
            lcd.print("          ");
            lcd.setCursor(0,3);
            lcd.print(Mode);
          }
          break; 
         #endif

         case '3': 
         // cfh 15.06.2019 added rudder settings code for screen 5
             if (Screen == 5) {
                Rudder_Store_at_zero();
                Serial.print("Store at zero:"); Serial.println(counts_at_zero);
               break;     
             } // cfh

        // needs to be entered from mode 1
          
          toggle = !toggle; // toggle starts out false and resets to false  when key 0 is pressed.  toggle toggles between TACK and WIND
          if (toggle)
            {
            if (Steering_Mode != 1) 
            {
                  heading_to_steer = heading; // if previously steering Comp going to tack does not reset HTS to current heading
                  integral_error = 0; // reset integral error
               #if RUDDER_OFFSET == 1
                  Rudder_Offset = rudder_position; // placed before toggle this works for TACK and WIND see notes 10.21.16
               #if PID_MODE == 3
                  integral_error = Rudder_Offset/PID_Ks[0]; // sets initial  
                          //integral error = rudder position  - bearingrate at time steering engaged divided by 
                          // overall PID gain because multiplied by overall gain in PID equation
               #endif
               #endif
               #if BEARINGRATE_OFFSET == 1
                 bearingrate_Offset = - bearingrate; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                          // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
               #endif  
            }
            Steering_Mode = 3;
            Mode = "TACK";    
            } // end if toggle is true
          
          if (!Wind_Input) // if the wind instrument is not availble in user setup this will skip toggling to wind mode
            { toggle = false;
             break;
            }  // end if wind instrument is not available
            if(!toggle)
              {
                Steering_Mode = 4;
                wind_to_steer = Wind_Dir ;
                Mode = "WIND";
              } //end if not toggle
        break; 
         
        case '*':   //Rudder on Dodge Left
          if(Steering_Mode == 0 || Steering_Mode ==5) break;
          {  
            DODGE_MODE = true;
            Previous_Mode = Mode;
            Mode = "PORT";
           motorspeed = motorspeedMAX;
            Left_Rudder();
          }
         break;
          
        case '#': //  rudder on Dodge right
         if(Steering_Mode == 0 || Steering_Mode ==5) break;
          {   
            DODGE_MODE = true;
            Previous_Mode = Mode;
            Mode = "STBD";
           motorspeed = motorspeedMAX;
            Right_Rudder();
          }
        break;
          
        case '4': 
             // cfh 15.06.2019 added rudder settings code for screen 5
             if (Screen == 5) {
                //Serial.print("Change rudder mode "); Serial.println(RUDDER_MODE); 
                Rudder_Store_at_right();
                Serial.print("Store right min:"); Serial.println(counts_max);
               break;     
             } // cfh
             
             if (Steering_Mode==1) heading_to_steer = heading_to_steer -10;           
             if(Steering_Mode ==3)
            { 
              heading_to_steer = heading_to_steer - Tack_Angle;
              TACK_ON = true;
            }
           if(Steering_Mode !=4){   
             if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
             if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
           }  // end if steering mode != 4     
            
 #if Wind_Input == 1
           if(Steering_Mode == 4)
            {
              wind_to_steer = wind_to_steer + 10; // opposite of HTS
              if (wind_to_steer < 0) wind_to_steer = wind_to_steer +360; 
              if (wind_to_steer > 360) wind_to_steer = wind_to_steer -360; 
              lcd.setCursor(15, 1);
              lcd.print(wind_to_steer,1);
            }
#endif              
              if (Steering_Mode == 22)
             {
               CTS_GPS2 = CTS_GPS2 - 10;
               if (CTS_GPS2 < 0) course = CTS_GPS2 +360;
                if (CTS_GPS2 > 360) CTS_GPS2 = CTS_GPS2 -360; 
                lcd.setCursor(15, 2);
                lcd.print(CTS_GPS2,1); 
             } // end if steering mode == 22
             
             if(Steering_Mode != 1 && Steering_Mode != 3 && Steering_Mode != 22 && Steering_Mode !=4)
             {
                //lcd.begin();
                lcd.setCursor(6,2);
                lcd.print("WRONG MODE");  // this would be a good place to put a audible alarm beep
               // delay(250);
             }
         break;  // Break for case PRESSED
         
              
         case '5': 
          if(Screen !=4)
          { 
            Steering_Mode = 5;
            Mode = "KNOB";     
          }
        #if Compass == 1 
          if(Screen == 4)
          {
          DataStored = false;
          BNO_SaveCal();
          }
        #endif
         break;  // case5
         
        case '6': 
             if(Steering_Mode==1) heading_to_steer = heading_to_steer + 10;
             if(Steering_Mode ==3)
            { 
              heading_to_steer = heading_to_steer + Tack_Angle;
              TACK_ON = true;
            }

            if(Steering_Mode!=4)
            {
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
            }
            #if Wind_Input == 1            
            if(Steering_Mode == 4){
              wind_to_steer = wind_to_steer - 10; // opposite HTS
              if (wind_to_steer < 0) wind_to_steer = wind_to_steer +360; 
              if (wind_to_steer > 360) wind_to_steer = wind_to_steer -360; 
              lcd.setCursor(15, 1);
              lcd.print(wind_to_steer,1);
            }
            #endif               
            if (Steering_Mode == 22)
             {
               CTS_GPS2 = CTS_GPS2 + 10;
               if (CTS_GPS2 < 0) course = CTS_GPS2 +360;
                if (CTS_GPS2 > 360) CTS_GPS2 = CTS_GPS2 -360; 
                lcd.setCursor(15, 2);
                lcd.print(CTS_GPS2,1); 
             } // end if steering mode == 22
             
             if(Steering_Mode != 1 && Steering_Mode != 3 && Steering_Mode != 22 && Steering_Mode !=4)
             {
                //lcd.begin();
                lcd.setCursor(6,2);
                lcd.print("WRONG MODE");  // this would be a good place to put a audible alarm beep
                delay(250);
             }
          break;
            
            
          case '7': 
             if (Steering_Mode==1 || Steering_Mode ==3)
             {
                heading_to_steer = heading_to_steer -1;
                if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
                if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360;
             }
               #if Wind_Input == 1             
                          if(Steering_Mode == 4){             
                            wind_to_steer = wind_to_steer + 1; // opposite HTS
                            if (wind_to_steer < 0) wind_to_steer = wind_to_steer +360; 
                            if (wind_to_steer > 360) wind_to_steer = wind_to_steer -360; 
                            lcd.setCursor(15, 1);
                            lcd.print(wind_to_steer,1);
                           }
              #endif             
              if (Steering_Mode == 22)
             {
               CTS_GPS2 = CTS_GPS2 -1;
               if (CTS_GPS2 < 0) course = CTS_GPS2 +360;
                if (CTS_GPS2 > 360) CTS_GPS2 = CTS_GPS2 -360; 
                lcd.setCursor(15, 2);
                lcd.print(CTS_GPS2,1); 
             } // end if steering mode == 22
             
             if(Steering_Mode != 1 && Steering_Mode != 3 && Steering_Mode != 22 && Steering_Mode != 4)
             {
               //lcd.begin();
                lcd.setCursor(6,2);
                lcd.print("WRONG MODE");
                delay(250);
             }
          break;
           
         case '8': 
              Screen = Screen +1;
             #if Compass == 1
             if( Screen > 5) Screen = 0;
             #endif
             #if Compass == 0
             if( Screen > 5) Screen = 0;  // cfh 15.06.2019 changed from 4 to five to include calibration screen
             #endif
             Serial.print("screen = "); Serial.println(Screen);
            // cfh
            //
            lcd.clear();
            LCD();
         break;
            
        case '9': 
             if (Steering_Mode == 1 || Steering_Mode == 3)
             {
                heading_to_steer = heading_to_steer +1;
                if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
                if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360;
             }
 #if Wind_Input == 1
           if(Steering_Mode == 4){
              wind_to_steer = wind_to_steer - 1; // opposite HTS
              if (wind_to_steer < 0) wind_to_steer = wind_to_steer +360; 
              if (wind_to_steer > 360) wind_to_steer = wind_to_steer -360; 
              lcd.setCursor(15, 1);
              lcd.print(wind_to_steer,1);
           }
#endif            
              if (Steering_Mode == 22)
             {
               CTS_GPS2 = CTS_GPS2 + 1;
               if (CTS_GPS2 < 0) course = CTS_GPS2 +360;
                if (CTS_GPS2 > 360) CTS_GPS2 = CTS_GPS2 -360; 
                lcd.setCursor(15, 2);
                lcd.print(CTS_GPS2,1); 
             } // end if steering mode == 22
             
             if(Steering_Mode != 1 && Steering_Mode != 3 && Steering_Mode != 22 & Steering_Mode !=4)
             {
               //lcd.begin();
               lcd.setCursor(6,2);
               lcd.print("WRONG MODE");
               delay(250);
             }
               
            break;
        // these next two cases come from the remote keypad where A or B sent when * or # released
        case'A':
         Star_Released(key);
        break;    
        
        case'B':
         Pound_Released(key);
        break;    
        
        case'C':
        #if GPS_Used == 1
         GPS2_mode();
        #endif
        break;    
            
      }   // end swtich key
} // End void KeyPressed

/************************************************************************/
void KeyReleased (char keyin)
{
   switch (keyin){ 
       
        case '*':  
         Star_Released(key);
        break;
          
        case '#': 
         Pound_Released(key);
        break;          
      }  // end swtich key  
} // end key released

/**************************************************/
void KeyHeld(char keyin){
  switch (keyin){
      case '2':
      #if GPS_Used ==1
      GPS2_mode(); 
      #endif
      break;
  } // end switch key
} // end void KeyHeld


/***********************************************/

        void Key0_Pressed()
        {
          Steering_Mode = 0;
          Mode = "OFF";
          GPS_Was_Available = false;
          Accept_Terms = 0;  // this quits printing the Terms and conditions on start up
          Screen = 0;  
          toggle = false; // resets key 3 to tack mode instead of wind mode
          analogWrite(LPWM_Output, 0);
          analogWrite(RPWM_Output, 0);
          
        #if Board == Arduino
          //lcd.begin();
        #endif
        #if BEARINGRATE_OFFSET == 1
          bearingrate_Offset = 0; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                  // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
        #endif  
          LCD();
        }  // end Key0 pressed

/*******************************************************/
void Key1_Pressed()
{
          Steering_Mode = 1;
          Mode = "COMP";
          heading_to_steer = heading;
          integral_error = 0; // reset integral error
          #if RUDDER_OFFSET == 1
            Rudder_Offset = rudder_position; // placed before toggle this works for TACK and WIND see notes 10.21.16 
            #if PID_MODE == 3
              integral_error = Rudder_Offset/PID_Ks[0] + bearingrate_Offset/PID_Ks[0]; // sets initial  
                      //integral error = rudder position at time steering engaged divided by 
                      // overall PID gain because multiplied by overall gain in PID equation
            #endif
           #endif
           #if BEARINGRATE_OFFSET == 1
             bearingrate_Offset = - bearingrate; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                      // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
           #endif  
     
}  // end key1 pressed

void Key2_Pressed(){
  #if GPS_Used
          Steering_Mode = 2;
          Mode = "GPS";   
          UTC_timer_old = millis();
          Date_Time();
          Time_decimal_old = Time_decimal;
          GPS_Steering = true; 
          XTE_course_correction = 0;
          XTE_integral_error = 0; // allows zeroing out integral error by re-pressing key 2
          Avg_course = course; 
          #if BEARINGRATE_OFFSET == 1
             bearingrate_Offset = - bearingrate; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                      // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
          #endif  
 #endif
}  // end Key2 pressed

       
/********************************************************/
void Star_Released(char keyin)
  {
          if(Steering_Mode == 0 || Steering_Mode ==5) return; 
          DODGE_MODE = false;
          Rudder_Stop();
          Mode = Previous_Mode;
   }  // end Star_Released
   
/*****************************************************/
void Pound_Released(char keyin)
  {
          if(Steering_Mode == 0 || Steering_Mode ==5) return; 
          DODGE_MODE = false;
          Rudder_Stop();
          Mode = Previous_Mode;
  
   }  // end Star_Released
/******************************************************/
#if GPS_Used == 1
void GPS2_mode()
 {
     if(!GPS_Available)return;
          //Mode = "NA"; // temporarily disabled
          
          Steering_Mode = 22;
          Mode = "GPS2"; 
          CTS_GPS2 = course; // captures current course as the course to steer and will steer this instead of going to waypoint  
          UTC_timer_old = millis();
          Date_Time();
          Time_decimal_old = Time_decimal;
          GPS_Steering = true; 
          XTE_course_correction = 0;
          XTE_integral_error = 0; // allows zeroing out integral error by re-pressing key 2
          Avg_course = course;
          #if BEARINGRATE_OFFSET == 1
             bearingrate_Offset = - bearingrate; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                      // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
          #endif  
           
          lcd.setCursor(0,3);
          lcd.print("          ");
          lcd.setCursor(0,3);
          lcd.print(Mode); 
 } // end GPS2_mode
 #endif
