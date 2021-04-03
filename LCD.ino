  /*********************** PRINT LCD ******************************/
void LCD(){
    // set the cursor to column 0, line 1
    // (note: counting begins with 0):
    String RP;
    int UTC_seconds;

    if (!Screen) Screen = 0;

    if(Screen == 0)
    {   

     //if(Use_CTS)lcd.print(Waypoint_next);
     //else 
     lcd.print(Active_waypoint);
          
    // lcd.print(HDG) also prints in compass for fast print rate, prints here for more stable LCD view
     lcd.setCursor(10,0);
     lcd.print("BRG ");
     lcd.setCursor(14,0);
     lcd.print(Bearing_to_destination,1); 
     
     lcd.setCursor(0, 1);
     lcd.print("HDG        ");
     lcd.setCursor(4, 1);
     lcd.print(heading,1);

     lcd.setCursor(0,2);
     lcd.print("COG       ");
     lcd.setCursor(4,2);
     lcd.print(course,1);
     lcd.setCursor(10, 2);   
     lcd.print("SOG       ");
     lcd.setCursor(14, 2);
     lcd.print(SOG,1);  
       
     lcd.setCursor(0,3);
     lcd.print("          ");
     lcd.setCursor(0,3);
     lcd.print(Mode);
  

     if(Steering_Mode != 4)
     {   
       lcd.setCursor(10, 1);   
       lcd.print("HTS      ");
       lcd.setCursor(14, 1);
       lcd.print(heading_to_steer,1);

     }
     
     if(Steering_Mode == 4)
     { 
       lcd.setCursor(10, 0);   
       lcd.print("WTS      ");
       lcd.setCursor(14, 0);
       lcd.print(wind_to_steer,1);        
     }

     if( RUDDER_MODE == 0)  // IF THERE IS A RUDDER POSITION INDICATOR
    {
     lcd.setCursor(5,3);
     lcd.print("Rud      "); // extra spaces clear old data  cfh 13.06.2019 added one space
     lcd.setCursor(9,3);
     lcd.print(rudder_position,0);
    }
   

} // END IF SCREEN = 0 

    /************* SCREEN = 1 ****************/
      if(Screen == 1)
     {
      lcd.setCursor(0,0);
      lcd.print("BRG ");
      lcd.print(Bearing_to_destination,3); 
       
      //lcd.setCursor(11,0);
      //lcd.print("XDE ");
      //lcd.print(XTE_differential_error,2);
      
      lcd.setCursor(0,1);
      lcd.print("XTE ");
      lcd.print(XTE,1);

      lcd.setCursor(11,1);
      lcd.print("Xint");
      lcd.print(XTE_integral_error,0);      
      
      lcd.setCursor(0,2);
      lcd.print("BOD ");
      lcd.print(Bearing_origin_to_destination,1);
      
     if(Steering_Mode !=4){ 
      lcd.setCursor(11,2);
      lcd.print("BRT ");
      lcd.print(bearingrate);
     }
    
      lcd.setCursor(0,3);
      lcd.print("XCR ");
      lcd.print(XTE_course_correction); 
      
     /* lcd.setCursor(11,3);
      lcd.print("Rud      "); // extra spaces clear old data
      lcd.setCursor(15,3);
      lcd.print(rudder_position,0);
    */
    
     lcd.setCursor(11,3);
     lcd.print("CMD    "); // extra spaces clear old data
     lcd.setCursor(15,3);
     lcd.print(rudder_command,0);
      
      /* 
      lcd.setCursor(0,1);
      lcd.print("DXTE ");
      lcd.print(XTE_differential_error); 
      */ 
     }// end screen 1
     
     
     
    /************* SCREEN = 2 ****************/
    
   
     
     if(Screen == 2)
     {
      
      lcd.setCursor(0, 0);
      lcd.print("                   ");
      lcd.setCursor(0, 0);
      lcd.print(GPRMC_fix_status); //no gps, no waypoint, or waypoint 
      lcd.setCursor(11,0);
      lcd.print("Lt "); lcd.print(Lat_current,3);
      lcd.setCursor(11,1);
      lcd.print("Ln "); lcd.print(Lon_current,3);
     
   //  This is a diagnostic it prints UTC time if GPS is processing
      lcd.setCursor(0,1);
      lcd.print("UTC ");
      lcd.print(UTC);
      /*
      lcd.setCursor(0,2);
      lcd.print("UTC Start ");
      lcd.print(UTC_start);
      
      lcd.setCursor(0,3);
      lcd.print("Max DT ");
      lcd.print(Time_decimal_MAX_seconds,0);
      */      
      lcd.setCursor(0,3);
      lcd.print("MAGV ");
      lcd.print(Magnetic_Variation,1);
      
      lcd.setCursor(0,2); 
      lcd.print("Rudder SPD       "); 
      lcd.setCursor(15,2);
      lcd.print(motorspeed); 
      
     } // End if screen = 2
 
   if(Screen == 3)
   {
     lcd.setCursor(0,0);
     if(Next_Turn >0) lcd.print("Next Turn R ");
      else lcd.print("Next Turn L ");
     // lcd.print(abs(Next_Turn), 1); 
      
     lcd.setCursor(0,1);
     lcd.print("Dist To Wpt = ");
   //  float Range_feet;
    //Range_feet = Range_Destination x 6076.1;
     lcd.print(Range_Destination,2);
     
     lcd.setCursor(0,2);
     lcd.print("Range ft =      ");
     lcd.setCursor(11,2);
     lcd.print(Range_Destination *6076.1,0); 
   
     lcd.setCursor(0,3);
     lcd.print(Active_waypoint);      
   }  // End screen = 3

      if(Screen == 4)
   {
      lcd.setCursor(0,0);
      lcd.print("HDG     ");
      lcd.setCursor(4,0);
      lcd.print(heading,0);
      lcd.setCursor(0,1);
      lcd.setCursor(10,0);
      lcd.print("DPT      ");
      lcd.setCursor(14,0);
      lcd.print(Depth,1);
      lcd.setCursor(0,1);
      lcd.print("Wind     ");
      lcd.setCursor(5,1);
      lcd.print(Wind_Speed,0);
      lcd.setCursor(10,1);
      lcd.print("MAX     ");
      lcd.setCursor(14,1);
      lcd.print(Wind_MAX,0);
      lcd.setCursor(0,2);
      lcd.print("Wind Angle      ");
      lcd.setCursor(11,2);
      lcd.print(Wind_Dir,0);
      #if UseBarometer
      lcd.setCursor(0,3); lcd.print("Hg      ");      
      lcd.setCursor(3,3); lcd.print(pressure); 
      lcd.setCursor(10,3); lcd.print("Temp     ");      
      lcd.setCursor(15,3); lcd.print(temperature,1); 
      #endif
       
   }
 // cfh 13.06.2019
      if(Screen == 5)
   {
      lcd.setCursor(0,0);
      lcd.print("<1> Rudder:");
      lcd.setCursor(11,0);
      // 0 uses rudder position, 1 does not
      Rudder_Change_Mode();
      if (RUDDER_MODE == 0) lcd.print("ON ");
      if (RUDDER_MODE == 1) lcd.print("OFF");
      lcd.setCursor(14,0);
      lcd.print("      ");
      lcd.setCursor(15,0);
      lcd.print(counts,0);
      //lcd.setCursor(4,0);
      //lcd.print(heading,0);
      lcd.setCursor(0,1);
      lcd.print("<2> Save left:      ");
      lcd.setCursor(15,1);
      lcd.print(counts_min,0);
      lcd.setCursor(0,2);
      lcd.print("<3> Save mid:       ");//lcd.setCursor(14,0);
      lcd.setCursor(15,2);
      lcd.print(counts_at_zero,0);
      lcd.setCursor(0,3);
      lcd.print("<4> Save right:     ");//lcd.setCursor(14,0);
      lcd.setCursor(15,3);
      lcd.print(counts_max,0);

      #if UseBarometer
      lcd.setCursor(0,3); lcd.print("Hg      ");      
      lcd.setCursor(3,3); lcd.print(pressure); 
      lcd.setCursor(10,3); lcd.print("Temp     ");      
      lcd.setCursor(15,3); lcd.print(temperature,1); 
      #endif
       
   }
// end cfh 13.06.2019
 /*
 #if Compass == 1
     if(Screen == 5)  // Save Compass Calibration
   {
     
     lcd.setCursor(0,0);
     lcd.print("BNO055 Compass Cal");
     lcd.setCursor(0,1);
     lcd.print("To save current Cal");
     lcd.setCursor(0,2); 
     lcd.print("Press Key 5");
     lcd.setCursor(0,3);
     if(DataStored)
      lcd.print("DATA STORED");    
   }  // End screen = 5
 #endif  
 */
}  // END Void LCD()
