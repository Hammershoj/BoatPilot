 /*********************************************************
 PRINT 1 TAB
 these print routines were developed to print GPS and NAV data.  The Print 2 tab is
 the OUTPUT TAB provided with Pololu MinIMU9 software for Gyro compass, They need to be
 integrated in terms of functions and varibles.
 ****************************************************/
#if GPS_Used 
        void Print_interval(){
          //Allows user to set Serial Port printing interval   
       int  print_timer = millis()-PT_old;
        //temp = print_time*1000;
        if (print_timer  > print_time*1000){
          print_level = print_level_max;
          PT_old = millis();
        }
        else{ print_level=0;}
       /************************** // diagnostic
        print_level=0;
        Serial.print("millis(): "); Serial.println(millis());
        Serial.print("print_time: "); Serial.println(print_time);
        Serial.print("PT_old: "); Serial.println(PT_old);
        Serial.print("Print Interval: "); Serial.println(print_time*1000);
        ***************************************************/
        }  // end of  void Print_interval()
  


/****************************************************/

    void NAV_DATA_PRINT()
    {
      
     // Serial.println(" UTC, waypoint, BRG, RNG, BOD, COG, XTE, XTE_Corr, CTS,HDG,  HTS, CMD, RUD);
      Serial.print(UTC_string); Serial.print(", ");
      Serial.print(Waypoint_next); Serial.print(", ");
      Serial.print(Bearing_to_destination); Serial.print(", ");
      Serial.print(Range_Destination,3); Serial.print(", ");
      Serial.print(Bearing_origin_to_destination ); Serial.print(", ");
      Serial.print(course ); Serial.print(", ");
      Serial.print(XTE); Serial.print(", ");
      Serial.print(XTE_course_correction ); Serial.print(", ");
      Serial.print(course_to_steer); Serial.print(", ");
      Serial.print(heading); Serial.print(", ");
      Serial.print(heading_to_steer); Serial.print(", ");
      Serial.print(bearingrate); Serial.print(", ");
      Serial.print(rudder_command ); Serial.print(", ");
      Serial.print(rudder_position);Serial.print(", ");
      Serial.println();
    }  // end NAV_DATA_PRINT
#endif    
 /***********************************************************************************/   


