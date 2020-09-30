  
 /***********************
 this loop attempts to integrate gyro and GPS with Gyro and PID at 50 hz 
 **************************/
 
   void A_P_Loop()
  { 
    static int DT_test;  // to print loop times
   // int rudder_duration = 40;  // this is time the rudder remains on each time it is turned, this slows down rudder
   static int Icount;
  #if GPS_Used == 1 && GPS_source == 1
    GET_sentence(); // This is done at a high rate so indiviual GPS bytes are captured  1 at a time as they come in
  #endif
  #if Wind_Input == 1
   if (Steering_Mode != 2 || Steering_Mode != 22) // fix Wind interference with GPS steering Need to find actual interference
   {
    get_Wind();
   }
  #endif
    if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    counter2++;
    counter3++;
    timer_old = timer;
    timer=millis();
  #if Compass == 0
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
      // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    Bearing_Rate();
    
 #endif

    // read compass more often
  /*   Icount ++;  // use this block to measure milliseconds per 100 iterations
     if(Icount>100)
     {Icount=0;
     Serial.println(millis());
     }
     */
    if (counter > 1)  // with delays loop runs at 10Hz  
     {
     
    /*
     Icount ++;  // use this block to measure milliseconds per 100 iterations
     if(Icount>100)
     {Icount=0;
     Serial.println(millis());
     }
    */ 
      counter=0;
   #if Compass == 0
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading Pololus Mag heading 
      JNE_AP_Compass_Correction();  // compute true heading
       //Serial.print(heading,1); Serial.print("   "); Serial.println(bearingrate); // temporary   
   #endif

  #if Compass == 1
    BNO055();
    JNE_AP_Compass_Correction();  // compute true heading
    if(counter2 >40)BNO055_Get_Cal(); //get BNO cal is not need in fast loop
  #endif

 /* #if Wind_Input ==1
  if(SoftSerial1.available() >25){
    SoftSerial1_Bytes = SoftSerial1.available();
   //Serial.print("Serial bytes available = "); Serial.println(SoftSerial1_Bytes);
    //byteWind=SoftSerial1.read();
    //Serial.write(byteWind);
    //Serial.println();  
    get_Wind();
  }
    #endif
 */  
/*
      if(Serial_Remote_Is_Connected)
         {
           Remote_Send_Data (); 
           ET2.sendData();  // Sends data to Remote
         }
 */

 #if GPS_Used == 1
  #if GPS_source == 2  //  get_GPS_data uses Easy Transfer to read data from second Mega 1 means one board, 2 means 2 boards
    if(ET.receiveData()) {
      get_GPS_data();
    } // end if(ET.receive data()) 
  #endif

#if CTS == 0
  Waypoint_Current();  // if Active_waypoint is not updated in Waypoint_Age_Max = 6 sec Active_waypoint = "NO WPT"
  //ETdata.SD_NEMA_sentence = NEMA_sentence;      
  if(NewData) //Set true in GPRMC 
  { 
    for (int i = 0; i< Number_of_waypoints; i++)  // get the index of the current GPS destination waypoint
     {      
       if( Waypoint_next == Route[i])
         { GPS_WPT_index = i; // this is the index of the current waypoint coming from the GPS
                             // Serial.print("GPS waypoint index "); Serial.println(GPS_WPT_index);
         break;
         } // end if Waypoint_next = 
     }  // end for 
    if (millis() - RTE_timer < 60000) RTE_Active = true; // less than one minute since last RTE sentence processed        
    if(Anticipate_Turns && RTE_Active) ANTICIPATE_TURN(); // if distance to waypoint is less than turning distance set the active waypoint as Waypoint_next
          // If less than turn distance WPT_index = GPS_WPT_index+1
    if (Anticipated_Turn_Active == false)
      {
        WPT_index = GPS_WPT_index; 
      }   
      Active_waypoint = Route[WPT_index]; 
    Get_Cross_Track_Error(); // gets cross track error and calculated course_to_steer
    //if(Use_CTS == 0) course_to_steer = -1;  // AP can use the negative value to determine wether to use incoming CTS from APB or to calculate CTS from BOD and XTE
  }  // end if new data
#endif
 if (Use_CTS == 1)Active_waypoint = Waypoint_next; // Waypoint next is NMEA input, Active waypoint is indexed up to Anticipate turns
 /*****   End GPS BLOCK A    ******/
  
 /**************  GPS BLOCK B   *****************/ 
 /* 
  * This block was executed after receiving GPS ET data 
  */
    if (!GPRMC_fix) GPS_status = "LOST GPS";
    if (GPRMC_fix) GPS_status = "GPS OK"; 
    Avg_course = K_course_avg * Avg_course + (1 - K_course_avg) * course;  //see PID GPS steering
    Is_GPS_Available();  //Checks UTC failing to increase for more than 6 seconds 
      // if( !GPS_Available && GPS_Was_Available)
   if(!GPS_Available)  
    {
     // Serial.println(" GPS NOT AVAILABLE"); // cfh 15.06.2019 commented out
     if(GPS_Was_Available) MSG = 1; // NO GPS
      if(Steering_Mode == 2) MSG = 2; // No Gps Steering HTS
      Active_waypoint = "NO GPS";
    }
    //Serial.print("MSG = "); Serial.println(MSG);
 /***   end GPS BLOCK B   ***/       
 #endif // end if GPS_Used  
    
 #if RF24_Attached == 1
     sendData1(); // RF send data1
 #endif
   
    Steer_PID(); 
   
   } // end if counter > 8 10 Hz loop
      
 #if Compass == 0   
   if (PRINT_DATA == 1) {     // See tab Print_2 needed for Python display note thi is conditional compiling
   printdata();
   }

   if(Print_Gyro)
    {  
    Serial.print("Gyros ");
    Serial.print(ToDeg( Gyro_Vector[0])); Serial.print("' ");
    Serial.print( ToDeg(Gyro_Vector[1])); Serial.print("' ");
    Serial.println( ToDeg(Gyro_Vector[2]));
      lcd.setCursor(0,0);
      lcd.print ("GYROS");
      lcd.setCursor(0,1);
      lcd.print(ToDeg(Gyro_Vector[0]));
       lcd.setCursor(0,2);
      lcd.print(ToDeg(Gyro_Vector[1]));
       lcd.setCursor(0,3);
      lcd.print(ToDeg(Gyro_Vector[2]));
    }
 #endif
 #if Board == Teensy
   #if TFT_Used 
     Print_Screen_0(); //tft screen update
   #endif
 #endif
  }  // end fast loop if millis > 20
  

  if (counter2 >47) // print loop
  {
   //this executes at 1 hz 
   counter2 = 0; 
   //get_GPS_data(); //  This receives the GPS data from separate Arduino, does so in 2 milliseconds 
   if(Print_LCD_AP) LCD(); //This is main LCD print of GPS and Compass can be turned off to use 
   //LCD to print special purpose diagnostics
  #if Compass == 0
   if(Print_LCD_IMU9) LCDprint();
  #endif 
  #if GPS_Used   
  if (print_Nav_Data) NAV_DATA_PRINT();  // Print_1 Tab 
  #endif
  // DT_test = millis() - DT_test;
  // Serial.println(DT_test);  // diagnostic to check loop timing  
  // DT_test = millis();
  //Serial.println(heading);  
  #if UseBarometer
  if (Screen == 4) Read_Barometer(); // only reads when screen 4 displayed to save computation time 
  #endif
  }  // end counter2 1 hz loop
  /*
  #if counter3 >= 3000 && Print_heading // 3000 = 1 minute Special purpose print loop for diagnotics
  
      Read_Compass();    // Read I2C magnetometer this is not main loop read compass here for diagnotic delete this line
      Compass_Heading(); // Calculate magnetic heading   Delete this line  
     
      Serial.print(millis()/1000); Serial.print(", ");
      Serial.print (magnetom_x); Serial.print(", ");
      Serial.print (magnetom_y); Serial.print(", ");
      Serial.print (magnetom_z); Serial.print(", ");
      Serial.print (c_magnetom_x,8); Serial.print(", ");
      Serial.print (c_magnetom_y,8); Serial.print(", ");
      Serial.print (pitch,8); Serial.print(", ");
      Serial.print (roll,8); Serial.print(", ");
      Serial.print ("Heading = ");
      Serial.print(MAG_X,8); Serial.print(", ");
      Serial.print(MAG_Y,8); Serial.print(", ");
      Serial.print(MAG_Heading_Degrees,5); Serial.print(", ");
      AVG_Heading = .9 *AVG_Heading + .1 *MAG_Heading_Degrees;
      Serial.println(AVG_Heading,1); 
      Serial.println(heading); 
      counter3 = 0;
   #endif  // end slow  print loop
   */ 
   }  // end void AP loop
 
