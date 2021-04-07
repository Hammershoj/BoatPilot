  
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
      timer_old = timer;
      timer=millis();

      if (counter > 5)  // with delays loop runs at 10Hz  
      {
      
          counter=0;
          //Serial.print("Timer: ");
          //Serial.println(timer);
          #if Compass == 1
            BNO055();  // Read compass data from Bosch BNO055 sensor
          #endif
          #if Wind_Input ==1
              if(SoftSerial1.available() >25){
                SoftSerial1_Bytes = SoftSerial1.available();
              //Serial.print("Serial bytes available = "); Serial.println(SoftSerial1_Bytes);
                //byteWind=SoftSerial1.read();
                //Serial.write(byteWind);
                //Serial.println();  
                get_Wind();
              }
          #endif
        
          Steer_PID(); 
    
      } // end if counter > 8 10 Hz loop
 
    }  // end fast loop if millis > 20
  

  if (counter2 > 47) // print loop
  {
    //this executes at 1 hz 
    counter2 = 0; 
    
    //get_GPS_data(); //  This receives the GPS data from separate Arduino, does so in 2 milliseconds 
    if(Print_LCD_AP) LCD(); //This is main LCD print of GPS and Compass can be turned off to use 
    //LCD to print special purpose diagnostics
    
    #if GPS_Used   
    if (print_Nav_Data) NAV_DATA_PRINT();  // Print_1 Tab 
    #endif
    
    #if UseBarometer
    if (Screen == 4) Read_Barometer(); // only reads when screen 4 displayed to save computation time 
    #endif
  }  // end counter2 1 hz loop

   }  // end void AP loop
 
