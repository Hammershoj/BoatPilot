 #if GPS_Used ==1
 void Get_GPRMC()
   {
   int N_magvar; // number of decimals for magvar. Garmin uses 1, nobeltec 2
      //Serial.println("GPRMC");
    j_MAX = 13;  // number of Words in NEMA Sentence
    Parse_Sentence();
    //Serial.println("done Parsing");
    for(int j = 0; j<j_MAX; j++)
     {
      data_RMC[j] = data_IN[j]; //see void Parse Sentence
      if(print_RMC)Serial.println(data_IN[j]);
     } 
 
 /********************* CONVERT DATA  **************************/
     // data_RMC[0] = header no processing
         
     UTC_string = data_RMC[1];
    // UTC_string.toCharArray(ETdata.SD_UTC,7);
    // Date_Time();
   // long1 = strtol(char_buf,&pEnd,10); 
    // temp = strtol(char_buf,&pEnd,10);
     UTC = UTC_string.toInt();  //This works je 1/15/14 to convert to integer (long)
     //ETdata.SD_UTC = UTC;
    // Serial.println(UTC);
    // ETdata.SD_UTC = data_RMC[1];
   //  ETdata.SD_Time_decimal = Time_decimal;
     
     // data_RMC[2] = UTC Status, no processing
     
     To_Degrees(data_RMC[3]);
     Lat_current = Lat_Lon_Deg; // value returned from function
     if(data_RMC[4] == "S") Lat_current = - Lat_current;
    // Serial.println(Lat_current); 
   //  ETdata.SD_Lat_current = Lat_current; //pass to ETdata 
     To_Degrees(data_RMC[5]);
     Lon_current = Lat_Lon_Deg; // value returned from function
     if(data_RMC[6] == "W")Lon_current = -Lon_current;
    // ETdata.SD_Lon_current = Lon_current;//pass to ETdata
    //  = Lon E/W, no processing
    
    // data_RMC[7] = speed over ground sample 12.6
    
    string1 = data_RMC[7];
    NEMA_TO_FLOAT(1);
    SOG = float3;
   // ETdata.SD_SOG = SOG;
    
    // data_RMC[8] = Course, sample 234.6
    string1 = data_RMC[8];
    NEMA_TO_FLOAT(1);
    course = float3;
    //ETdata.SD_course = course; 
    
    //   data_RMC[9]Date
   /*  data_RMC[9].toCharArray(char_buf,16);
    long1 = strtol(char_buf,&pEnd,10); 
    // temp = strtol(char_buf,&pEnd,10);
     Date = long1;
   */
    // ETdata.SD_Date = Date;
         
    //  data_RMC[10] and data_RMC[11] Magnetic Variation E/W,     
    string1 = data_RMC[10];
    N_magvar = data_RMC[10].length() - data_RMC[10].indexOf('.') -1; // number of decimals
    NEMA_TO_FLOAT(N_magvar); // 2 for Nobeltec, 1 for GPSMAP 60CSX depends on how many decimal data sent or auto detect
    MagVar = float3;
    if(data_RMC[11] == "W") MagVar = -MagVar;
    //ETdata.SD_MagVar = MagVar;  
    
    
    // data_RMC[12] Fix Status 
     if (Word_count == 12)  data_RMC[12] = "D"; // NOBELTEC does not send data for data_APB[15] or RMC[12]
           if(data_RMC[12]=="A" || data_RMC[12]=="D" && checksum_status){GPRMC_fix = true;}
           else{GPRMC_fix= false;}                 
           if(GPRMC_fix){GPRMC_fix_status = "VALID FIX";}
           else{GPRMC_fix_status= "BAD FIX";}  
              
     // ETdata.SD_GPRMC_fix = GPRMC_fix;  
      if(GPRMC_fix) NewData = true;              

      if(print_RMC) {PRINT_RMC();}      
                
   }  //end of void GPRMC() case
                 
     
/********************* PRINT RMC **************************************************/  
    
     void PRINT_RMC()
   {
                 Serial.println();
                 Serial.println("---------");
  
                 Serial.print("Header: ");  
                 Serial.println(data_RMC[0]);
                   
                 Serial.print("UTC: ");    
                 Serial.println(UTC_string);
                 
                 Serial.print("UTC Status: ");              
                 Serial.println(data_RMC[2]);
                 Serial.print("Latitude: ");  
                 Serial.println(Lat_current,8);
                 
                 Serial.print("Lat N/S: ");
                 Serial.println(data_RMC[4]);
                 
                 Serial.print("Longitude: ");
                 Serial.println(Lon_current,8);

                 Serial.print("Lat E/W: ");
                 Serial.println(data_RMC[6]);
                 
                 Serial.print("Speed: ");
                 Serial.println(SOG,1);  
                 
                 Serial.print("Course: ");      
                 Serial.println(course,1);
                 
                 Serial.print("Date: ");    
                 Serial.println(data_RMC[9]);
                 
                 Serial.print("Mag Variation: ");              
                 Serial.println(MagVar);
               
                 Serial.print("variation E/W: ");   
                 Serial.println(data_RMC[11]);
                 
                 Serial.print("Type of Fix: ");  
                 Serial.println(data_RMC[12]);
                 
                 Serial.print("GPRMC Fix Status: ");
                 Serial.println(GPRMC_fix_status);
                 
                 Serial.println();
                 
          }  // End void PRINT_RMC
 #endif         
