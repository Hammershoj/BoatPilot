#if GPS_Used ==1
   void Get_GPRMB()
   {
     //Serial.println("GPRMB");
    j_MAX = 15;  // number of Words in NEMA Sentence
    Parse_Sentence();
    for(int j = 0; j<j_MAX; j++)
     {
      data_RMB[j] = data_IN[j];  //see void Parse Sentence
      if(print_RMB)Serial.println(data_IN[j]);
     } 
     
    // data_RMB[0] Header
    // data_RMB[1] Data Status
    // data_RMB[2] Cross track error
    // data_RMB[3] Cross track error L/R
    // data_RMB[4] Waypoint Origin
      Origin_Waypoint = data_RMB[4];
      if(Origin_Waypoint == "") { Origin_Waypoint = "NO Origin"; }
    //  Origin_Waypoint.toCharArray(ETdata.SD_Origin_Waypoint,11); 
         
    // data_RMB[5] Destination Waypoint
      Waypoint_next = data_RMB[5];
         if(Waypoint_next == "") { Active_waypoint = "NO WPT"; }
         //Waypoint_next.toCharArray(ETdata.SD_Active_waypoint,11);
     Waypoint_Age = millis();    
   if(Anticipate_Turns == 0 && Number_of_waypoints == 0) // When anticipate turns is 1 and number of waypoints > 0 the destination
      // lat and lon are taken from RTE and WPL in ANTICIPATE TURNS.  Recalculating them here screws up anticipate turns                                                        
   {
    // data_RMB[6] Destination Latitude
     To_Degrees(data_RMB[6]);
     Lat_destination = Lat_Lon_Deg; // value returned from function 
     if(data_RMB[7] == "S") Lat_Waypoint = - Lat_Waypoint;
     //data_RMB[7]; // Lat N/S
   
    // data_RMB[8] Destination Logitude
     To_Degrees(data_RMB[8]);
     Lon_destination = Lat_Lon_Deg; // value returned from function
     if(data_RMB[9] == "W")Lon_destination = -Lon_destination;
     //data_RMB[9]; // Lon E/W;
   }
    // data_RMB[10] Destination Range
      string1 = data_RMB[10];
      NEMA_TO_FLOAT(3); //string1 is the input, 2 is the number of decimal positions, returns float3
      Range_Destination = float3; 
     // ETdata.SD_Range_Destination = Range_Destination; // Changed this to facilitate Anticipated Turns which range gets sent to ET data will be set in void Loop()
        
   //  data_RMB[11] Destination Bearing
      string1 = data_RMB[11];
      NEMA_TO_FLOAT(1); //string1 is the input, 2 is the number of decimal positions, returns float3
      Bearing_to_destination = float3; 
     // ETdata.SD_Bearing_to_destination = Bearing_to_destination; //Changed this to facilitate Anticipated Turns which range gets sent to ET data will be set in void Loop(
    
    // data_RMB[12] Velocity towards Destination
    // string1 = data_RMB[12];
     // NEMA_TO_FLOAT(1); //string1 is the input, 2 is the number of decimal positions, returns float3
     // Velocity_towards_destination = float3;
    //  ETdata.SD_Velocity_towards_destination = Velocity_towards_destination;;
    // data_RMB[13] Arrival Alarm
    // data_RMB[14] Fix Status

    
   
    
/*
     Serial.println();
     Serial.print("Lat Destination = ");
     Serial.println(Lat_destination,4);
     Serial.print("Lon Destination = ");
     Serial.println(Lon_destination,4);
*/
      if(print_RMB) {PRINT_RMB();}      
                
   }  //end of void GPRMB() case
                 
     
/********************* PRINT RMB **************************************************/  
    
   void PRINT_RMB()
     {
                 Serial.println();
                 Serial.println("---------------");    

                 Serial.print("Header: ");  
                 Serial.println(data_RMB[0]);
                 
                 Serial.print("Data Status: ");    
                 Serial.println(data_RMB[1]);
                 
                 Serial.print("Cross Track Error: ");              
                // Serial.println(XTE);
                Serial.println(data_RMB[2]);
                 
                 Serial.print("Error L or R: ");  
                // Serial.println(XTE_LR);
                Serial.println(data_RMB[3]);
                 
                 Serial.print("Origin Waypoint: ");
                // Serial.println(Origin_Waypoint);
                Serial.println(Origin_Waypoint);
                 
                 Serial.print("Destination Waypoint: ");
                // Serial.println(Waypoint_next);
                Serial.println(Waypoint_next);
                 
                 Serial.print("Destination Latitude: ");
                 Serial.println(Lat_destination,4);
                 
                 Serial.print("Latitude N/S: ");
                 Serial.println(data_RMB[7]);
                 
                 Serial.print("Destination Logitude: ");      
                 Serial.println(Lon_destination,4);
                 
                 Serial.print("Longitude E/W: ");    
                 Serial.println(data_RMB[9]);
                 
                 Serial.print("Range to Destination: ");              
                 Serial.println(Range_Destination,3);
                 
                 Serial.print("True Bearing to Destination: ");   
                // Serial.println(Bearing_to_destination);
                 Serial.println(Bearing_to_destination);
                 
                 Serial.print("Velocity towards Destination: ");  
                 Serial.println(Velocity_towards_destination);
                 
                 Serial.print("Arrival Alarm, A = Arrived, V= Not Arrived: ");   
                 Serial.println(data_RMB[13]);
                 
                 Serial.print("Fix Status A, D or V: ");   
                 Serial.println(data_RMB[14]);
                                   
            Serial.println("---------------");   
        }  // end void PRINT_RMB   
#endif 
