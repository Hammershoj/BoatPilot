 #if GPS_Used ==1
 void Get_GPBOD()
   {
     //Serial.println("GPBOD");
    j_MAX = 7;  // number of Words in NEMA Sentence
    Parse_Sentence();
    for(int j = 0; j<j_MAX; j++)
     {
      data_BOD[j] = data_IN[j];  //see void Parse Sentence
      if(print_BOD)Serial.println(data_IN[j]);
     } 
   //    *************  PROCESS DATA  **********************  
 
        // data_BOD[0], header, Processed in Get_Sentence()
        // data_BOD[1], TRUE Bearing From Start Waypoint to Destination Waypoint
         string1 = data_BOD[1];
          NEMA_TO_FLOAT(1); //string1 is the input, (n) is the number of decimal positions, returns float3
          Bearing_origin_to_destination = float3;
        //  ETdata.SD_Bearing_origin_to_destination = Bearing_origin_to_destination; 
          
        // data_BOD[2], M or T True for BOD[1}
         BOD_MorT = data_BOD[2];
        // BOD_MorT.toCharArray(ETdata.SD_BOD_MorT,2);
         
        // data_BOD[3], Magnetic Bearing From Start Waypoint to Destination Waypoint Not Processed
        // string1 = data_BOD[3];
        // NEMA_TO_FLOAT(1); //string1 is the input, (n) is the number of decimal positions, returns float3
        // Bearing_origin_to_destination = float3;
        // ETdata.SD_Bearing_origin_to_destination = Bearing_origin_to_destination; 
          
        // data_BOD[4], M or T True for BOD[3} NOT Processed
        // BOD_MorT = data_BOD[4];
        // BOD_MorT.toCharArray(ETdata.SD_BOD_MorT,2);
              
        // data_BOD[5], Destination Waypoint ID
         Waypoint_next = data_BOD[5];
         if(Waypoint_next == "") {Active_waypoint = "NO WPT"; }
        // Active_waypoint = Waypoint_next;
        // Active_waypoint.toCharArray(ETdata.SD_Active_waypoint,11); 
         Waypoint_Age = millis();
      // data_BOD[6], Origin Waypoint ID
         Origin_Waypoint = data_BOD[6];
         if(Origin_Waypoint == "") { Origin_Waypoint = "NONE"; }
        // Waypoint_next.toCharArray(ETdata.SD_Waypoint_next,11); 
         NewData = true;  // added in rev H6B this controls when ETdata is sent to the autopilot
      if(print_BOD) {PRINT_BOD();}
                      
   }  //end of void GPBOD() case
                 
    
  /**************************  PRINT BOD  ******************************/
  
   void PRINT_BOD()
 {  
    Serial.println();
    Serial.println("---------------");   
   
            Serial.print("Header: ");  
                 Serial.println(data_BOD[0]);
              
            Serial.print("True Bearing Origin to Destination: ");    
                 Serial.println(Bearing_origin_to_destination);
                 
            Serial.print("Mag or True: ");              
                 Serial.println(data_BOD[2]);
          
            Serial.print("Destination Waypoint: "); 
                 Serial.println(Waypoint_next);
                // Serial.println(Active_waypoint);
             
             Serial.print("Origin Waypoint: ");
                 Serial.println(Origin_Waypoint);
                                   
   }
    
#endif 
