#if GPS_Used == 1
 void Get_GPAPB()
   {
     //Serial.println("GPAPB");
    j_MAX = 16;  // number of Words in NEMA Sentence
    Parse_Sentence();
    for(int j = 0; j<j_MAX; j++)
     {
      data_APB[j] = data_IN[j];  //see void Parse Sentence
      if(print_APB)Serial.println(data_IN[j]);
     } 
   //    *************  PROCESS DATA  **********************  
 
        // data_APB[0], header, Processed in Get_Sentence()
        // data_APB[1], Loran Data, not processed
        // data_APB[2], Loran Data, Not processed
        
        // data_APB[3], Cross Track Error
         // string1 = data_APB[3];
         // NEMA_TO_FLOAT(2); //string1 is the input, 2 is the number of decimal positions, returns float3
        //  XTE = float3;
        //  ETdata.SD_XTE = XTE;
        
        // data_APB[4], XTE L/R
         // if( data_APB[4] == "L"){ XTE=-XTE;}  //R means you are steer right, L means steer left and is therefore negative do this in main PID/GPS
        // XTE_LR = data_APB[4];
      //   XTE_LR.toCharArray(ETdata.SD_XTE_LR,2);

        // data_APB[5], XTE Units
        // XTE_unit = data_APB[5];
       //  XTE_unit.toCharArray(ETdata.SD_XTE_unit,2); 
        
        // data_APB[6], Arrival Alarm Circle
        // data_APB[7], Arrival Alarm Perpendicular
        
        // data_APB[8], Bearing Origin to Destination
          string1 = data_APB[8];
          NEMA_TO_FLOAT(1); //string1 is the input, 2 is the number of decimal positions, returns float3
          Bearing_origin_to_destination = float3;
      //    ETdata.SD_Bearing_origin_to_destination = Bearing_origin_to_destination; 
        
        // data_APB[9], Mag or True
         BOD_MorT = data_APB[9];
   //      BOD_MorT.toCharArray(ETdata.SD_BOD_MorT,2); 
           
        // data_APB[10], Destination Waypoint ID
         Waypoint_next = data_APB[10];
         if(Waypoint_next == "") { Active_waypoint = "NO WPT"; }
       //  Waypoint_next =Waypoint_next.substring(10);
         //Waypoint_next.toCharArray(ETdata.SD_Active_waypoint,11);
        
        
        //NOTE get true bearing to destination from GPRMB  
        // data_APB[11], Bearing present position to destination
      //   string1 = data_APB[11];
      //  NEMA_TO_FLOAT(1); //string1 is the input, 2 is the number of decimal positions, returns float3
       //  Bearing_to_destination = float3;
     //    ETdata.SD_Bearing_to_destination = Bearing_to_destination; 
        
        // data_APB[12], Mag or True
      //   BTD_MorT = data_APB[12];
       //  data_APB[12].toCharArray(ETdata.SD_BTD_MorT,2);
         
        // data_APB[13], Course to Steer
         string1 = data_APB[13];
         NEMA_TO_FLOAT(1); //string1 is the input, 2 is the number of decimal positions, returns float3
         //course_to_steer = float3;
         GPS_course_to_steer = float3;  // change AP_JNE_15.0A so Use_CTS works with single board AP
         
        
        // data_APB[14], Mag or True
        // CTS_MorT = data_APB[14];
        // CTS_MorT.toCharArray(ETdata.SD_CTS_MorT,2);
         
        // data_APB[15], Type of Fix
         if (Word_count == 15)  data_APB[15] = "D"; // NOBELTEC does not send data for data_APB[15]
        
         if(data_APB[15]=="A" || data_APB[15]=="D" && checksum_status){
              GPAPB_fix = true;
              Waypoint_Age = millis();
          }
         else{
              GPAPB_fix= false;}
        // data_APB[16], Fix Validity
         if(GPAPB_fix){
              GPAPB_fix_status = "GPAPB OK";} // used to print for testing
         else{ 
              GPAPB_fix_status= "NO GPAPB";}   
         
 //        ETdata.SD_GPAPB_fix = GPAPB_fix;   
         if(GPAPB_fix) NewData = true; // uncommented in rev H6B 6.11.16 added to BOD also

      if(print_APB) {PRINT_APB();}      
                
   }  //end of void GPAPB() case
                 
   
  /**************************  PRINT APB  ******************************/
  
   void PRINT_APB()
 {  
     Serial.println();
    Serial.println("---------------");   
   
            Serial.print("Header: ");  
                 Serial.println(data_APB[0]);
              
            Serial.print("Loran Data: ");    
                 Serial.println(data_APB[1]);
                 
            Serial.print("Loran Data: ");              
                 Serial.println(data_APB[2]);
          
            Serial.print("Cross Track Error: ");  
                 Serial.println(XTE);
             
             Serial.print("Error L or R: ");
                 Serial.println(XTE_LR);
                 
             Serial.print("Cross track error units: ");
                 Serial.println(XTE_unit);
                
             Serial.print("Arival Alarm Circle: ");
                 Serial.println(data_APB[6]);
                
             Serial.print("Arrival Alarm Perpendicular: ");
                 Serial.println(data_APB[7]);
               
             Serial.print("Bearing Origin to Destination: ");      
                 Serial.println(Bearing_origin_to_destination);
                
             Serial.print("Mag or True: ");    
                 Serial.println(BOD_MorT);
                 
             Serial.print("Destination Waypoint ID: ");              
                 Serial.println(Waypoint_next);
                 
             Serial.print("Bearing Present Position to Destination: ");   
                 Serial.println(Bearing_to_destination);
                
             Serial.print("Mag or True: ");  
                 Serial.println(BTD_MorT);
                 
             Serial.print("Course to Steer: ");   
                 Serial.println(course_to_steer,1);
                       
            Serial.print("Mag or True: ");   
                 Serial.println(CTS_MorT);
                    
            Serial.print("Type of Fix: ");   
                 Serial.println(data_APB[15]);
                 
             Serial.print("GPAPB Fix Validity; ");   
                 Serial.println(GPAPB_fix_status);
                 
             Serial.println();
                  
   }
    
#endif   
