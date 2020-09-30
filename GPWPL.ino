#if GPS_Used == 1
 void Get_GPWPL()
 {
   //  This Tab/Routine captures the waypoints sent by the $GPWPL sentence and stores the data in a serie of arrays
   // It also calulates the bearing and range between waypoints, also it computes from the bearings what the next turn
   // will be and stores the data in indexed matricies.  The Index is the same as the Route[i] (list of waypoints in the route
   
    j_MAX = 6;  // number of Words in NEMA Sentence
    Parse_Sentence();
    for(int j = 0; j<j_MAX; j++)
     {
      data_WPL[j] = data_IN[j];  //see void Parse Sentence
      if(print_WPL)Serial.println(data_IN[j]);
     } 
   //    *************  PROCESS DATA  **********************  
 
        // data_BOD[0], header, Processed in Get_Sentence()
        // data_BOD[1],Waypoint Lattitude
         
     To_Degrees(data_WPL[1]);
     Lat_Waypoint = Lat_Lon_Deg; // value returned from function
     if(data_WPL[2] == "S") Lat_Waypoint = - Lat_Waypoint;
     // data_WPL[2] = Lat N/S, no processing
      
     To_Degrees(data_WPL[3]);
     Lon_Waypoint = Lat_Lon_Deg; // value returned from function
     if(data_WPL[4] == "W") Lon_Waypoint = - Lon_Waypoint;
    // data_WPL[5] = Waypoint name
       Waypoint_name = data_WPL[5];  
      
      //  STORE DATA IN ARRAY UISING ROUTE INDEX
     for (int i = 0; i< Number_of_waypoints; i++) 
     {
       if (data_WPL[5] == Route[i])
       {
         
         Waypoint_Lat[i] = Lat_Waypoint;
         Waypoint_Lon[i] = Lon_Waypoint;
         if(i>0) // compute range and bearing from previous waypoint, skip fo first waypoint
           { 
              Range_Bearing_Between_Points(i, Waypoint_Lat[i-1], Waypoint_Lon[i-1], Waypoint_Lat[i], Waypoint_Lon[i]);
              
              Next_Turn[i-1] = Waypoint_Bearing_From[i] - Waypoint_Bearing_From[i-1];
                   if(Next_Turn[i-1] > 180) Next_Turn[i-1] = 360 - Next_Turn[i-1]; // keep angle < 180 with clockwise as positive
                   if(Next_Turn[i-1] < -180) Next_Turn[i-1] = -360 - Next_Turn[i-1];
           }
                      
        int1 = i;  //used to print stored matrix data

        break;
       }
       //  capture the case where first goto waypoint is the first waypoint in the route, in which case 
       // the bearing from the first waypoint is the bearing from GPBOD
       if( Waypoint_next == Route[0])
       {
         Waypoint_Bearing_From[0] = Bearing_origin_to_destination;
        // Waypoint_Range_From[0] = 
       }  // end if waypoint next == first waypoint (Route[0]
       
       if (Waypoint_next == Route[i]) NEXT_TURN = Next_Turn[i];
       if (Active_waypoint == "NO WPT") NEXT_TURN = 0.0;
//       ETdata.SD_NEXT_TURN = NEXT_TURN;
     } // end for 
     
      if(int1 == Number_of_waypoints -1){       
       if(print_WPL) PRINT_WPL();
      }
                        
 }  // END GET GPWPL
 
 /***************************************************************************/
 
   void PRINT_WPL()
 {  
    Serial.println();
    Serial.println("---------------");   
   
        Serial.print("Waypoint, ");  Serial.print("WPL Lat, "); Serial.print("WPL Lon, "); 
        Serial.print("Bearing From Previous, "); Serial.print("Range From Previous, "); Serial.println("Next Turn, ");
       
        for (int i = 0; i< Number_of_waypoints; i++) 
         {
          Serial.print(Route[i]); Serial.print(",  ");
          Serial.print( Waypoint_Lat[i],8); Serial.print(",  ");  
          Serial.print( Waypoint_Lon[i],8); Serial.print(",  ");
          Serial.print(Waypoint_Bearing_From[i],2);  Serial.print(",  ");
          Serial.print(Waypoint_Range_From[i],1); Serial.print(",  ");
          Serial.println(Next_Turn[i],1);  
         }
       Serial.print("Next Turn, "); Serial.println(NEXT_TURN);
          
           // Serial.print("WPL Lat ");  Serial.print(Lat_Waypoint,4);
                 
            // Serial.print("WPL Lat  N/S ");    
                // Serial.println(data_WPL[2]);  
           //   if (data_WPL[2] == "N") Serial.println ( "   NORTH");   
                 
          //  Serial.print("WPL Lon ");   Serial.print(Lon_Waypoint,4);
                 
           // Serial.print("WPL Lon  E/W ");    
               //  Serial.println(data_WPL[4]);  
            //  if (data_WPL[4] == "W") Serial.println ( "  WEST");     
   }
#endif   
