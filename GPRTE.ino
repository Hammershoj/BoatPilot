  #if GPS_Used == 1
  void Get_GPRTE()
  
 {
     static int klast;
     static int kstart;
     RTE_timer = millis();
     int waypoint_count;
   
    j_MAX = word_count ; // problem with RTE is sentece length depends on number of way points, word_count calculted void Get_Sentence()
    Parse_Sentence();
    
    
    for(int j = 0; j<j_MAX; j++)
     {
      data_RTE[j] = data_IN[j];  //see void Parse Sentence 
      if(print_RTE)Serial.println(data_IN[j]); 
     }
     
     Number_of_sentences = To_Integer(data_RTE[1]);
     
     //Current_sentence_number = strtol(data_RTE[2],&pEnd,10)
     Current_sentence_number = To_Integer(data_RTE[2]);
     //  put the sequential sentences into a unified array
     waypoint_count = Word_count-5;
     if(Current_sentence_number == 1)
       {
         kstart = 0;
         klast = waypoint_count;
       }
     else
       {
         kstart = klast ;
         klast = klast + waypoint_count;
       } // end else
       
    if(Current_sentence_number == Number_of_sentences) Number_of_waypoints = klast;  //  First 5 words each sentence are not waypoints
     
     for(int j = 5; j<Word_count; j++)  
       {    
        Route [kstart + j-5 ] = data_RTE[j];
       } 
   
      if(print_RTE) {PRINT_RTE();}
 }  // end get_GPRTE()
 
 /**************************************************************/
 
    void PRINT_RTE()
 {  
    Serial.println();
    Serial.println("---------------");   
    Serial.println("RTE DATA "); 
    Serial.print("Number of sentences "); Serial.println(Number_of_sentences);
    Serial.print("Current Sentence "); Serial.println(Current_sentence_number);
    Serial.print("Word Count "); Serial.println(Word_count);
     for(int j = 0; j<Word_count; j++)
      {   
         Serial.println(data_RTE[j]);
      }
    Serial.print("Number of Waypoints, "); Serial.println(Number_of_waypoints);
    Serial.print("Route Data Array  ");
    for(int j = 0; j< Number_of_waypoints; j++)
    {
      Serial.print(j);
      Serial.print(Route[j]);
      Serial.print(", ");
    }
    if(Current_sentence_number == Number_of_sentences) Serial.println();
    
    
 }
 #endif
