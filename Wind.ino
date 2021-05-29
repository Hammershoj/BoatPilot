#if Wind_Input == 1

void get_Wind()
{
const int print_wind = 1;
float Kwind = .1; // 1 all new data (unfiltered) .2 (5 samples smoothing) Look into Kalman filter
if (count_b > wind_buffer_length -1) Reset_wind_buffer(); //if buffer length exceeded w/o finding byteGPS = 13 toss buffer and start again hope eliminate buffer getting garbage from RF
if (SoftSerial1.available() > 0){
    byteWind = SoftSerial1.read();
    //Serial.println("Reading Wind data from RS232 ");
    if(byteWind !=13)
    {
      Wind_buffer[count_b] = byteWind;    
      if (print_wind) Serial.write(byteWind); 
      //Serial.println(byteWind);
      if(byteWind>127) //I was getting junk data a y with 2 dots over it ASCII char 255 binary 11111111, could not get out of loop, this fixed it 
        {
          Reset_wind_buffer(); 
          return;                                                                                 
        }
      count_b++;
    }   // if byteGPS != 13
    else 
    {
       Wind_buffer[count_b] = '\0';  // terminate buffer with null character             
       // Clear rest of buffer
       for (int i=count_b+1;i<wind_buffer_length;i++)
       {      
         Wind_buffer[i]=' '; 
         count_b = 0;                 
       }  
       Windheader = "";
       for (int i=2;i<7;i++)
       {           
         Windheader = Windheader + Wind_buffer[i];
       } // end for i= 2,7                                
         Serial.print("Header = "); Serial.println(Windheader);           
       if (Windheader == "WIMWV")
       {         
         Get_WIMWV();
         Wind_MAX = max(Wind_Speed,Wind_MAX);
         if(!sw2) Wind_MAX = 0;  // sw2 can be used to reset wind max
         Wind_Avg = (1-Kwind) * Wind_Avg + Kwind * Wind_Dir; // set wind average Kwind at beginning of subroutine with wind update 1 Hz Kwind = .2 should be about 5 sec average
             Serial.print(Wind_Dir); Serial.print(" "); Serial.println(Wind_Avg);
           //  Wind_Differential();
         Reset_wind_buffer();  
         return;  
       }
       if (Windheader == "SDDBT") // Depth
       {  
         Get_SDDBT();
         Reset_wind_buffer();          
       } 
    } // end else
 }  // if (SoftSerial1.available()
} // void get_Wind2

void Reset_wind_buffer()
{  
 for (int i=0;i<wind_buffer_length;i++)
 {      
   Wind_buffer[i]=' ';        
 } 
 count_b = 0;
}

/**************************************************/

void Get_WIMWV()
   {
    const int print_MWV = 1;
    j_MAX = 5;  // number of Words in NEMA Sentence
    Parse_Wind ();
    //Serial.print("Checksum Status "); Serial.println(checksum_status);
    if(checksum_status){  // if checksum is true process data 
    for(int j = 0; j<j_MAX; j++) // Since 
     {
      data_MWV[j] = data_IN[j];  //see void Parse Sentence
     } 
    string1 = data_MWV[1];
    NEMA_TO_FLOAT(1); // this takes the char data that looks like the wind bearing and converts it to a floating point value
    Wind_Dir = float3;  
     //Serial.print("wind Dir in WIMWV "); Serial.println(Wind_Dir,0);            
        // data_MWV[2], R or T Relative or True        
        // data_MWV[3], Wind Speed
    string1 = data_MWV[3];
    NEMA_TO_FLOAT(1); // this takes the char data that looks like the wind speed and converts it to a floating point value
    Wind_Speed = float3;      
        // data_MWV[4], Wind_Speed Units K/M/N N = knots       
      if(print_MWV) {PRINT_MWV();}      
    } // end if checksum_status true     
    //PRINT_MWV();        
   }  //end of void WIMWV() case
                 
  /*******************************************************************/

void Get_SDDBT() //Depth
{
   j_MAX = 4;  // number of Words in NEMA Sentence, for DBT I am only reading 2nd word, first word is header
   Parse_Wind();
   if(checksum_status){
     string1 = data_IN[3];
      NEMA_TO_FLOAT(1); // this takes the char data that looks like the wind bearing and converts it to a floating point value
      Depth = float3;
      //Serial.print("Depth "); Serial.print(Depth,1);
   }
} // End get SDDBT
  /*******************************************************************/

void Parse_Wind()
{
  char comma = ',';
     char star = '*';
     int last_k = 0;
     int k_start = 0;
     //Word_count = 0;          
       Checksum_wind();
       //Serial.print("checksum status "); Serial.println(checksum_status);
         if (checksum_status)
        {
           
      for (int j=0; j<j_MAX + 1; j++)
      {data_IN[j] = "";
      }             
                for (int j=0; j<j_MAX; j++)
                {  
                       for(int k = k_start; k < 80; k++)
                       {   
                              data_IN[j] = data_IN[j] + Wind_buffer[k]; 
                              last_k = k; 
                              if (Wind_buffer[k] == comma || Wind_buffer[k] == star )
                               {
                                 data_IN[j] = data_IN[j].substring(0,data_IN[j].length() - 1); //cuts off last character which is comma or star
                                // Word_count = Word_count +1;
                                Serial.println(data_IN[j]);
                                 break; 
                               }   
                       }  // end for k
                           
                        k_start = last_k +1;                        
                }// end for j  
      } // end for i 
}  // end Parse_Wind

 
  /**************************  PRINT MWV  ******************************/
 
   void PRINT_MWV()
 {  
    // Serial.println();
    Serial.println("---------------");   
   
            Serial.print("Header: ");  
                 Serial.println(data_MWV[0]);
              
            Serial.print("Wind Bearing: ");    
                 Serial.println(Wind_Dir);
                 
            Serial.print("Wind R or T ");              
                 Serial.println(data_MWV[2]);
          
            Serial.print("Wind Speed: ");  
                 Serial.println(Wind_Speed);
             
             Serial.print("Speed Units ");
                 Serial.println(data_MWV[4]);

             Serial.print("Depth ");
                 Serial.println(Depth);
                
             Serial.println();                  
   }

/*************************************************************/

  void Checksum_wind(){    
       /**********************************************
        From garmin.com search support for "how is checksum calculated in NEMA 0183
        The checksum is the 8-bit exclusive OR (no start or stop bits) of all characters in the sentence, including the "," delimiters, 
        between -- but not including -- the "$" and "*" delimiters. The hexadecimal value of the most significant and least significant
        4 bits of the result are converted to two ASCII characters (0-9, A-F) for transmission. The most significant character is transmitted first. 
        Therefore in the routine below the counter starts at 1 to skip "$" in  checksum routine and ends at index of "*".
      ***********************************************/      
            int index=0;                  
                              //  Serial.println("");                          // diagnostic may be commented out
                              //  Serial.print("DATA TO BE CHECK SUMMED  ");   // diagnostic may be commented out
            checksum=0;
            for(int x=1; x<100; x++){  // you have to skip the $ sign and it works if x starts at 1
                if (Wind_buffer[x] == '$'){
                                        index=x;}
                                      break;}
                                     //   Serial.print("index = ");  // diagnostic may be commented out
                                      //  Serial.println(index);  // diagnostic may be commented out
                                    for(int x=index+1; x<100; x++){    
                                        if(Wind_buffer[x]=='*'){ 
          checksum_received = strtol(&Wind_buffer[x + 1], NULL, 16);//Parsing received checksum...     
    // Serial.print("checksum_received = ");Serial.println(checksum_received);                             
                                          break; 
          }else{ 
          checksum ^= Wind_buffer[x]; //XOR the received data... 
                                       // Serial.print(gps_buffer[x]);  // this should = gps_buffer less $ in and * at end which is data to checksum
          }}
   //Serial.print("checksum = "); Serial.println(checksum);     
          checksum_status = false;
          if(checksum_received == checksum){ 
             checksum_status = true;       
   //Serial.print("checksum status "); Serial.println(checksum_status);                                                                          
        }    
  }// End of Checksum subroutine 
 /*
  void Wind_Differential()
  {
   static float delta_Wind, Wind_DeltaT, windrate_smoothed;
    static float Wind_old, windtime_old; 
    const float Kwr = .3;
    
    delta_Wind = Wind_Dir - Wind_old;
 
    if (abs(delta_Wind) > 180) // this limits error to < 180 
      {
         if(Wind_old < Wind_Dir) delta_Wind = delta_Wind - 360;
         if(Wind_old > Wind_Dir) delta_Wind = 360 + delta_Wind;
      }
  Wind_DeltaT = float((millis()-windtime_old)/1000.0);     
  windrate = delta_Wind /Wind_DeltaT;
  windrate_smoothed = (1-Kwr) * windrate_smoothed + Kwr * windrate; // updates 50/sec, 3000/min, 10,000 = 3.3 min avg.
  windrate =  windrate_smoothed; // should remove bias but during a turn the BR smoothed may get to big
  if (Screen == 1 && Steering_Mode == 4)
  {
    lcd.setCursor(11, 2);
    lcd.print("WRT      ");
    lcd.setCursor(15, 2);
    lcd.print(windrate);
  }
   windtime_old = millis();
   Wind_old = Wind_Dir;
 if (counter2 >5){ Serial.println(); Serial.print("Wind rate and bearing rate, "); Serial.print(windrate); Serial.print(" "); Serial.println(bearingrate);}
  } // end wind differential
*/
  #endif
