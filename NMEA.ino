
 #if Wind_Input == 1

 byte byteNMEA;
 String NMEAheader;
 const int NMEA_buffer_length = 32;
 char NMEA_buffer[NMEA_buffer_length]; 
 
void get_NMEA(String Header)
{
const int print_NMEA = 1;
float Kwind = .1; // 1 all new data (unfiltered) .2 (5 samples smoothing) Look into Kalman filter
if (count_b > NMEA_buffer_length -1) Reset_NMEA_buffer(); //if buffer length exceeded w/o finding byteGPS = 13 toss buffer and start again hope eliminate buffer getting garbage from RF
if (SoftSerial1.available() > 0){
    byteNMEA = SoftSerial1.read();
    //Serial.println("Reading NMEA data from RS232 ");
    if(byteNMEA !=13)
    {
      NMEA_buffer[count_b] = byteNMEA;    
      if (print_NMEA) Serial.write(byteNMEA); 
      //Serial.println(byteWind);
      if(byteNMEA>127) //I was getting junk data a y with 2 dots over it ASCII char 255 binary 11111111, could not get out of loop, this fixed it 
        {
          Reset_NMEA_buffer(); 
          return;                                                                                 
        }
      count_b++;
    }   // if byteGPS != 13
    else 
    {
       NMEA_buffer[count_b] = '\0';  // terminate buffer with null character             
       // Clear rest of buffer
       for (int i=count_b+1;i<NMEA_buffer_length;i++)
       {      
         NMEA_buffer[i]=' '; 
         count_b = 0;                 
       }  
       NMEAheader = "";
       for (int i=2;i<7;i++)
       {           
         NMEAheader = NMEAheader + NMEA_buffer[i];
       } // end for i= 2,7                                
         Serial.print("Header = "); Serial.println(NMEAheader);           
       if (NMEAheader == "WIMWV")
       {         
         Get_WIMWV();
         Wind_MAX = max(Wind_Speed,Wind_MAX);
         if(!sw2) Wind_MAX = 0;  // sw2 can be used to reset wind max
         Wind_Avg = (1-Kwind) * Wind_Avg + Kwind * Wind_Dir; // set wind average Kwind at beginning of subroutine with wind update 1 Hz Kwind = .2 should be about 5 sec average
             Serial.print(Wind_Dir); Serial.print(" "); Serial.println(Wind_Avg);
           //  Wind_Differential();
         Reset_NMEA_buffer();  
         return;  
       }
       if (NMEAheader == "SDDBT") // Depth
       {  
         Get_SDDBT();
         Reset_NMEA_buffer();          
       } 
    } // end else
 }  // if (SoftSerial1.available()
} // void get_NMEA


void Reset_NMEA_buffer()
{  
 for (int i=0;i<NMEA_buffer_length;i++)
 {      
   NMEA_buffer[i]=' ';        
 } 
 count_b = 0;
}




void Parse_NMEA()
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
                              data_IN[j] = data_IN[j] + NMEA_buffer[k]; 
                              last_k = k; 
                              if (NMEA_buffer[k] == comma || NMEA_buffer[k] == star )
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
}  // end Parse_NMEA
#endif

