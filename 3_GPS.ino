#if GPS_Used == 1
/*  GPS TAB INCLUDES
    void GET_sentence() it reads the NEMA sentence.
    void Checksum_calc() computes a check sum on NEMA sentence to verify data integrity. 
    void Get_GPRMC() parses the NEMA sentence and looks for the GPRMC sentence.
    void Get_GPAPB() parses the NEMA sentence and looks for the GPRMC sentence.
    Get_sentence is called from Get_GPRMC and Get_GPAPB so program can alternate between the two GPS sentences.
    GPS Reading based on code by Igor Gonzalez Martin. 05-04-2007 igor.gonzalez.martin@gmail.com
          English translation by djmatic 19-05-2007
    Modified for this Application by Jack Edwards
 */


void GET_sentence() {
    // long loop_time=millis();
     //word_count = 1; // word count counts the commas add one for the star
  if (bufpos > buffer_length -1) Reset_buffer(); //if buffer length exceeded w/o finding byteGPS = 13 toss buffer and start again hope eliminate buffer getting garbage from RF
  if (Serial_GPS.available() > 0){
      //static int bufpos = 0;
      //Serial.println("We are getting some GPS data...");
      byteGPS = Serial_GPS.read();
      if(byteGPS !=13){
        gps_buffer[bufpos] = byteGPS;
        if(byteGPS == ',') word_count_temp = word_count_temp+1;      
        if (print_NEMA) Serial.write(byteGPS);                                                
        bufpos++;
      } // if byteGPS != 13
      else { Process_GPS_Data();
      }
    }
    else {
      //Serial.println("No GPS data...");
    }
}     

void Process_GPS_Data(){
     checksum_status= false;   
     NEMA_sentence=false;
     gps_buffer[bufpos] = '\0';  // terminate buffer with null character       
     GPSheader = "";
     //Serial.println("Process GPS data...");
     for (int i=4;i<7;i++){           
       GPSheader = GPSheader + gps_buffer[i];
     } // end for i= 1,7
     
     // Clear rest of buffer
       for (int i=bufpos+1;i<buffer_length;i++){      
         gps_buffer[i]=' '; 
          bufpos = 0;                 
         }
      word_count = word_count_temp;
      word_count_temp = 1; //reset for next sentence
         if(print_GPS_buffer) Serial.print(gps_buffer);
       //Serial.println(); Serial.print(GPSheader); 
                            
      if (GPSheader == "RMC"){ 
        Get_GPRMC();
        return;   
     } 
         
     if (GPSheader == "APB") {         
        Get_GPAPB();
        return;  
     } 
                    
     if (GPSheader == "RMB"){  
        Get_GPRMB();
        return; 
     }  

//   version G3_V2comment out BOD, WPL, RTE to speed other seentences for reliability 9/21/15 
// version H2 changed to using "Use_CTS" to skip these statements if using CTS from $GPAPB like my garmin gpsmap 740s       
       
     if(Use_CTS == 0)
   {  
      if (GPSheader == "BOD"){  
          Get_GPBOD();
          return;              
      }   
     
      if (GPSheader == "WPL"){
          Get_GPWPL();
          return;              
      }     
      
        if (GPSheader == "RTE"){
          Get_GPRTE();
          return;              
      }     
   }  // end if(Use_CTS ==0 )
       NewData=false;
       Reset_buffer();   
  }
   
void Reset_buffer(){
  for (int i=0;i<buffer_length;i++){      
     gps_buffer[i]=' '; 
     bufpos = 0;
  }
}

    void Checksum_calc(){
      
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
                if (gps_buffer[x] == '$'){
                                        index=x;}
                                      break;}
                                     //   Serial.print("index = ");  // diagnostic may be commented out
                                      //  Serial.println(index);  // diagnostic may be commented out
                                    for(int x=index+1; x<100; x++){    
                                        if(gps_buffer[x]=='*'){ 
          checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...     
                                        
                                          break; 
          }else{ 
          checksum ^= gps_buffer[x]; //XOR the received data... 
                                       // Serial.print(gps_buffer[x]);  // this should = gps_buffer less $ in and * at end which is data to checksum
          }}
               
          checksum_status = false;
          if(checksum_received == checksum){ 
                                        checksum_status=true; 
                                          }
                                       
                                     /***********************************************/
                                     // this section added to diagnose checksum and is no longer needed
                                       if(print_checksum) { 
                                         //Checking checksum Convert hex number to hex decimal representation i.e. 15 = F
                                            // Lines below are diagnostic and may be commented out   
                                                 temp = checksum_received/16;
                                                 temp2 = checksum_received-temp*16;
                                                 temp4 = String(temp2);
                                                 //temp3 = temp*10 + temp2;
                                                     if(temp2 == 10){temp4="A";}
                                                      if(temp2 == 11){temp4="B";}
                                                      if(temp2 == 12){temp4="C";}
                                                      if(temp2 == 13){temp4="D";}
                                                      if(temp2 == 14){temp4="E";}
                                                      if(temp2 == 15){temp4="F";}
                                                      temp5 = String(temp);
                                                      temp5 += temp4;
                                        
                                        
                                          //printing checksums for a check 
                                        Serial.print("checksum received hexadecimal = "); 
                                        Serial.println(temp5);
                                        Serial.print("checksum received decimal =");
                                        Serial.println(checksum_received);
                                        Serial.print("checksum calculated = "); 
                                        Serial.println((checksum));
                                        Serial.println("***********************");
                                       }  //  end if print_checksum
                                       
                                       /****************************************************/
                                        
        }    // End of Checksum subroutine
#endif // end if GPS_Used == 1        
