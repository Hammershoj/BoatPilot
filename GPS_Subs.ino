  #if GPS_Used == 1
/******************* PARSE SENTENCE *************************/

void Parse_Sentence()
{
  //Serial.println("Parse Sentence");
  char comma = ',';
     char star = '*';
     int last_k = 0;
     int k_start = 0;
     Word_count = 0;
     
   //  j_Max = 14; // number of words for NEMA type set j_MAX in calling subroutine
     
       Checksum_calc();   // checksum calculation subroutine
        if (checksum_status)
        {
          NEMA_sentence = true;
           
      for (int j=0; j<j_MAX + 1; j++)
      {data_IN[j] = "";
      }
              
                for (int j=0; j<j_MAX; j++)
                {  
                       for(int k = k_start; k < buffer_length; k++)
                       {   
                              data_IN[j] = data_IN[j] + gps_buffer[k]; 
                              last_k = k; 
                              if (gps_buffer[k] == comma || gps_buffer[k] == star )
                               {
                                 data_IN[j] = data_IN[j].substring(0,data_IN[j].length() - 1); //cuts off last character which is comma or star
                                 Word_count = Word_count +1;                                
                                 break; 
                               } 
                                
                       }  // end for k
                           //Serial.println(data_IN[j]); 
                        k_start = last_k +1;                        
                }// end for j  
      } // end for checksumi 
}  // end Parse_Sentence
/******************  CONVERT LAT/LON TO DEGREES  *************************/

 void To_Degrees(String Lat_Lon)
{

  // NOTE ASSUMES NORTH LATITUDE AND WEST LOGITUDE ARE POSITIVE NEED TO ADD HANDLING FOR SOUTH LATITUDE AND EAST LOGITUDE (treat them as negative
  
double Degrees;
double Minutes;
double Decimal;

int N1;
// if LatLon_decimals is 0 in user input this will auto detect number of decimals or manually set the number of decimals in user input
if(LatLon_decimals == 0 && Detect_LatLon_decimals == 0){ 
 N1 = Lat_Lon.length() - Lat_Lon.indexOf('.') -1;
 //Detect_LatLon_decimals = 1; This is used to only detect number of decimals once to save time 
}
else N1 = LatLon_decimals; // 
 //Serial.print("number of decimals = "); Serial.println(N1);
 //int N1 = 4; // number of characters after decimal (precision of lat lon sentence
 int N2 = N1+1; // length - n2 is position after ending position for minutes counting the first position as 0 = N1 + 1
 int N3 = N1+3; // lenghth - n3 is starting position for minutes = N1 + 3
 int N4 = pow(10,N1); // divisor for decimal part of minutes to convert interger to decimal = 10^N1

// Lat/Lon have form dddmm.pppp(p, ddd is degrees (1 to 3 characters, mm is min, and ppp(p) is decimal minutes 3 o 4 characters

//for (int i=0;i<16;i++)  char_buf[i]=' ';
  //Serial.print("Lat_Lon "); Serial.println(Lat_Lon);
string1 = Lat_Lon.substring(Lat_Lon.length() - N1);// decimal part of Lat Lonchar_buf[] = "";
string1.toCharArray(char_buf,16);
long1 = strtol(char_buf,&pEnd,10);
Decimal = double(long1)/N4; // convert integer to decimal
 //Serial.print("Decimal "); Serial.println(Decimal,6);
string1 = Lat_Lon.substring(Lat_Lon.length()-N3, Lat_Lon.length()-N2); // gets minutes Note start is inclusive, stop is exclusive (position it stops before) see arduino substring
string1.toCharArray(char_buf,16);
long1 = strtol(char_buf,&pEnd,10);
Minutes = double(long1);
  //Serial.print("minutes "); Serial.println(Minutes,3);
string1 = Lat_Lon.substring(0, Lat_Lon.length()-N3); // gets degrees
string1.toCharArray(char_buf,16); 
long1 = strtol(char_buf,&pEnd,10);
Degrees = double(long1);
  //Serial.print("Degrees "); Serial.println(Degrees,8);
Lat_Lon_Deg = Degrees + Minutes/60.0 + Decimal/60.0;
  //Serial.print("Lat_Lon_Deg "); Serial.println(Lat_Lon_Deg,8);
} // end To_Degrees

/************************************************************************/

    void Range_Bearing_Between_Points(int j, float Lat_from, float Lon_from, float Lat_to, float Lon_to)
    {
      float Dx1, Dy1;
      float bearing;
      float range;
      
      
      Dx1 = cos(Lat_from*PI/180)*degrees_to_feet*(Lon_to - Lon_from); 
      Dy1 = degrees_to_feet*(Lat_to - Lat_from);
      bearing = (180/PI) * atan2(Dx1 , Dy1); // note 180/PI*atan2 returns -180 to +180 accounts for sign Dx and Dy
      if(bearing < 0 ) bearing = 360 + bearing; 
      range = sqrt(Dx1*Dx1 + Dy1*Dy1);
      Range_and_Bearing[0] = range;
      Range_and_Bearing[1] = bearing;
      if(j>=0)  // This routine was set up to calculate the bearing from waypoint to waypoint.  To use it to calculate range and bearing
                // from a position to a second position or waypoint use a negative value in the input.  This skips assigning results to watpoints 
      {
        Waypoint_Range_From[j] = range;
        Waypoint_Bearing_From[j] = bearing;
      }
    }  // end void Range and Bearing


/***********************  WAYPOINT  AGE  ****************************/
void Waypoint_Current()
{
  /* Waypoint current checks to see if the waypoint next has been updated within a specified amount of time
   *  if not the waypoint next is set to NO WPT. Necessary because nav programs quit sending GPAPB while others send GPAPB 
   *  but set the waypoint next to null,"", which prgram detects but does not detect thatGPAPB is not sent.
   */
  int Waypoint_Age_Max = 6000; // in milli seconds
   if (millis() - Waypoint_Age > Waypoint_Age_Max) 
   {
    Active_waypoint = "NO WPT";
    //Serial.println("Waypoint_Age > 6");    
   }
  // else Waypoint_Age_Old = millis();
}  // end waypoint current

#endif

/******************************/

 int To_Integer(String string1)
{
  int int1;
  string1.toCharArray(char_buf,16);
  int1 = strtol(char_buf,&pEnd,10);
  return int1;
}  // end To_Integer

/******************* CONVERT NEMA INPUT TO FLOATING VARIABLE *****************/

void NEMA_TO_FLOAT(int N)
{  // this needs work not getting decimal part. need new approach so i don't have to identfy how many decimals.
 //string1 input, float3 output  
    string2 = string1.substring(string1.length()- N);// last N characters of data
    string2.toCharArray(char_buf,16);
    long1 = strtol(char_buf,&pEnd,10);
    float1 = float(long1)/pow(10,N);
    //Serial.println();
   // Serial.println(float1);
    string2 = string1.substring(0, string1.length() - N+1);// all but last N+1 characters, includes decimal point
   // Serial.println(string2);
    string2.toCharArray(char_buf,16);
    long1 = strtol(char_buf,&pEnd,10);
    float2 = float(long1);
    //Serial.println(float2);
    float3 = float1 + float2;
}  // END NEMA TO FLOAT

 

