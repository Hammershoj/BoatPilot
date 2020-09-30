#if RF24_Attached == 1
 /*  RADIO RF24 */
 // based on formatted data sketch
 /* 
Sending formatted data packet with nRF24L01. 
Maximum size of data struct is 32 bytes.
 contributted by iforce2d

The wire numbers listed are using a ribbon wire and a two row ribbon wire connector
1 - GND, wire 2
2 - VCC 3.3V !!! NOT 5V, wire 1
3 - CE to Arduino pin 9, wire 4
4 - CSN to Arduino pin 10, wire 3 
5 - SCK to Arduino pin 13 for Uno, 52 on Mega, wire 6
6 - MOSI to Arduino pin 11 for Uno, 51  on Mega, wire 5
7 - MISO to Arduino pin 12 for Uno,  50 on Mega, wire 8
8 - UNUSED, wire 7
*/

void sendData1() 
{
  //int tmp1 = int(Wind_Dir); 
  /*When I make this assignment directly inside the data structure for  some reason I don't 
  * understand the value of Wind_Dir gets reset to 0, not every time though. ditto for Wind_Speed.  making the conversion 
  * outside of the data structure seems to fix the problem. The other float parameters do't seem to be affected though need to 
  * search forum for answer. turn serial prints on to see effect
  */
  //int tmp2 = int(Wind_Speed);
// Serial.println(sizeof(RF_DATA));
 Active_waypoint.toCharArray(RFdata.RFD_text,8);
 RFdata.RFD_int1 = int(heading);
 RFdata.RFD_int2 = int(heading_to_steer);
// if(Steering_Mode == 4) RFdata.RFD_int2 = int(wind_to_steer);
 RFdata.RFD_int3 = int(course);
 RFdata.RFD_int4 = int(course_to_steer);
 RFdata.RFD_int5 = byte(Steering_Mode);
 RFdata.RFD_int6 = int(bearingrate);   
 RFdata.RFD_int7 = int(Waypoint_Bearing_From[WPT_index]); //BOD
 RFdata.RFD_int8 = int(Bearing_to_destination_by_LatLon);
 RFdata.RFD_int9 = byte(MSG);
 RFdata.RFD_int10 = bnoCAL_status;
 RFdata.RFD_int11 = int(Wind_Dir);
 RFdata.RFD_int12 = int(Wind_Speed); 
 if (Use_CTS == 1) RFdata.RFD_int13 = 0;
 else RFdata.RFD_int13 = int(XTE);

 radio.stopListening(); // stop listening so we can send data 
 radio.powerUp();
 delay(3);
 radio.write(&RFdata, sizeof(RF_DATA));  
 radio.startListening(); // resume listening
/* 
 RFdata.RFD_set = 2;
 Mode.toCharArray(RFdata.RFD_text,8);
// RFdata.RFD_float1 = XTE;
 RFdata.RFD_float1 = bearingrate;  // temporary displays bearing rate on RF remote where XTE is programed
 RFdata.RFD_float2 = Bearing_origin_to_destination;
 RFdata.RFD_float3 = Bearing_to_destination;
 RFdata.RFD_float4 = MSG;
 #if Compass == 1
 RFdata.RFD_float5 = float(bnoCAL_status);
 #endif
 radio.stopListening(); // stop listening so we can send data 
 radio.write(&RFdata, sizeof(RF_DATA));  
 radio.startListening(); // resume listening
 */
}
/********************************/
/*
void sendData2() 
{
// test data comment out
SOG = 6.2;
XTE = 123;
Bearing_origin_to_destination = 234;
Bearing_to_destination = 245;
Range_Destination = 2.3;
course_to_steer = 241;
// end test data
 RFdata2.RFdata_set = 2;
 RFdata2.SOG = SOG;
 RFdata2.XTE = XTE;
 RFdata2.Bearing_origin_to_destination = Bearing_origin_to_destination;
 RFdata2.Bearing_to_destination = Bearing_to_destination;
 RFdata2.Range_Destination = Range_Destination;
 RFdata2.course_to_steer = course_to_steer;


 radio.stopListening(); // stop listening so we can send data 
 radio.write(&RFdata2, sizeof(RF_DATA2));
  
 radio.startListening(); // resume listening
}
*/
/*********************************************/

void Recv_Data()
{  //Serial.println(" test Radio rec interrupt  ");
  //delay(5);
       if ( radio.available()) 
  {
   // Serial.println ("radio Available");
    radio.read( &KeyIn2, sizeof(KeyIn2) );
    //Serial.print("KeyIn2 = ");  Serial.println(KeyIn2);
    KeyPressed(KeyIn2);  
   #if Board == Teensy
     lcd.init();
     LCD();
     sendData1();
   #endif   
  }   
}
/*******************/

#endif

