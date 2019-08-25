#if GPS_Used ==1
#if Use_CTS == 0
/***************** CROSS TRACK ERROR CALC *********************/

void Get_Cross_Track_Error()
{
     float angle1;
     //float angle2;  // angle between BOD an course 
    
     Destination_Bearing ();// Note comment this out to use Bearing_to_Destination from GPRMB (less accurate, more robust)
     angle1 = Waypoint_Bearing_From[WPT_index] - Bearing_to_destination_by_LatLon; // change to use WPT_index
     //angle1 = Waypoint_Bearing_From[GPS_WPT_index] - Bearing_to_destination_by_LatLon; 
     //angle1 = Waypoint_Bearing_From[GPS_WPT_index] - Bearing_to_destination;
     if(angle1 > 180) angle1 = 360 - angle1; // keep angle < 180 with clockwise as positive
     if(angle1 < -180) angle1 = 360 + angle1;
     XTE = Range_Destination_by_LatLon * sin(PI*angle1/180.0);// XTE in feet, if XTE positive right of track steer left
     //XTE = Range_Destination * sin(PI*angle1/180.0);// XTE in feet, if XTE positive right of track steer left
     // Serial.print("angle1 "); Serial.print(angle1); Serial.print("  Range destination, "); Serial.print(Range_Destination_by_LatLon); Serial.print(" XTE, ");Serial.println(XTE);
     //angle2 = course - Bearing_origin_to_destination; // positive if Dxte is positive
     //if(angle2 > 180) angle2 = 360 - angle2; // keep angle < 180 with clockwise as positive
     //if(angle2 < -180) angle2 = 360 + angle2;
     //XTE_differential_error = SOG* sin(PI*angle2/180.0); // cross track velocity in knots
     
     if(print_Nav_Data)
     {
      Serial.println();
      Serial.print("Waypoint Next ");
      Serial.println(Waypoint_next);
      Serial.print("WPT_Index "); Serial.println(WPT_index);
      Serial.print("BOD: ");
      //Serial.println(Bearing_origin_to_destination);
      Serial.println(Waypoint_Bearing_From[WPT_index]);
      Serial.print("Angle1 "); Serial.println(angle1);
      Serial.println("WPT Lat Lon followed by current Lat Lon ");
      Serial.print(Current_Waypoint_Lat,5); Serial.print("  "); Serial.println(Current_Waypoint_Lon,5);
      Serial.print(Lat_current,5); Serial.print("  "); Serial.println(Lon_current,5);
      Serial.print("Bearing to Destination ");
      Serial.print(Bearing_to_destination,3);Serial.print("  "); //Serial.println(Bearing_to_destination,3);
      Serial.print("Range: ");
      //Serial.println(Range_Destination,3);
      Serial.println(Range_Destination_by_LatLon,0);  
      Serial.print("XTE: ");
      Serial.println( XTE,2);  
      //Serial.print ("XTE rate ");
      //Serial.println(XTE_differential_error,1);
     }  // end print_Nav_Data
    
}// End Get cross track error

/***************  DESTINATION BEARING  *******************************/
  void Destination_Bearing ()
  {
      float Dx1, Dy1;
      float bearing;
      
      // First get current waypoint lat and lon
/*     for (int i = 0; i< Number_of_waypoints; i++) 
     {
         //if (data_WPL[5] == Waypoint_next)
         if(Route[i]== Waypoint_next)
        {
          //Serial.print(Waypoint_next); Serial.print(" Current_Waypoint_Lat Index "); Serial.println(i);
         Current_Waypoint_Lat = Waypoint_Lat[i];
         Current_Waypoint_Lon = Waypoint_Lon[i];
         break;
        }
     }
*/
     Current_Waypoint_Lat = Waypoint_Lat[WPT_index]; // if anticipate turn true and range < turn range WPT_index = GPS_index +1 ( next waypoint)
     Current_Waypoint_Lon = Waypoint_Lon[WPT_index];
      
      Dx1 = cos(Lat_current*PI/180)*degrees_to_feet*(Current_Waypoint_Lon - Lon_current);
      Dy1 = degrees_to_feet*(Current_Waypoint_Lat - Lat_current);
      bearing = (180/PI) * atan2(Dx1 , Dy1); // note 180/PI*atan2 returns -180 to +180 accounts for sign Dx and Dy
      if(bearing < 0 ) bearing = 360 + bearing;
      Bearing_to_destination_by_LatLon = bearing;
      Range_Destination_by_LatLon = sqrt(Dx1*Dx1+Dy1*Dy1);  //range in feet
      
      //ETdata.SD_Bearing_to_destination = Bearing_to_destination; // same as course to steer

  }  // End Destination Bearing
  /*********************************************************************/

    void ANTICIPATE_TURN()
   {
    // if distance to waypoint less than turn distance this sets Active_waypoint and BOD to the values for the waypoint after waypoint next
    // if(Number_of_waypoints == 0) return; // This line added for GPS60 case where there is a goto waypoint but not a route and no $GPWPL
                 // which results in no WPL lat lon (i. = 0, 0 ) and 6076.1 mile xte so junk WPT next and XTE do know effect with other GPSs
     if( 6076.1 * Range_Destination < Turn_distance)
      {
        if(Anticipated_Turn_Active == false) // to only advance the index once each time anticipated turn is activated
        {
        WPT_index = GPS_WPT_index + 1;
        Anticipated_Turn_Active = true;
        }
      }
      else  // if range not < turn distance
       {
         /*    If we get to a turn we want the index to advance.  After passing the orginal WPT the GPS
         should shift to the next waypoint which is the one the anticipate turn already advanced to.  we don't want the
         anticpated next waypoint to shift back to the old waypoint.  Likewise, when we pass the wapoint the GPS goes to the 
         next waypoint but if we are still < turn distance we don't want the index to advance a second time.
         */
        if( WPT_index == GPS_WPT_index) // i.e. we have passed waypoint and GPS waypoint has advanced to our anticipated waypoint
         {
           Anticipated_Turn_Active = false;
         }
       }        
   }  //  End ANTICIPATE_TURN
   /**************************************************************************************************/    
#endif
#endif
