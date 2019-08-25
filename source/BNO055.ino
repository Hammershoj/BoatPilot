#if Compass == 1
void BNO055()
{
float Kbr = .3; // bearing rate low pass filter, 0 is no filtering.
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
heading = event.orientation.x;
roll = -event.orientation.y;  // in degrees
pitch = -event.orientation.z;
// added LCD print here for faster print this is temporary debug info
  /*
   if (Screen == 0){
   lcd.setCursor(0, 1);
   lcd.print("HDG        ");
   lcd.setCursor(4, 1);
   lcd.print(heading,0);
   lcd.setCursor(0, 2);
   lcd.print("BRT        ");
   lcd.setCursor(4, 2);
   lcd.print(bearingrate,0);
   }
   */
   //Serial.print(heading); Serial.print(" "); Serial.println(bearingrate);
  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The processing sketch expects data as roll, pitch, heading */ 
/*
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
*/
  /* Also send calibration data for each sensor. */
/*
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
*/
/**********************************************/
//  ROUTINES FROM BNO055 EXAMPLE rawdata

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  // To get euler angles
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//BEARING RATE TWO WAYS 
 // imu::Vector<3> gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
 // Serial.print("X.gyros "); Serial.print(57.3 *gyros.x()); Serial.print("Y.gyros "); Serial.print(57.3 * gyros.y()); Serial.print("Z.gyros "); Serial.println(57.3 * gyros.z());
  //bearingrate = -ToDeg(float(gyros.z()));
 // bearingrate = -ToDeg((float(gyros.z())*cos(roll) - float(gyros.x())*sin(roll))*cos(pitch) - float(gyros.y())*sin(pitch)); // 
 //The method above works great on the Pololu IMU9 but doesn't work here because of my confusion with xy axes and or the accelerometers not calibrating
 //Testing on a turntable showed the delta_heading/delta_time to work well so will go with that for now
  delta_heading = heading - heading_old;
    if (abs(delta_heading) > 180) // this limits error to < 180 
      {
         if(heading_old < heading) delta_heading = delta_heading - 360;
         if(heading_old > heading) delta_heading = 360 + delta_heading;
      }
  float BR2_DeltaT = float((millis()-bearingrate2_time)/1000.0);     
  bearingrate = delta_heading /BR2_DeltaT;
  bearingrate_smoothed = (1-Kbr) * bearingrate_smoothed + Kbr * bearingrate; // updates 50/sec, 3000/min, 10,000 = 3.3 min avg.
  bearingrate =  bearingrate_smoothed; // should remove bias but during a turn the BR smoothed may get to big
  if (Screen == 1 && Steering_Mode !=4)// Print wind rate when steering mode = 4
  {
    lcd.setCursor(11, 2);
    lcd.print("BRT      ");
    lcd.setCursor(15, 2);
    lcd.print(bearingrate);
  }
  bearingrate2_time = millis();
  heading_old = heading;
// if (counter2 >47){Serial.print(bearingrate); Serial.print(",  "); Serial.print(bearingrate2); Serial.print(",  "); Serial.println( BR2_DeltaT);}
  
  /* Display the floating point data */
 /*
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.println(euler.z());
  //Serial.print("\t\t");
*/
  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */
}  // end BNO055

  /* Display calibration status for each sensor. */
 void BNO055_Get_Cal(){
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  bnoCAL_status = int(system)*1000 + int(gyro)*100 + int(accel)*10 + int(mag); 
  if(Screen == 3){
  lcd.setCursor(0,3); // prints system cal status last 4 digit on line 0 of LCD
  lcd.print("BNO Cal Status      ");
  lcd.setCursor(15,3);
  lcd.print(system),lcd.print(gyro);lcd.print(accel);lcd.print(mag);
  }
 /*
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
*/
}  // End BNO055_Get_Cal


/**************************************************************************/

void BNO_RestoreCal()
{
//  Get and restore BNO Calibration offsets  

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
       // Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
       lcd.print("NO BNO CAL DATA");
       lcd.setCursor(0,1); lcd.print("DO MANUAL CAL");
        delay(2000);
    }
    else
    {
        //Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

      //  displaySensorOffsets(calibrationData);

        //Serial.println("\n\nRestoring Calibration data to the BNO055...");
        lcd.begin(20,4);
        lcd.setCursor(0,0);
        lcd.print ("LOADING BNO DATA");
        bno.setSensorOffsets(calibrationData);

        //Serial.println("\n\nCalibration data loaded into BNO055");
        lcd.setCursor(0,1);
        lcd.print("BNO CAL DATA LOADED");
        foundCalib = true;
        delay(2000);
    }
//  get and restore BNO calibration


}  // end BNO Restore Cal
/*************************************************************/
void BNO_SaveCal()
{
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    //Serial.println("\n\nStoring calibration data to EEPROM...");
    lcd.clear();
    lcd.print("Storing Calibration");
    eeAddress = 0;
    sensor_t sensor;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    //Serial.println("Data stored to EEPROM.");
    DataStored = true;
    Serial.println("\n--------------------------------\n");
    delay(2000);

}  //  end BNO_SaveCal
/********************************************************/

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/****************************************************************/
#endif

