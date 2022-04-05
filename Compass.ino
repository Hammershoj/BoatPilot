void Init_Compass()
{
 #if Compass == 1  /* Initialize Bosch BNO055 sensor */
    if(!bno055.begin())   /* There was a problem detecting the BNO055 ... check your connections */
      {
        lcd.setCursor(0,0);
        lcd.print(" No BNO055 detected");
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        compass_connected = false;
        if (SoftSerial1.available() > 0) 
          {
            lcd.setCursor(0,1);
            lcd.print(" Using NMEA input ");
            Serial.println("Using NMEA compass input instead");
          }
        else 
          {
            lcd.setCursor(0,1);
            lcd.print(" No NMEA connected");
            lcd.setCursor(0,2);
            lcd.print("Not able to run AP");
            while(1);
          }
        }
    else
      {  
        //  Get and restore BNO Calibration offsets   
        BNO_RestoreCal();
        delay(100);
        bno055.setExtCrystalUse(true);
        bno055.setMode(bno055.OPERATION_MODE_NDOF);
        compass_connected = true;
      }
  #endif  // Compass == 1
 
  #if Compass == 2  /* Initialize Bosch BNO08x sensor */
    Wire.begin();
    Wire.setClock(400000); //Increase I2C data rate to 400kHz

    Serial.println(F("Magnetometer enabled"));
    Serial.println(F("Output in form x, y, z, in uTesla"));
        
    if(!bno08x.begin())   /* There was a problem detecting the BNO08x ... check your connections */
      {
        lcd.setCursor(0,0);
        lcd.print(" No BNO08x detected");
        Serial.println("Ooops, no BNO08x detected ... Check your wiring or I2C ADDR!");
        compass_connected = false;
        if (SoftSerial1.available() > 0) 
          {
            lcd.setCursor(0,1);
            lcd.print(" Using NMEA input ");
            Serial.println("Using NMEA compass input instead");
          }
        else 
          {
            lcd.setCursor(0,1);
            lcd.print(" No NMEA connected");
            lcd.setCursor(0,2);
            lcd.print("Not able to run AP");
            while(1);
          }
        }
    else
      {  
        bno08x.enableMagnetometer(50); //Send data update every 50ms
        compass_connected = true;
      }
  #endif  // Compass == 2
}

void BNO055()
{
  float Kbr = .3; // bearing rate low pass filter, 0 is no filtering.
  /* Get a new sensor event */
  sensors_event_t event;
  bno055.getEvent(&event);
  heading = event.orientation.x;
  roll = -event.orientation.y;  // in degrees
  pitch = -event.orientation.z;

  AP_Compass_Correction();  // compute true heading

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
    //Serial.print("Heading: "); Serial.print(heading); Serial.print(" Roll: "); Serial.print(roll);  Serial.print(" Pitch: "); Serial.print(pitch); Serial.print("  BearingRate: "); Serial.println(bearingrate);
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
    // OLED
      /* Get a new sensor event */ 
    display.clearDisplay();

    display.setTextSize(3);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(20, 10);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    if(heading < 10)
      display.print("00");
    if(heading < 100 && heading >= 10)
      display.print("0");
    display.println(heading, 1);
    // second line Heading to steer
      display.setCursor(20, 42);     // Start at top-left corner
    if(heading_to_steer < 10)
      display.print("00");
    if(heading_to_steer < 100 && heading_to_steer >= 10)
      display.print("0");
    display.println(heading_to_steer, 1);
    display.display();
    // OLED end
}  // end BNO055


void BNO08x()
{
  if (bno08x.dataAvailable() == true)
  {
     heading = bno08x.getMagX();
     roll = bno08x.getMagY();
     pitch = bno08x.getMagZ();
     byte accuracy = bno08x.getMagAccuracy(); 
 
     Serial.print(heading, 2);
     Serial.print(F(","));
     Serial.print(roll, 2);
     Serial.print(F(","));
     Serial.print(pitch, 2);
     Serial.print(F(","));
     Serial.print(accuracy);
     Serial.print(F(",")); 
     Serial.println();
  }
  
  AP_Compass_Correction();  // compute true heading

}  // end BNO08x

void AP_Compass_Correction()
{
  #if Compass > 0 
    Magnetic_Variation = MagVar_default;
    if (GPRMC_fix) Magnetic_Variation = MagVar;
    heading = heading + Magnetic_Variation;
    if(heading < 0) heading = 360 + heading; //already a minus, convert to 0-360
    if(heading > 360) heading = heading -360;  //these corrections need if calibration result runs over or under 360
  #endif  
}  // End AP_Compass_Correction()

  /* Display calibration status for each sensor. */
 void BNO055_Get_Cal(){
  uint8_t system, gyro, accel, mag = 0;
  bno055.getCalibration(&system, &gyro, &accel, &mag);
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
    bno055.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
       // Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
       lcd.setCursor(0,0);
       lcd.print("No Comp calib data");
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
        lcd.setCursor(0,0);
        lcd.print ("Loading comp cal     ");
        Serial.print("Cal data:"); displaySensorOffsets(calibrationData);
        bno055.setSensorOffsets(calibrationData);
        delay(1000);
        //Serial.println("\n\nCalibration data loaded into BNO055");
        lcd.setCursor(0,0);
        lcd.print("Comp cal data loaded");
        foundCalib = true;
        Serial.print("Calibration data loaded for sensor type: ");
        Serial.println(sensor.type);
    }
//  get and restore BNO calibration


}  // end BNO Restore Cal
/*************************************************************/
void BNO_SaveCal()
{
    adafruit_bno055_offsets_t newCalib;
    bno055.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    //Serial.println("\n\nStoring calibration data to EEPROM...");
    lcd.clear();
    lcd.print("Storing Calibration");
    eeAddress = 0;
    sensor_t sensor;
    bno055.getSensor(&sensor);
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


