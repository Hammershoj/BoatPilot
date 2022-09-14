void Init_Compass()
{
  #if Compass == 2  /* Initialize Bosch BNO08x sensor */
    Wire.begin();
    Wire.setClock(400000); //Increase I2C data rate to 400kHz

    Serial.println(F("Magnetometer enabled"));
    Serial.println(F("Output in form x, y, z, in uTesla"));
        
    if(!bno08x.begin_I2C(0x4B))   /* There was a problem detecting the BNO08x ... check your connections */
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
        //bno08x.calibrateAll();
        Serial.println("BNO08x Found!");

        for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
          Serial.print("Part ");
          Serial.print(bno08x.prodIds.entry[n].swPartNumber);
          Serial.print(": Version :");
          Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
          Serial.print(".");
          Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
          Serial.print(".");
          Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
          Serial.print(" Build ");
          Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
        }
        if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
          Serial.println("Could not enable magnetic field calibrated");
        }
        compass_connected = true;
      }
  #endif  // Compass == 2
}


void BNO08x()
{
     float heading_read = 0;
     if (heading_avg_n>=5) {
        heading_avg = heading_avg/heading_avg_n;
        Serial.print(" Avg:");
        Serial.println(heading_avg);
        if (abs(heading-heading_avg))
        heading = heading_avg;
        AP_Compass_Correction();
        heading_avg = 0;
        heading_avg_n = 0;
      }
     else {
      if (bno08x.getSensorEvent(&sensorValue))
        {
           float x = sensorValue.un.magneticField.x;
           float y = sensorValue.un.magneticField.y;
           float z = sensorValue.un.magneticField.z;
           heading_read = atan2(y,x) * (180/PI);
           if(heading_read < 0) heading_read = 360 + heading_read;
           if(heading_read > 360) heading_read = heading_read - 360;
        }   
      heading_avg = heading_avg + heading_read;
      heading_avg_n++;
     }
}  // end BNO08x

void AP_Compass_Correction()
{
  #if Compass == 2 
    Magnetic_Variation = MagVar_default;
    if (GPRMC_fix) Magnetic_Variation = MagVar;
    heading = heading + Magnetic_Variation;
    if(heading < 0) heading = 360 + heading; //already a minus, convert to 0-360
    if(heading > 360) heading = heading -360;  //these corrections need if calibration result runs over or under 360

  #endif  
}  // End AP_Compass_Correction()