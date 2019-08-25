#if UseBarometer

void Init_Barometer(){
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();
}

void Read_Barometer(){
  pressure = ps.readPressureInchesHg();
  altitude = ps.pressureToAltitudeFeet(pressure);
  temperature = ps.readTemperatureF();

  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" inHg\ta: ");
  Serial.print(altitude);
  Serial.print(" ft\tt: ");
  Serial.print(temperature);
  Serial.println(" deg F");
}
#endif
