#if Compass == 0
  void LCDprint()
  {
      lcd.setCursor(0,0);
      lcd.print ("HEAD");
      lcd.setCursor(7,0);
      lcd.print(MAG_Heading_Degrees);
      
      lcd.setCursor(0,1);
      lcd.print ("PITCH");
      lcd.setCursor(7,1);
      lcd.print(ToDeg(pitch));
      //lcd.print(Head_count);
      
      lcd.setCursor(0,2);
      lcd.print ("ROLL");
      lcd.setCursor(7,2);
      lcd.print(ToDeg(roll));
     
      /* 
      lcd.setCursor(0,1);
      lcd.print ("mag X       ");
      lcd.setCursor(7,1);
      lcd.print(magnetom_x);
      
      lcd.setCursor(0,2);
      lcd.print ("mag Y        ");
      lcd.setCursor(7,2);
      lcd.print(magnetom_y);
      */
        
      lcd.setCursor(0,3);
      lcd.print ("YAW");
      lcd.setCursor(7,3);
      lcd.print(ToDeg(yaw));
      
  }  // end void LCD print
#endif 
 
  
