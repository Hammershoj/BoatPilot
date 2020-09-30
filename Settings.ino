 /*
Settings TAB is linked to LCD screen 5 and used to change rudder settings
Added 15.06.2019 by cfh
*/

/******************************



void setup()
{
  for (int i = 0; i < 255; i++)
    EEPROM.write(i, i);
}

void loop()
{
}


***********************************/
 

/**********************************************************************/
void Rudder_Change_Mode()
{
  if (RUDDER_MODE ==1 && Change_rudder_mode) { RUDDER_MODE = 0; Change_rudder_mode = false; EEPROM.put(1, RUDDER_MODE);} //   
  if (RUDDER_MODE ==0 && Change_rudder_mode) { RUDDER_MODE = 1; Change_rudder_mode = false; EEPROM.put(1, RUDDER_MODE);} // rudder feed back RFB not availabl  
}  // Rudder_Change_Mode()

void Rudder_Store_at_left()
{
  counts_max = counts;
}  // Rudder_Store_at_left()          

void Rudder_Store_at_zero()
{
   counts_at_zero = counts; 
   counts_min = counts_at_zero -400;
   counts_max = counts_at_zero +400;        
}  // Rudder_Store_at_zero()    

void Rudder_Store_at_right()
{
   counts_min = counts;         
}  // Rudder_Store_at_right()               
