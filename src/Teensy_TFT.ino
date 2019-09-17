
/* 
Teensy TFT with touch screen User interface using bottons and emulating a keypad and Keypad library style functions.
The Code is based on Adafruit 
Arduin-o-Phone. Phone elements are removed.  Libraries are the Teensy equivalent libraries.
The Sketch uses the Adafruit button helpers to create and determine which button pressed
The Sketch uses code from Teensy touch screen example  ILI9341Test
This Version by Jack Edwards with help from Kris Kasprzak

The exapmle uses Jack Edwards screen layout for a marine autopilot
Nov/15/2017
*/

#if TFT_Used
#if Board == Teensy // Teensy
#include <SPI.h>
#include <ILI9341_t3.h>
#include <font_Arial.h> // from ILI9341_t3
#include <XPT2046_Touchscreen.h>

#define TFT_DC 9
#define TFT_CS 10

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
#define CS_PIN  8
XPT2046_Touchscreen ts(CS_PIN);
#define tftXmax 240
#define tftYmax 320

/*  This is calibration data for the raw touch data to the screen coordinate
 *  Print touch screen raw data to the serial monitor then use a pencil point for pointer.  Use the white tft screen corners for 
 *  calibration points and enter them below. Print the Calibrated data and compare to tft screen of 0,0 and screen size max
 */
#define TS_MINX 168 // at tft 0,0
#define TS_MINY 290 // at tft 0,0
#define TS_MAXX 3758 //value from touch screen upper right
#define TS_MAXY 3971 //value from touch screen upper right

#define print_unscaled_data 0 // 1 to print raw touch screen data  or 0 to not print 
#define print_calibrated_data 0

/******************* UI details */
#define BUTTON_NUMBER 16
#define BUTTON_ROWS 4
#define BUTTON_COLS 4 // BUTTON COLUMNS
#define BUTTON_X 35
#define BUTTON_Y 165
#define BUTTON_W 40
#define BUTTON_H 30
#define BUTTON_SPACING_X 15
#define BUTTON_SPACING_Y 15
#define BUTTON_TEXTSIZE 2
#define BUTTON_LABELSIZE 3
#define BUTTON_DISPLAYSIZE 4
#define buttonHELDtime 800 // how long in millis before button pressed becomes buttton held 800 used for demo probably use 400

// text box where numbers go
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_WHITE

int buttonTIMER;
int buttonTIMERstart;
String buttonSTATUS;
String buttonSTATUSwas;
int TFTkey;

/* create 15 buttons, lables and colors */
char buttonlabels[BUTTON_NUMBER][BUTTON_LABELSIZE+1] = { "CMP", "GPS", "TAK", "A", "-10", "KNB", "+10","B", "-1", "SCN", "+1","C", "PRT", "OFF", "STB","D" };
uint16_t buttoncolors[BUTTON_NUMBER] =      
                            { ILI9341_BLUE, ILI9341_BLUE, ILI9341_BLUE, ILI9341_RED, 
                              ILI9341_BLUE,ILI9341_BLUE, ILI9341_BLUE, ILI9341_RED, 
                             ILI9341_BLUE, ILI9341_BLUE, ILI9341_BLUE, ILI9341_RED,                              
                             ILI9341_ORANGE, ILI9341_BLUE, ILI9341_ORANGE,ILI9341_RED};
                             //ILI9341_DARKGREEN, ILI9341_DARKGREY, ILI9341_RED, 
Adafruit_GFX_Button buttons[BUTTON_NUMBER]; // isPressed(), justPressed, justReleased(), 

 String textDisplay[BUTTON_NUMBER ] = { "COMP", "GPS", "TACK", "A", "-10", "KNOB", "+10", "B", "-1", "SCREEN", "+1", "C", "PORT", "OFF", "STBD", "D"};                            

void Init_Teensy_TFT() {
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  
  // eep touchscreen not found?
  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
    while (1);    
  }
  Serial.println("Touchscreen started");
 
  Create_Buttons();  
}  // end Init_Teensy

  void Create_Buttons(){
  for (uint8_t row=0; row<BUTTON_ROWS; row++) {
    for (uint8_t col=0; col<BUTTON_COLS; col++) {
      buttons[col + row*BUTTON_COLS].initButton(&tft, BUTTON_X+col*(BUTTON_W+BUTTON_SPACING_X), 
                 BUTTON_Y+row*(BUTTON_H+BUTTON_SPACING_Y),    // x, y, w, h, outline, fill, text
                 BUTTON_W, BUTTON_H, ILI9341_WHITE, buttoncolors[col+row*BUTTON_COLS], ILI9341_WHITE,
                 buttonlabels[col + row*BUTTON_COLS], BUTTON_TEXTSIZE); 
      buttons[col + row*BUTTON_COLS].drawButton();
    }
  }
   tft.drawRect(0,0,240,320, ILI9341_WHITE); // Draws white rectangle around screen use as touch screen calibration points
}  // end create buttons

void Print_Screen_0(){  //Print the static screen elements once
    tft.drawFastHLine(2,48,238,ILI9341_WHITE);
    tft.drawFastHLine(2,96,238,ILI9341_WHITE);
    tft.drawFastHLine(2,144,238,ILI9341_WHITE);
    tft.drawFastVLine(81,2,144,ILI9341_WHITE);
    tft.drawFastVLine(160,2,144,ILI9341_WHITE);
    
    tft.setTextColor(ILI9341_GREEN);
    tft.fillRect(2,4,76,24,ILI9341_BLACK); 
    tft.setCursor(4,4); tft.setTextSize(3);tft.print(heading,0);
    tft.setCursor(4,28); tft.setTextSize(2); tft.print(" Head");

    tft.fillRect(84,4,76,24,ILI9341_BLACK); 
    tft.setCursor(84,4); tft.setTextSize(3); tft.print(heading_to_steer,0);
    tft.setCursor(84,28); tft.setTextSize(2); tft.print(" HTS");

    tft.fillRect(164,4,75,24,ILI9341_BLACK); 
    tft.setCursor(164,4); tft.setTextSize(3); tft.print(course_to_steer,0);
    tft.setCursor(164,28); tft.setTextSize(2); tft.print(" CTS");

    tft.fillRect(2,52,76,24,ILI9341_BLACK); 
    tft.setCursor(4,52); tft.setTextSize(3); tft.print(Mode);
    tft.setCursor(4,76); tft.setTextSize(2); tft.print(" Mode");

    tft.fillRect(4,100,76,24,ILI9341_BLACK); 
    tft.setCursor(4,100); tft.setTextSize(3); tft.print(Mode);
    tft.setCursor(4,124); tft.setTextSize(2); tft.print(" Mode");

    
    
    // line++; tft.setCursor(0,line * line_space); tft.print("HDG: "); tft.print(heading);
    //line++; tft.setCursor(0,line * line_space); tft.print("HTS: ");
    //line++; tft.setCursor(0,line * line_space); tft.print("COG: ");
   // line++; tft.setCursor(0,line * line_space); tft.print("CTS: ");
   // line++; tft.setCursor(0,line * line_space); tft.print("XTE: ");
   // line++; tft.setCursor(0,line * line_space); tft.print("WND: ");
    //line++; tft.setCursor(0,line * line_space); tft.print("SPD: ");
    //line++; tft.setCursor(0,line * line_space); tft.print("CAL: ");

    if (Steering_Mode == 4)
    {
      //line++; tft.setCursor(0,line * line_space); tft.print("WTS: ");
    } 
    //line++; tft.setCursor(0,line * 24); tft.print("RNG: ");
    //line++; tft.setCursor(0,line * 24); tft.print("BRG: ");
    //line++; tft.setCursor(0,line * 24); tft.print("SOG: ");  
    //line++; tft.setCursor(0,line * 24); tft.print("TURN ");
   // line++; tft.setCursor(0,line * 24); tft.print("UTC: ");
    //line++; tft.setCursor(0,line * 24); tft.print("RUD: ");
    //tft.fillRect(0, 270,240,48, ILI9341_GREEN);tft.fillRect(05, 275,230,38, ILI9341_BLACK); // draws rudder graphic at bottom
}  

   


void Get_Touchscreen_Input() {
  //  Get touch screen input
  //Serial.println("touch Screen ");  
  TS_Point p;
   boolean istouched = ts.touched(); // this line from teensy IL9341Test
  //if (ts.bufferSize()) { // this line not working with <ILI9341_t3.h>
  if(istouched) { // this line works with <ILI9341_t3.h>
    //Serial.print("ts.BuferSize "); Serial.println(ts.bufferSize());
    p = ts.getPoint(); 
  }
  else {
    // this is our way of tracking touch 'release'!
    p.x = p.y = p.z = -1;
  }
  
  if (p.z != -1) { 
      
       if(print_unscaled_data) {  // use to get data to calibrate touch screen
          Serial.print("Raw Touch Screen ");
          Serial.print("("); Serial.print(p.x); Serial.print(", "); 
          Serial.print(p.y); Serial.print(", "); 
          Serial.print(p.z); Serial.println(") ");
       }
    /* SCALE TOUCH SCREEN TO TFT corordinates
     * Teensy touch screen in portrait view is 0,0 in upper right, x is down Y is left
     * tft screen in portrait mode is 0,0 in upper left x to the right and y down
     */
       
      uint16_t temp_PX = p.x;
    p.x  = map(p.y, TS_MAXY,TS_MINY,0,240);
    p.y  = map(temp_PX, TS_MINX,TS_MAXX,0,320);
        if(print_calibrated_data) {
          Serial.print("Calibrated data ");
          Serial.print("("); Serial.print(p.x); Serial.print(", "); 
          Serial.print(p.y); Serial.print(", "); 
          Serial.print(p.z); Serial.println(") ");  
        }
  }
  
  // go thru all the buttons, checking if they were pressed
  for (uint8_t b=0; b<BUTTON_NUMBER; b++) {
    if (buttons[b].contains(p.x, p.y)) {
      //Serial.print("Pressing: "); Serial.println(b);
      buttons[b].press(true);  // tell the button it is pressed
    } else {
      buttons[b].press(false);  // tell the button it is NOT pressed
    }
  }
  
   buttonSTATUSwas = buttonSTATUS; // used to print screen data only when data has changed
   
  // now we can ask the buttons if their state has changed
  for (uint8_t b=0; b<BUTTON_NUMBER; b++) {
    if (buttons[b].justReleased()) {
      //Serial.print("Button "); Serial.print(b); Serial.println(" Released"); 
      buttons[b].drawButton();  // draw normal
      // Print text window      
      
      /*tft.fillRect(TEXT_X+1, TEXT_Y+1, TEXT_W-2, TEXT_H-2, ILI9341_BLACK);
      tft.setCursor(TEXT_X+4, TEXT_Y+12);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(3);
      tft.print(textDisplay[b]); // this line will print String textDisplay {b}
      */
      buttonSTATUS = " released"; 
      KEY_RELEASED();             
    }
    
    if (buttons[b].justPressed()) {
        buttons[b].drawButton(true);  // draw invert!
        //Serial.print("Button "); Serial.print(b); Serial.println(" Pressed");
        buttonSTATUS = " pressed";
        buttonTIMERstart = millis(); 
        TFTkey = b;
        KEY_PRESSED();  // subroutine where user can put code for button action     
        /*
        tft.fillRect(TEXT_X+1, TEXT_Y+1, TEXT_W-2, TEXT_H-2, ILI9341_BLACK);
        tft.setCursor(TEXT_X+4, TEXT_Y+12);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(3);
        //for(int j= 0; j<BUTTON_LABELSIZE;j++) // these two lines will print button labels 
        //tft.print(buttonlabels[b][j]);
        tft.print(textDisplay[b]); // this line will print String textDisplay {b}
        tft.setTextSize(2);
        */        
      } // end if buttons just pressed
    
    if (buttons[b].isPressed()) {
        // Serial.print("Button "); Serial.print(b); Serial.println(" is being held");
         buttonTIMER = millis() - buttonTIMERstart;
      //Serial.print ("button Timer = "); Serial.println(buttonTIMER);
        if (buttonTIMER > buttonHELDtime ){ 
          buttonSTATUS =" held";
          KEY_HELD();              
        }
    }
       /*
        if (buttonSTATUS != buttonSTATUSwas){ // print to screen only when status changes
          tft.setCursor(120,25);
          tft.fillRect(120,15,100,30, ILI9341_BLACK);
          tft.setTextSize(2);
          tft.print(buttonSTATUS);
         // Serial.println(buttonSTATUS);
        }
        */
  }  //for loop 
  //delay(100); // UI debouncing 

}  // end Get Touchscreen Input

  /********************     KEY HANDLING    **************************/
  /*    Only the first two buttons are programed     */

  void KEY_PRESSED(){
    //Serial.print("KEY PRESSED "); Serial.print("  Key = "); Serial.println(key);
    switch(TFTkey){
      case 13: //OFF
       KeyPressed('0');
      break;
      
      case 0: //COMP
       KeyPressed('1');
      break;
      
      case 1: //GPS
       KeyPressed('2');
      break;

      case 2: //TACK and WIND Wind could be moved to separate key
       KeyPressed('3');
      break;

      case 4: // -10 deg
       KeyPressed('4');
      break;

       case 6: // +10 deg
       KeyPressed('6');
      break;

       case 8: // -1 deg
       KeyPressed('7');
      break;

       case 10: // +1 deg
       KeyPressed('9');
      break;

       case 12: // * dodge left
       KeyPressed('*');
      break;

       case 14: // # dodge right
       KeyPressed('#');
      break;

       case 5: // Knob Steering
       KeyPressed('5');
      break;

       case 9: //  Screen up
       KeyPressed('8');
      break;
      
    } // end switch
   }  // end TFTkey pressed
  
  void KEY_HELD(){
  switch (TFTkey){
      case 1:
         KeyHeld('2');
      break;
  } // end switch key

  }  // end TFTkey held

  void KEY_RELEASED(){
    switch(TFTkey){
        case 12: // * released 
       KeyReleased('*');
      break;

       case 14: //  # released
       KeyReleased('#');
      break;
    } // end switch  
  } //end TFTkey released
#endif // end if Board == Teensy 
#endif // end if TFT_Used

