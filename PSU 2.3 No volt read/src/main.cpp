

/********************************************************************************** 
 *  Combining the CD4013 code for debouncing rotary encoders, with TFT_espi to produce a power supply                                      
 *  13 March 2022 - v3 20 March 2022 12:04
 *                  v6 27 March 2022 
 *                  v7 30 March 2022
 *                  v8 17 April 2022 - good version use to revert
 * TFT Setup - this is the config in the user_setup.h file in the TFT_espi library
 *     #define TFT_MISO 19
 *     #define TFT_MOSI 23
 *     #define TFT_SCLK 18
 *     #define TFT_CS    5  // Chip select control pin
 *     #define TFT_DC    2  // Data Command control pin
 *     #define TFT_RST   4  // Reset
 *  TFT screen is working with buttons in portrait
 *  Built the touch screen capability - enable button, dual tracking, etc.done
 *  Need to fix the touchscreen issue - voltage set becomes unresponsive if screen is polled - fixed
 *  Need to fix the rotary encoder response - fixed. Note: This is best version as of 31 March 2022
 *  This version written in PlatformIO - added current and voltage sense, refactored variables
 *  Multicore solution implemented and working
 *  Also need to review scope - done
 *  Need to add 2nd rotary encoder for current setting - not done
 *  Refactor all button draw routines  - not done
 *  Add short-circuit protection (if Rl < 1 ohm) not done, I think
 *  Fixed code to eliminate precision adjustment - Not required
 *  Must fix ads1115
 *********************************************************************************/
// Definitions
//===============================================================================
  #include <iostream> 
  #include <TFT_eSPI.h> 
  #include <Wire.h>
  #include <SPI.h>
  #include <ADS1X15.h>
  #include "Free_Fonts.h" 
  #include <driver/adc.h>
  using namespace std;
  using namespace ADS1X15;

  #define DEBUG 1                   // Change this to 0 when debugging is over
  #define CLK 17                    // Use 17 on ESP32 for Rot_Enc CLK 1
  #define DATA 26                   //must make this 26 for Rot_Enc DATA 1
  #define CLK2 33                   // tba 33 clock 2
  #define DATA2 13                  // tba 13 data 2
  #define Reset1 16                 // Rot_Enc Reset 1
  #define Reset2 14                 // Rot_Enc Reset 2
  #define V2out 27                  // Positive channel Vout
  #define V1out 32                  // Negative channel Vout
  #define TFT_GOLD1  0xED86         // Colour Gold
  #define TFT_BROWN1 0x8320         // Brown
  #define TFT_CHARC 0x29C5          // Charcoal
  #define ENABLE_RLA 12

  #if DEBUG == 1
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
  #else
    #define debug(x) 
    #define debugln(x) 
  #endif  

  TaskHandle_t Task0;
  TaskHandle_t Task1;

  ADS1115<TwoWire> ads(Wire);  //Instantiate ADS1115
  
 // Adafruit_ADS1115 ads1115;	// Construct an ads1115 
//===============================================================================
// Variables
//===============================================================================

  int c = 0;                        // counter for the encoder
  int old_c = 0;                    // counter
  int c2 = 0 ;
  int8_t old_c2 = 0 ;
  int i = 0 ;                       // counter for current limit rotary encoder
  int Iout = 0;                     // This is the var for the current limit - displayed via Ilimit below
  float Vset = 0;                   // Setpoint display voltage for + channel
  float Vset2 = 0;                   // Setpoint display voltage for + channel
  float Ilimit = 0;                 // Display current limit
  float Vsetneg = 0;                // Setpoint display voltage for - channel
  float Ioutneg = 0;                // Display current for - channel
  float Vsense0 ;                    // For the final Voltage output display decimal
  float Vsense1 ;
  float Isense0 ;
  float Isense1 ; 
  int MAV2, MAV1 ;
  int V2Array[255] ;                  // Array for moving average voltage for positive voltage
  unsigned short usStackHighWaterMark;  // Just to check stack usage
   int16_t adc0, adc1, adc2, adc3;

//  gpio_num_t ADC_out_ref = (gpio_num_t) 25;

  uint16_t SetResolution = 200 ;                 // This is to set the resolution of the set voltage, 400 will give 50mV res.
  const unsigned long Delay1 = 25;              // This is for the delay waiting for the quadrature pulse 10 ms is optimal
  const unsigned long DelayRST = 20;              // This is the length of the reset pulse to the 4013
  const unsigned long Delay2 = 10 ;              // This is for the delay waiting for the quadrature pulse 10 ms is optimal
  const unsigned long Delay3 = 1; 
  unsigned long previousTime = 0;
  unsigned long currentTime = 0;
  uint16_t x = 0, y = 0, z = 0;                   // To store the touch coordinates
 // uint8_t dec_len = 2;                            // Number of decimals to display
  bool pressed, up;                               // Logic to determine state of screen pressed
  bool Precise, OldPrecise, SOAR, CL, OP, DT = false;    // Status
  bool ToggleCh1, ToggleCh2 ;

 //===============================================================================
// Interrupt routine just sets a flag when rotation is detected
//===============================================================================
 
volatile bool rotaryEncoder = false;

//===============================================================================
// Instantiate the display
//===============================================================================
 
  TFT_eSPI tft = TFT_eSPI();  // Invoke custom library  
  TFT_eSprite sprite= TFT_eSprite(&tft);

/************** Setting PWM Properties *******************/

  int dutyCycle;
  const int PWMFreq = 10000; 
  const int PWMChannel1 = 0;
  const int PWMResolution = 12;
  const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

 /*************** end of PWM properties ******************/

 
//===============================================================================
// Preprocessor declaration 
//===============================================================================

void loop0(void * parameter);
void loop1(void * parameter);
void Check_current();
void Draw_Button_Channel(int, int, String);
void Draw_Button_Enable();
void Draw_Button_DT();
void Draw_Button_Spare();
void Draw_Button_Ch1_pressed();
void Draw_Button_Ch2_pressed();
void Draw_Button_Enable_pressed();
void Draw_Button_DT_pressed();
void Draw_Button_Spare_pressed();
void Check_Rot_Enc() ;
void Display_Output() ;
void Check_button_press();
void Function_Select();
void Display_status();
void Read_volts();
void rotary() ;
void MyDelay(unsigned long);
void RefreshScreen();
/* End of preprocessor declaration */

void Check_Rot_Enc() 
{
    if (digitalRead(CLK) && !digitalRead(DATA) ) {    // pulse train out of phase, read first pulse to find direction
      up = true;
      debugln("up");
      delay(Delay1);                                     // wait for trailing quadrature pulse to arrive - play with this value
    }
       
  if (digitalRead(DATA) && !digitalRead(CLK))  {      // checks if pin 2 or pin 7 is leading, if 7 it's counting down
   up = false; 
   debugln("down");
   delay(Delay1);  
  }
                           
      if (digitalRead(CLK) && digitalRead(DATA)) {      // wait for trailing quadrature pulse to arrive - play with this value
          if( up && c < SetResolution ) { c++; } 
          if( !up && c > 0 ) { c--; }
      
      digitalWrite(Reset1, HIGH); // Reset the 4013 latch
      delay(DelayRST);
      digitalWrite(Reset1, LOW); 
    //  debugln(c);
      }
  if (c != old_c)
      {
        Vset =  c  * 20.0 / SetResolution ;
   
        Check_current();
  }
    old_c = c;
    rotaryEncoder = false;

//-----------------------------------------
// Read the second encoder for current set
//-----------------------------------------
 digitalWrite(Reset2, LOW);      // Make sure CD4013 is enabled

    if (digitalRead(CLK2) && !digitalRead(DATA2) ) {    // pulse train out of phase, read first pulse to find direction
      up = true;
      delay(Delay1);                                     // wait for trailing quadrature pulse to arrive - play with this value
    }
       
  if (digitalRead(DATA2) && !digitalRead(CLK2))  {      // checks if pin 2 or pin 7 is leading, if 7 it's counting down
   up = false; 
   delay(Delay1);  
   
     }                        
      if (digitalRead(CLK2) && digitalRead(DATA2)) {      // wait for trailing quadrature pulse to arrive - play with this value
          if( up && c2 < SetResolution ) {c2++; } 
          if( !up && c2 > 0 ) { c2--; }
      digitalWrite(Reset2, HIGH); // Reset the 4013 latch
      delay(DelayRST);
      digitalWrite(Reset2, LOW); 
 //     debugln(c);
      }
  if (c2 != old_c2)
      {
        Vset2 =  c2  * 20.0 / SetResolution ;
   
        Check_current();
  }
    old_c2 = c2;
    digitalWrite(Reset2, HIGH); // Reset the 4013 latch
    delay(DelayRST);
    digitalWrite(Reset2, LOW); 


 }
void Read_volts()    
  {
    /*  for (int Vcount = 0; Vcount < 127; Vcount++ ) 
         {   
           V2Array[Vcount] = ads1115.readADC_SingleEnded(0); ; 
           delay(2);  
          MAV2 = ( MAV2 + V2Array[Vcount] ) ; 
          }
        MAV2 = (MAV2 >> 3 ) ;
    
       Vsense2 = ( MAV2 * 20.0 )/ 569179  ;
       debugln(Vsense2);
        Serial.println(Vsense2);  */

      adc0 = ads.readADCSingleEnded(0);
      adc1 = ads.readADCSingleEnded(1);
      adc2 = ads.readADCSingleEnded(2);
      adc3 = ads.readADCSingleEnded(3);

      Vsense0 = ads.computeVolts(adc0);
      Vsense1 = ads.computeVolts(adc1);
      Isense0 = ads.computeVolts(adc2);
      Isense1 = ads.computeVolts(adc3);

  /*  Serial.print("V0= ");
    Serial.println(Vsense0);
    Serial.print("V1= ");
    Serial.println(Vsense1);
    Serial.print("I0= ");
    Serial.println(Isense0);
    Serial.print("I1= ");
    Serial.println(Isense1);
    delay(1000);*/

  }
void Draw_Button_Channel(int xoffset, int yoffset, String Chan )  // This replaces two calls to draw channel pressed and not
  {
    // Commmon button procedure
  tft.fillRoundRect( 65 + xoffset, 12, 92, 31, 2, TFT_CHARC);  
  tft.fillRect(63 + xoffset, 10, 90, 30, TFT_BROWN1);    //base for Channel 1
  tft.fillRect(63 + xoffset, 13, 90, 23, TFT_GOLD1);     // Button for Channel 1
  tft.fillRect(65 + xoffset, 15, 86, 3, TFT_WHITE);     // Accent for Channel 1
  tft.setFreeFont(FSS9);
  tft.setTextColor(TFT_BLACK, TFT_GOLD1);
  tft.drawString( Chan, 72 + xoffset, 18, 2);
  

  }
void Draw_Button_EnableMultiF(int xoffset, int yoffset, String ButtonFace)  // 
  {
   
  tft.fillRoundRect( xoffset + 2, yoffset + 2, 82, 31, 2, TFT_CHARC);  //x= 20, y = 200
  tft.fillRect(xoffset, yoffset , 80, 30, TFT_BROWN1);    //base for Enable
  tft.fillRect(xoffset , yoffset + 3, 80, 23, TFT_GOLD1);     // Button for Enable
  tft.fillRect(xoffset + 2, yoffset + 5, 76, 3, TFT_WHITE);     // Accent for Enable
  tft.setFreeFont(FSS9);
  tft.setTextColor(TFT_BLACK, TFT_GOLD1); 
  tft.drawString( ButtonFace, xoffset + 8, yoffset + 8, 2 );
  // OP = 0 ;
  }
void Draw_Button_Enable_pressed()
  {
    tft.fillRect( 20, 200, 82, 32, TFT_BLACK);  
    tft.fillRect(22, 203, 80, 30, TFT_BROWN1);    //base for Channel 1
    tft.fillRect(22, 206, 80, 23, TFT_GOLD1);     // Button for Channel 1
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_BLACK, TFT_GOLD1);
    tft.drawString("Enabled", 30, 210, 2);
     // OP = 1 ;
  }  
void Draw_Button_Ch1_pressed()
  {
    tft.fillRect( 63, 10, 92, 32, TFT_BLACK);  
    tft.fillRect(65, 13, 90, 30, TFT_BROWN1);    //base for Channel 1
    tft.fillRect(65, 16, 90, 23, TFT_GOLD1);     // Button for Channel 1
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_BLACK, TFT_GOLD1);
    tft.drawString("Channel 1", 80, 20, 2);
    // Toggle_CH1 = 1 ;
  }
void Draw_Button_Ch2_pressed()   
{
    tft.fillRect( 173, 10, 92, 32, TFT_BLACK);  
    tft.fillRect(175, 13, 90, 30, TFT_BROWN1);    //base for Channel 1
    tft.fillRect(175, 16, 90, 23, TFT_GOLD1);     // Button for Channel 1
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_BLACK, TFT_GOLD1);
    tft.drawString("Channel 2", 180, 20, 2);
    // Toggle_CH2 = 1 ;
  }
void Draw_Button_DT_pressed()
  {
    tft.fillRect( 120, 200, 82, 32, TFT_BLACK);  
    tft.fillRect( 122, 203, 80, 30, TFT_BROWN1);    //base for Channel 1
    tft.fillRect( 122, 206, 80, 23, TFT_GOLD1);     // Button for Channel 1
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_BLACK, TFT_GOLD1);
    tft.drawString("DT on", 130, 210, 2);
  }
void Draw_Button_Spare_pressed()
  {
   tft.fillRect( 195, 200, 82, 32, TFT_BLACK);  
    tft.fillRect(197, 203, 80, 30, TFT_BROWN1);    //base for Channel 1
    tft.fillRect(197, 206, 80, 23, TFT_GOLD1);     // Button for Channel 1
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_BLACK, TFT_GOLD1);
    tft.drawString("50mV", 210, 208, 2);
    // Precise = 1 ;
  }
void Check_current()  
{
  Ilimit  = 3 ;      // Sets the current to an arbitrary amount, 3 amp for now
 // debug(Ilimit);
  // debug(" A is the limit");
 // Isense2 = analogRead( Is2 );
//  Isense1 = analogRead( Is1 );
  // Vout = Vset ; // This is also only valid for testing. Normally analogue read Vout
  // Iout = Vout / 10 ; // This is just to test, assuming a load of 10 ohms
  // debugln( Iout );

  if (Iout < Ilimit) 
  {
    CL = 0 ;
  //  debugln(" not at the limit ");
  }
  else if ( Iout >= Ilimit )
  {
    debugln(" hit the limit");
    CL = 1 ;
    Vset = Ilimit * Vsense0 / Isense0 ; // calculated on the fly. Rl = Vsense2 / Isense2
    c = ( Vset * SetResolution / 20 );
  }
  // Vout = Vset ; // This is also only valid for testing. Normally analogue read Vout
}
void Display_Output() 
  
  {     
       // This is the output voltage
     
      tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( abs(Vsense0), 2, 175, 65, 4 );
  //    tft.drawString("V", 125, 175, 4);

      // This is the set voltage
     
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Vset, 2, 175, 95, 4);
   //   tft.drawString("V", 125, 45, 4);

      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Isense0, 2, 175, 125, 4);
 //     tft.drawString("A", 125, 75, 4);

      // This is the Current limiting
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Ilimit, 2, 175, 155, 4);
  //    tft.drawString("A", 125, 115, 4);

//      And now the negative voltage settings

  // This is the output voltage
      tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawString("-", 75, 65, 4);
      tft.drawFloat( abs(Vsense0), 2, 85, 65, 4);
    //  tft.drawString("V", 125, 15, 4);

      // This is the set voltage
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawString("-", 75, 95, 4);
      tft.drawFloat( Vset, 2, 85, 95, 4);
    //  tft.drawString("V", 125, 45, 4);

      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Iout, 2, 85, 125, 4);
    //  tft.drawString("A", 125, 245, 4);

      // This is the current limit
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Ilimit, 2, 85, 155, 4);
 //     tft.drawString("A", 125, 285, 4);
       
  }
void Check_button_press()
  {  
    x = y = 0 ;
    pressed = tft.getTouch(&x, &y, z);
   // delay(50);
    if (pressed) {
      Function_Select();
   //   delay(100);
    }
    return;
  }
void Function_Select()
 { 
  
  /*    if (( x > 70 ) && ( x < 150) && ( y > 150) && !DT  ) 
      { 
        debugln("Channel 1");
        debug(ToggleCh1);

            if ( !ToggleCh1 )
            {
            //  Add lots of code to hold Ch1 voltage
        //    Draw_Button_Ch1_pressed();
            ToggleCh1 = 1 ; 
            }
            else
            {
            Draw_Button_Channel( 0, 0, "Channel 1");
            ToggleCh1 = 0 ;
            }
        }
      else if (( y > 150 ) && ( x > 150 ) && !DT )
      {
        debugln("Channel 2");
        debug(ToggleCh2);

            if ( !ToggleCh2 )
               {
              //  Add lots of code to hold Ch1 voltage
         //         Draw_Button_Ch2_pressed();
                  ToggleCh2 = 1 ; 
               }
          else
              {
              Draw_Button_Channel( 110, 0, "Channel 2");
              ToggleCh2 = 0 ;
              }
      } */
      
      if (( y < 70) && ( x > 20 ) && ( x < 120 ))
      {
      debugln("Enable");
      delay(20);
     
      debug(OP);

          if ( !OP )
          {
           Draw_Button_Enable_pressed();
           OP = 1 ; 
           digitalWrite(ENABLE_RLA,1);
          }
          else
          {
             Draw_Button_EnableMultiF( 20, 200, "Enable O/P");
  
           OP = 0 ;
          }
      }
  /*  else if (( y < 70) && ( x > 120 ) && ( x < 200))
    {
      debugln("DT");
  
      delay(20);
     
      debug(DT);

          if ( !DT )
          {
           Draw_Button_DT_pressed();
           DT = 1 ; 
           debug("x: ");
           debug(x);
           debug("  y: ");
           debug(y);
          }
          else
          {
           Draw_Button_EnableMultiF( 120, 200, "Dual Track");
           DT = 0 ;
          }
    } */

    else if (( y < 70) && ( x > 220 ))
    {
      
      delay(20);
     
       if ( !Precise )
          {
           Draw_Button_Spare_pressed();
           Precise = 1 ; 
           debugln("Precise");
           debug("x: ");
           debug(x);
           debug("  y: ");
           debug(y);
          }
      else
          {
           Draw_Button_EnableMultiF( 195, 200, "100mV");
           Precise = 0 ;
           debugln("Precise off");
          }
      } 
  } 
void Display_status() 
{
        tft.setTextColor(TFT_RED, TFT_BLACK);

      if (OP) {
        tft.drawString("Output", 275, 50, 2);
      }
       if (!OP) {
        tft.fillRect( 275, 50, 40, 20, TFT_BLACK) ; 
       }
      if (DT){
        tft.drawString("Track", 275, 70, 2);
      }
        if (!DT){
         tft.fillRect( 275, 70, 40, 20, TFT_BLACK) ; 
      }
   
      if (SOAR) {
        tft.drawString("SOAR!", 275, 90, 2);
      }
      if (!SOAR) {
        tft.fillRect( 275, 110, 40, 20, TFT_BLACK) ; 
      }

      if (CL) {
          tft.drawString("Limit", 275, 110, 2);
              }
      if ( !CL ) { 
          tft.fillRect( 275, 90, 40, 20, TFT_BLACK) ;
          }
  }
void IRAM_ATTR rotary()
{
    rotaryEncoder = true;
} 
void MyDelay(unsigned long Delaytime)
{

  if(millis()-previousTime <= Delaytime)
  {
    // do something
    debugln("Waiting  ");
    debug("Previous:   ");
    debug(previousTime);
    debug("millis:   ");
    debugln(millis());
    previousTime = millis();
  }
    
}
void RefreshScreen()
{
/**********************************************************************************/
// Draw the static elements in the display
/*********************************************************************************/

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);   // Depending on the mode
  tft.setCursor(0, 0, 4);
  tft.drawRect(10, 50, 260, 140, TFT_YELLOW);
  tft.drawRect(60, 50, 110, 140, TFT_YELLOW);

  // Channel headers
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.drawString("Channel 1", 55, 5, 4 );  // Channel label negative

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.drawString("Channel 2", 175, 5, 4 );  // Channel label positive


 
/*  Draw_Button_Channel( 0, 0, "Channel 1");
  Draw_Button_Channel( 110, 0, "Channel 2"); */
  Draw_Button_EnableMultiF( 20, 200, "Enable O/P");
 /* Draw_Button_EnableMultiF( 120, 200, "Dual Track");   */
  Draw_Button_EnableMultiF( 220, 200, "100mV");
 
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 80, 4);
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.print("V");
  tft.setCursor(30, 80, 2);
  tft.setFreeFont(FSS9);
  tft.println("out");
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 110, 4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.println("V");
  tft.setFreeFont(FSS9);
  tft.setCursor(30, 115, 2);
  tft.println("set");
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 140, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("I");
  tft.setFreeFont(FSS9);
  tft.setCursor(20, 145, 2);
  tft.println("out");
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 170, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println("I"); 
  tft.setFreeFont(FSS9);
  tft.setCursor(20, 175, 2);
  tft.println("limit"); 

  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.drawString("V", 150, 65, 4);
  tft.drawString("V", 245, 65, 4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("V", 150, 95, 4);
  tft.drawString("V", 245, 95, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("A", 150, 125, 4);
  tft.drawString("A", 245, 125, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("A", 150, 155, 4);
  tft.drawString("A", 245, 155, 4);

}
void setup() 
  {

 /*********** Pin Mode Setup **************/

    pinMode( CLK, INPUT );
    pinMode( DATA, INPUT );
    pinMode( CLK2, INPUT );
    pinMode( DATA2, INPUT );
    pinMode( Reset1, OUTPUT ) ;
    pinMode( Reset2, OUTPUT );
    pinMode( V2out, OUTPUT );
    pinMode( V1out, OUTPUT );
    pinMode( ENABLE_RLA, OUTPUT);    // This is to drive the relay to apply the output
 //   pinMode( ADC_out_ref, OUTPUT );

/************* Rest of Setup ***************/

    Serial.begin (115200);
    digitalWrite(Reset1, HIGH);
    delay(2);
    digitalWrite(Reset1, LOW);
    
/**********************************************************************************/
// Set up TFT Display
/**********************************************************************************/

  tft.init();
  tft.setTextFont(GLCD); 
  
  /**********************************************************************************/
  // Set pwm resolution  to mode 7 (12 bit)
  /**********************************************************************************/
    
   ledcSetup(PWMChannel1, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channel to the GPIO Pin */
    ledcAttachPin(V2out, PWMChannel1);

  /*********************************************************************************/
  /*    Interupt setup                                                             */
  /********************************************************************************/

  // We need to monitor both pins, rising and falling for all states
    attachInterrupt(digitalPinToInterrupt(CLK), rotary, RISING);
    attachInterrupt(digitalPinToInterrupt(DATA), rotary, RISING);

  // ===============================================================================
  // Settings for ADC  
  //================================================================================  
 
//    ads1115.begin(0x48);  // Initialize ads1115 at the default address 0x48
//    ads1115.setGain(GAIN_ONE);  // 0.125mV steps, 4096mV max

    ads.begin();
    ads.setGain(Gain::ONE_4096MV);
 // ads.setDataRate(Rate::ADS1015_250SPS);  Used to work so can be fallback position
    ads.setDataRate(Rate::ADS1115_250SPS);
   // ads.setDataRate(Rate::ADS1115_8SPS); // Slowest rate for highest noise immunity
    
  //===============================================================================
  // Multicore set up //
  //===============================================================================
  xTaskCreatePinnedToCore(
      loop0,          // Function to loop through task0
      "Task0",        // Name of task
      5000,           // Size of stack
      NULL,           // Parameter
      0,              // Priority
      &Task0,         // Task handle
      0  ) ;            // Core where task should run
              
  xTaskCreatePinnedToCore (
      loop1,          // Function to loop through task0
      "Task1",        // Name of task
      5000,           // Size of stack
      NULL,           // Parameter
      0,              // Priority
      &Task1,         // Task handle
      1  ) ;            // Core where task should run
//===============================================================================
 /* Draw the static screen elements */
 //=================================

  RefreshScreen();
  DT = false ;
    debugln("Setup complete...correct program");
    Serial.println("Setup complete...correct program2");
    delay(2000);
  // End of setup
    
 //===============================================================================
}
void loop0( void * parameter )
{
  for (;;)
  {
    Read_volts();
 //   Serial.println("Core 0:");
     debug("Core 0:  ");
    // debugln( uxTaskGetStackHighWaterMark(NULL) );
    Serial.print("V0= ");
    Serial.println(Vsense0);
    Serial.print("V1= ");
    Serial.println(Vsense1);
    Serial.print("I0= ");
    Serial.println(Isense0);
    Serial.print("I1= ");
    Serial.println(Isense1);
    delay(1000);
  }
  
}
void loop1( void * parameter )  
{
  int j = 0 ;
  for(;;)
  {
    // The next block is new and might fail
   if ( Precise ) 
      { 
        SetResolution  = 400 ; 
        if ( Precise != OldPrecise )
        {
          c = c * 2 ;
        }
      }
    else 
      {
      SetResolution  = 200 ;
      if ( Precise != OldPrecise )
       {
        c = c / 2 ;
       }
     }
    OldPrecise = Precise ;
// end of block */
  
        if (  j == 10 ) 
            { 
                Check_button_press();  
                j = 0; 
            }
        j++;   
     
    //  Check_Rot_Enc();
      Display_Output();
      Display_status();
      ledcWrite(PWMChannel1, c * MAX_DUTY_CYCLE / SetResolution);
  if (rotaryEncoder)
    {
      debugln("Rot_Enc 1 detected") ;   //if it works, use to call Check_Rot_Enc
      Check_Rot_Enc();
    } 
  }
   
  
}
void loop() 
{
 delay(1);
// debugln(Vsense0);
}
