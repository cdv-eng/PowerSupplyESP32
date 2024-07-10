

/********************************************************************************** 
 *  Combining the CD4013 code for debouncing rotary encoders, with TFT_espi to produce a power supply                                      
 *  13 March 2022 - v3 20 March 2022 12:04
 *                  v6 27 March 2022 
 *                  v7 30 March 2022
 *                  v8 17 April 2022 - good version use to revert
 *                  v9 25 January 2023 - Perfect. Finished
 *                  v10 28 May 2024 - Removed Vset completely, removed Enable, changed precision
 *                  v11 10 June 2024 - Better rotary encoder algorithm
 *                  v12 16 June 2024 - Improved display - resolution of 50mV/100mV
 *                  v13 19 June 2024 - Added current limmiting to rotary encoder loop
 *                  v14 29 June 2024 - Replaced delays with millis,
 *                  v15 10 July 2024 - all bugs fixed, decimal display sorted.
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
 *  Need to add 2nd rotary encoder for current setting -  done
 *  Refactor all button draw routines  -  done
 *  Add short-circuit protection (if Rl < 1 ohm) done
 *  ads1115 working
 *  
 
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
  #include <Adafruit_INA219.h>
  using namespace std;
  using namespace ADS1X15;

  #define DEBUG 1                   // Change this to 0 when debugging is over
  #define CLK 17                    // Use 17 on ESP32 for Rot_Enc CLK 1
  #define DATA 26                   //mESP32 pin 26 for Rot_Enc DATA 1
  #define CLK2 33                   // tba 33 clock 2
  #define DATA2 13                  // tba 13 data 2
  #define Reset1 16                 // Rot_Enc Reset 1
  #define Reset2 14                 // Rot_Enc Reset 2
  #define V2out 27                  // Positive channel Vout
  #define TFT_GOLD1  0xED86         // Colour Gold
  #define TFT_BROWN1 0x8320         // Brown
  #define TFT_CHARC 0x29C5          // Charcoal
  #define ENABLE_RLA 32             // O/P pin for output relay (optional)
  #define Precision 25                // Since touch has stopped working, this is to toggle between hi-res and normal

  #if DEBUG == 1
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
  #else
    #define debug(x) 
    #define debugln(x) 
  #endif  

  TaskHandle_t Task0;
  TaskHandle_t Task1;

  Adafruit_INA219 ina219_A ;     // Instatiate positive side INA219
//  Adafruit_INA219 ina219_B(0x41) ;     // Instatiate negative side INA219

  ADS1115<TwoWire> ads(Wire);  //Instantiate ADS1115

  
//===============================================================================
// Variables
//===============================================================================

  int c = 0;                        // counter for the encoder
  int old_c = 0;                    // counter
  int c2 = 0 ;
  int8_t old_c2 = 0 ;
  int i = 0 ;                       // counter for current limit rotary encoder
  float Iout = 0;                   // This is the var for the current limit - displayed via Ilimit below
  float I2out = 0;                  // For the negative side current 
  float Vset = 0;                   // Setpoint display voltage for + channel
  float Vset2 = 0;                  // Setpoint display voltage for + channel
  float Ilimit = 1.0;                 // Current limit, default is 1A but adjusted at runtime
  float Vsense0 ;                   // For the final Voltage output display decimal
  float Vsense1 ;
  float Isense0 ;
  float Isense1 ; 
  
  unsigned short usStackHighWaterMark;  // Just to check stack usage
   int16_t adc0, adc1, adc2, adc3 ;

  uint16_t SetResolution = 200 ;                 // This is to set the resolution of the set voltage, 400 will give 50mV res.
  const unsigned long Delay1 = 2;                // This is for the delay waiting for the quadrature pulse 10 ms is optimal
  const unsigned long DelayRST = 5;              // This is the length of the reset pulse to the 4013
  const unsigned long Delay2 = 10 ;              // This is for the delay waiting for the quadrature pulse 10 ms is optimal
  const unsigned long Delay3 = 1; 
  unsigned long previousTime = 0;
  unsigned long currentTime = 0;
 // uint16_t x = 0, y = 0, z = 0;                   // To store the touch coordinates
 // uint8_t dec_len = 2;                          // Number of decimals to display
 // bool pressed, Vup, Iup ;                        // Logic to determine state of screen pressed
  bool Precise = false, OldPrecise, SOAR, CL, SC = false;    // Status

 //===============================================================================
// Interrupt routine just sets a flag when rotation is detected
//===============================================================================
 
volatile bool rotaryEncoder1 = false;
volatile bool rotaryEncoder2 = false;
volatile bool rotaryEncoder3 = false;
volatile bool rotaryEncoder4 = false;

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
void Draw_Button_Enable();
void Draw_Button_DT();
void Draw_Button_Spare();
void Draw_Button_Enable_pressed();
void Draw_Button_DT_pressed();
void Draw_Button_Spare_pressed();
void Check_Rot_Enc1() ;
void Check_Rot_Enc2() ;
void Display_Output() ;
void Check_button_press();
void Function_Select();
void Display_status();
void Read_volts();
void RefreshScreen();
/* End of preprocessor declaration */

void Check_Rot_Enc1()
{             
   if (digitalRead(CLK) && digitalRead(DATA)) 
      
      {      
          if( c > 0 ) { c--; }                  
          else if (c <= 0) { Vset = 0; }
        digitalWrite(Reset1, HIGH); // Reset the 4013 latch
     
            if(millis()-previousTime > DelayRST)
                  {
                    digitalWrite(Reset1, LOW); 
                    previousTime = millis();
                  }
         
     
        debug("c down= ");
        debugln(c);

        if (c != old_c )
            {
              Vset =  c  * 20.0 / SetResolution ;
              Check_current();
            }
      old_c = c;
     rotaryEncoder1 = false;
      }
 }
void Check_Rot_Enc2() 
{                              
  if (digitalRead(CLK) && digitalRead(DATA)) 
  
      {      
        if( c < SetResolution && !CL) { c++; } 

          digitalWrite(Reset1, HIGH); // Reset the 4013 latch
     //     delay(DelayRST);
           if(millis()-previousTime > DelayRST)
              {
                digitalWrite(Reset1, LOW); 
                previousTime = millis();
              }
         
          debug("c up= ");
          debugln(c);

      if (c != old_c)
          {
            Vset =  c  * 20.0 / SetResolution ;
            Check_current();
          }
        old_c = c;
        rotaryEncoder2 = false;
    }
}
void Check_Rot_Enc4() 
{        
    debugln("encoder 3 reached");                   
  if (digitalRead(CLK2) && digitalRead(DATA2)) 
      
      {    
        debugln("c2 up clk and data asserted");  
         if( c2 < 30 ) { c2++; }                      // 100 milliamp steps up to 3 Amps
          digitalWrite(Reset2, HIGH);                 // Reset the 4013 latch
     //     delay(DelayRST);
           if(millis()-previousTime > DelayRST)
              {
                digitalWrite(Reset2, LOW); 
                previousTime = millis();
              }
         
          digitalWrite(Reset2, LOW); 
          debug("c2 up= ");
          debugln(c2);
      
     if (c2 != old_c2)
      {
        Ilimit = abs( c2 / 10.0 );
   
        Check_current();
  }
    old_c2 = c2;
    rotaryEncoder4 = false;
      }
//    rotaryEncoder3 = true;
//    delay(Delay1);           // wait for trailing quadrature pulse to arrive - play with this value
}
void Check_Rot_Enc3()
{        
  debugln("encoder 3 reached");
 
      if (digitalRead(CLK2) && digitalRead(DATA2)) // wait for trailing quadrature pulse to arrive - both asserted, reset latch
      
        
      {     
        debugln("c2 down clk and data asserted"); 
           if (c2 > 0) {c2--;}
       //    else if ( c2 <= 0 ) { c2 = 0; }
  
          digitalWrite(Reset2, HIGH);             // Reset the 4013 latch
       //   delay(DelayRST);
           if(millis()-previousTime > DelayRST)
              {
                digitalWrite(Reset2, LOW); 
                previousTime = millis();
              }
         
          digitalWrite(Reset2, LOW); 
      
        if (c2 != old_c2)
          {
              Ilimit = c2 / 10.0 ;
              debug("c2 down= ");     
              debugln(c2);
              Check_current();
          }
    old_c2 = c2;
     rotaryEncoder3 = false;
      }
    //  delay(Delay1);
 }
void Read_volts()    
  {
      adc0 = ads.readADCSingleEnded(0);
      adc1 = ads.readADCSingleEnded(1);
      adc2 = ads.readADCSingleEnded(2);
      adc3 = ads.readADCSingleEnded(3);

      Vsense0 = ads.computeVolts(adc0);             // This provides an accurate reading from 0-4.096V  3.3V represents 20V output
      Vsense1 = ads.computeVolts(adc1);
      Isense0 = ads.computeVolts(adc2);             // For current, 1A = 185mV. So a reading of 2.5+.185=2.685 = 1A
      Isense1 = ads.computeVolts(adc3);

// for the display:
 //   Iout = ( Isense0 - 2.5 ) / 0.185 ;   //   same for negative amps
 //   I2out = (Isense1 - -2.5 ) / 0.185 ;
 
  }
void Draw_Button_EnableMultiF(int xoffset, int yoffset, String ButtonFace)  
  { 
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK); 
    tft.drawString( ButtonFace, xoffset + 8, yoffset + 8, 2 );
  // OP = 0 ;
  }
void Check_current()  
{
  
//  Iout = ( (Isense0 - 2.5 ) / 0.185 );   // must put back for ACS712
//  I2out = ( (Isense1 - 2.5 ) / 0.185 );

// Isense0 = ina219_A.getCurrent_mA();
  if ((( 22 - (Vsense0 / 3.3 * 20 ) * 1.1429))  * Iout > 50 ){ SOAR = true ; }
  if (Iout || I2out > 3) 
      { 
        c = 0; 
        SC = true;  // Short circuit protection
      }   

  else if (Iout || I2out > Ilimit)    // Need to uncomment this again
    // if (Iout > Ilimit) 
      {
        CL = 1 ;
        debugln(" hit the limit");
      }
  else if ( Iout < Ilimit )
      {
        CL = 0 ; 
      } 
}
void Display_Output() 
  
  {    
       // This is the output voltage
        
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setFreeFont(FSS12);
   //   tft.drawFloat( abs( ( Vsense0 / 3.3 * 20 ) * 1.1429), 2, 175, 65, 4);  // this is working but point moves around
      double V1 = abs( ( Vsense0 / 3.3 * 20 ) * 1.1429) ;         
    
    if (V1 >= 10 )
    {
      tft.drawFloat( V1, 2, 170, 65, 4); 
    }
    else
    {      
        tft.drawFloat(V1, 2, 170, 65, 4); 
        tft.drawString("    ", 220, 65, 4 );
    }
     
        // Current output
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat (abs(Iout), 2, 170, 100, 4);
         
 
      // This is the Current limiting
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Ilimit, 2, 170, 130, 4); 

//      And now the negative voltage settings

      // This is the output voltage
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setFreeFont(FSS12);
      double V2 = abs( ( Vsense1 / 3.3 * 20 ) * 1.1834) ; 
      tft.drawString("-", 65, 65, 4);
   //   tft.drawFloat( abs((Vsense1 / 3.3 * 20) * 1.1834), 2, 75, 65, 4);
      if (V2 >= 10 )
        {
          tft.drawFloat( V2, 2, 75, 65, 4);
        }
      else
        {
          tft.drawFloat( V2, 2, 75, 65, 4);
           tft.drawString("   ", 123, 65, 4 );
        }

    // This is the current output
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( abs(I2out ) , 2, 75, 100, 4);
     
      // This is the current limit
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setFreeFont(FSS12);
      tft.drawFloat( Ilimit, 2, 75, 130, 4); 


      // This is to display the power (W) drawn

       tft.drawFloat((V1* Iout ),2, 65, 200, 4);
  }
void Check_button_press()
  {  
 
  if (digitalRead(Precision)) 
  {
    if ( !Precise )
        { 
          SetResolution  = 400 ; 
          c = c * 2 ;
         debugln("High Precision 50mV");
         tft.drawRect(175, 198, 48, 20, TFT_WHITE);
         tft.drawRect(223, 198, 48, 20, TFT_BLACK);
         Precise = true ;
        }
    else 
        {
          SetResolution  = 200 ;
          c = c / 2 ;
          debugln("Low Precision 100mV");       
           tft.drawRect(223, 198, 48, 20, TFT_WHITE);
           tft.drawRect(175, 198, 48, 20, TFT_BLACK);
           Precise = false ;
        }
   }
  }
void Display_status() 
{
        tft.setTextColor(TFT_BLACK, TFT_RED);
   
      if (SOAR) {
        tft.drawString("SOAR!", 275, 90, 2);
      }
      if (!SOAR) {
        tft.fillRect( 275, 90, 40, 20, TFT_BLACK) ; 
      }

      if (CL) {
          tft.drawString("Limit", 275, 110, 2);
              }
      if ( !CL ) { 
          tft.fillRect( 275, 110, 40, 20, TFT_BLACK) ;
          }

      if (SC)
          {
             tft.drawString("Short!", 275, 110, 2);
          }
       if (!SC)
        {
            tft.fillRect( 275, 110, 40, 20, TFT_BLACK) ;
        }
     
  }
void IRAM_ATTR rotary1()
{
    rotaryEncoder1 = true;
    rotaryEncoder2 = false;
} 
void IRAM_ATTR rotary2()
{
    rotaryEncoder2 = true;
    rotaryEncoder1 = false;
} 
void IRAM_ATTR rotary3()
{
    rotaryEncoder3= true;
    rotaryEncoder4 = false;
} 
void IRAM_ATTR rotary4()
{
    rotaryEncoder4 = true;
    rotaryEncoder3 = false;
} 
void RefreshScreen()
{
/**********************************************************************************/
// Draw the static elements in the display
/*********************************************************************************/

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);   // Depending on the mode
  tft.setCursor(0, 0, 4);
  tft.drawRect(10, 50, 260, 120, TFT_YELLOW);
  tft.drawRect(55, 50, 110, 120, TFT_YELLOW);

  // Channel headers
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.drawString("Negative", 55, 15, 4 );  // Channel label negative

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.drawString("Positive", 170, 15, 4 );  // Channel label positive
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 80, 4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.println("V");
  tft.setFreeFont(FSS9);
  tft.setCursor(30, 80, 2);
  tft.println("out");
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 110, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("I");
  tft.setFreeFont(FSS9);
  tft.setCursor(20, 110, 2);
  tft.println("out");
  
  tft.setFreeFont(FSS12);
  tft.setCursor(15, 145, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println("I"); 
  tft.setFreeFont(FSS9);
  tft.setCursor(20, 145, 2);
  tft.println("limit"); 

  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("V", 140, 65, 4);
  tft.drawString("V", 240, 65, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("A", 140, 100, 4);
  tft.drawString("A", 240, 100, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("A", 140, 130, 4);
  tft.drawString("A", 240, 130, 4);

  /*---------------------------------------------------------------------------------
  * Draw initial resolution for rotary encoder
  * 
  *-----------------------------------------------------------------------------------*/
  
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Resolution", 190, 180, 2);
    tft.drawString("50mV", 180, 200, 2);
    tft.drawString("100mV", 225, 200, 2);
    tft.drawRect(223, 198, 48, 20, TFT_WHITE);

/* Draw power header*/
      tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
      tft.drawString("P= ", 25, 200, 4);

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
    pinMode( ENABLE_RLA, OUTPUT);    // This is to drive the relay to apply the output
    pinMode( Precision, INPUT );

/************* Rest of Setup ***************
*   Initialise Rotary Encoder hardware     */

    Serial.begin (115200);
    digitalWrite(Reset1, HIGH);
    delay(2);
    digitalWrite(Reset1, LOW);
    digitalWrite(Reset2, HIGH);
    delay(2);
    digitalWrite(Reset2, LOW);
    
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
  /*    Interrupt setup                                                             */
  /********************************************************************************/

  // We need to monitor both pins, rising and falling for all states
    attachInterrupt(digitalPinToInterrupt(CLK), rotary1, HIGH);
    attachInterrupt(digitalPinToInterrupt(DATA), rotary2, HIGH);
    attachInterrupt(digitalPinToInterrupt(CLK2), rotary3, HIGH);
    attachInterrupt(digitalPinToInterrupt(DATA2), rotary4, HIGH);

  // ===============================================================================
  // Settings for ADC  
  //================================================================================  
 
//    ads1115.begin(0x48);  // Initialize ads1115 at the default address 0x48
//    ads1115.setGain(GAIN_ONE);  // 0.125mV steps, 4096mV max

    ads.begin();
    ads.setGain(Gain::ONE_4096MV);
 // ads.setDataRate(Rate::ADS1015_250SPS);  Used to work so can be fallback position
 // ads.setDataRate(Rate::ADS1115_250SPS);
    ads.setDataRate(Rate::ADS1115_16SPS); // Slowest rate for highest noise immunity
   // Can do 16, 32, 64, 128 as well

 //   ads1.begin(0x49);
 //   ads1.setGain(Gain::ONE_4096MV);
 //   ads.setDataRate(Rate::ADS1015_250SPS);  Used to work so can be fallback position
 //   ads.setDataRate(Rate::ADS1115_250SPS);
  //  ads1.setDataRate(Rate::ADS1115_16SPS); // Slowest rate for highest noise immunity
   // Can do 16, 32, 64, 128 as well

   /*******************************************************************************
    *   Set up INA219 current sensors                                           
    *****************************************************************************/ 
// Default address for ina 219 is 0x40. Leave one device on this address and initialise the other on 0x41 (A0 on chip bridged)
   ina219_A.begin();
//   ina219_B.begin();


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
    debugln("Setup complete...correct program");
    Serial.println("Setup complete...correct program");
    delay(10);
  // End of setup
    
 //===============================================================================
}
void loop0( void * parameter )
{
  for (;;)
  {
    Read_volts();
    Check_current();
  //   debug("Core 0:  ");
  // debugln( uxTaskGetStackHighWaterMark(NULL) );
  
  }
  
}
void loop1( void * parameter )  
{
  int j = 0 ;
  for(;;)
  {
        if (  j == 10 ) 
            { 
                Check_button_press();  
                j = 0; 
            }
        j++;   
     
      Display_Output();
      Display_status();
      ledcWrite(PWMChannel1, c * MAX_DUTY_CYCLE / SetResolution);
  //========================Rotary Encoder to set Voltage============================    
  if (rotaryEncoder1)
    {
      debugln("Rot_Enc 1 up detected") ;   //if it works, use to call Check_Rot_Enc
      Check_Rot_Enc1();
  //    Vup = false;
      rotaryEncoder1 = false;
    } 
  if (rotaryEncoder2)
    {
      debugln("Rot_Enc 2 down detected") ;   //if it works, use to call Check_Rot_Enc
      Check_Rot_Enc2();
  //    Vup = true;
      rotaryEncoder2 = false;
    } 
//=======================Rotary Encoder to set Current limit==========================
    if (rotaryEncoder3)
    {
      debugln("Rot_Enc 3 up detected") ;   //if it works, use to call Check_Rot_Enc
      Check_Rot_Enc3();
    //  Iup = false;
      rotaryEncoder3 = false;
    } 
    if (rotaryEncoder4)
    {
      debugln("Rot_Enc 4 down detected") ;   //if it works, use to call Check_Rot_Enc
      Check_Rot_Enc4();
    //  Iup = true;
      rotaryEncoder4 = false;
    } 
  }
     
}
void loop() 
{
 delay(1);
// debugln(Vsense0);
}
