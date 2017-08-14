/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //       ==== ABOVE SCIENCE 1994 ====
   //
   //   Ab0VE TECH - HONDA Prelude Gen4 VFD Gauges controller
 */

#include <SPI.h>
#include <avr/pgmspace.h>
#include <U8glib.h> //platformio lib install "U8glib"
#include <Adafruit_ADS1015.h> // platformio lib install "Adafruit ADS1X15"
#include <Adafruit_MLX90614.h> // platformio lib install "Adafruit MLX90614 Library"
#include <max6675.h> // platformio lib install "MAX6675"

#include "img.h" // Mode logos

/******** TODO **********
   OLED                  ✓
   dimm                  ✓
   switch                ✓
   VFD needile           ✓
   VFD bar               ✓
   ANALOG TO ADS1115!!   ❏
   Voltmeter             ❏
   Oil pressure          ❏
   Oil temperature       ❏
   AirFuelRatio sensor   ❏
   EGT                   ❏
   Brakes temperature    ❏
 *************************/

int thermoDO  = 4;
int thermoCS  = 5;
int thermoCLK = 6;

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE); // I2C / TWI
Adafruit_ADS1115 ads;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX6675 thermocouple;

#define VISUAL_DELAY  0 // Refresh delay

//
// SENSORS
//

/* OIL TRESSURE / OIL TEMPERATURE */
#define OIL_TEMP_SENSOR     A0
#define R3 1000.0 // resistance of R3 (1K) in voltage devider
float OIL_TEMP = 0;

#define OIL_PRESSURE_SENSOR A1
#define R4 1000.0 // resistance of R4 (1K) in voltage devider
float OIL_PRESSURE = 0;


/* VOLTMETER    /        - */
#define VOLTMETER_SENSOR    A2
#define R1 100000.0 // resistance of R1 (100K) in voltage divider
#define R2 10000.0  // resistance of R2 (10K) in voltage divider
float VOLTAGE = 0;

/*    -         / BRAKES TEMPERATURE
   MLX90614
      SDL-----------A4
      SDA-----------A5
 */
float BRAKES_TEMP = 0;

/* O2 LAMBDA   / EXHAUST TEMPERATURE

   ADS1X15/max6675    arduino
     SDL-------------A4
     SDA-------------A5
 */
#define AFR_INPUT 0
float AFR = 0;
float EGT = 0;

/*
     VFD Contoller
     μPD6232C    arduino
     SI-------------8 BLue
     SCK------------7 red
     LH-------------6 green
 */
int SI_PIN  = 8; // Blue
int SCK_PIN = 7; // Green
int LH_PIN  = 6; // Gray

int DIMMER_PIN = 3; // Dimer input - High +12V!
int DIMMER_STATE = 0;
int DIMMER_PREVSTATE = 0;

int BUTTON_PIN = 12; // BUTTON to Ground
int BUTTON_STATE = 0;
int BUTTON_PREVSTATE = 0;

/*
     OLED DISPLAY  arduino
     SDL-------------A4
     SDA-------------A5
 */

#define NONE   0
#define BAR    1
#define NEEDLE 2

boolean DRAW_R = true;
boolean DRAW_RL = true;  // C - H
boolean DRAW_L = true;
byte TYPE_R = BAR;
byte TYPE_L = BAR;

byte POSITION_L = 0;
byte TARGETPOS_L = 0;
byte POSITION_R = 0;
byte TARGETPOS_R = 0;

#define STATE_OIL     1
#define STATE_EXHAUST 2
#define STATE_BRAKES  3
#define STATE_VOLT    4

boolean SHOW_LOGO = true;
int LOGO_STATUS = STATE_OIL;
#define MAX_LOGO      4

unsigned long time=0;
unsigned long timeP=0;
#define LOG_DELAY 1000
unsigned long timeL=0;
#define LOGO_DELAY 3000

void setup() {
        pinMode (SI_PIN, OUTPUT);
        pinMode (SCK_PIN, OUTPUT);
        pinMode (LH_PIN, OUTPUT);
        pinMode (BUTTON_PIN, INPUT);
        pinMode (DIMMER_PIN, INPUT);

// OLED INIT
        u8g.setContrast(0xff);
        u8g.firstPage();
        u8g.setRot180(); // Rorate screen with wires goes down
        do {
              u8g.drawXBMP( 0, 0, 128, 64, Prelude_LOGO);
        } while( u8g.nextPage() );

        //                                                     ADS1015 ADS1115
        // ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V 1 bit = 3mV     0.1875mV
        // ads.setGain(GAIN_ONE);        // +/- 4.096V 1 bit = 2mV     0.125mV
        // ads.setGain(GAIN_TWO);        // +/- 2.048V 1 bit = 1mV     0.0625mV
        // ads.setGain(GAIN_FOUR);       // +/- 1.024V 1 bit = 0.5mV   0.03125mV
        // ads.setGain(GAIN_EIGHT);      // +/- 0.512V 1 bit = 0.25mV  0.015625mV
        // ads.setGain(GAIN_SIXTEEN);    // +/- 0.256V 1 bit = 0.125mV 0.0078125mV
        ads.begin();
        ads.setGain(GAIN_TWO); // ADS1115: +2.048V/0.0625mV

        thermocouple.begin(thermoCLK, thermoCS, thermoDO);

        mlx.begin();

// init VFD
        digitalWrite (SI_PIN, LOW);
        digitalWrite (SCK_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);

        delay(1500);

        Serial.begin(115200);
        Serial.println("Ab0VE-TECH Honda Prelude Gauges contoller");
        Serial.println("TIME, OIL_T, OIL_P, VOLT, AFR ,EGT, BRAKES, AMBIENT");
}

void DrawGauges()
{
        int i;


        if (time>timeL)
        {
        u8g.firstPage();
        do
        {
          u8g.setFont(u8g_font_fub20);
          u8g.setPrintPos(0, 20);
        // Add extra info to OLED and make precount for VFD
                switch (LOGO_STATUS)
                {
                case STATE_OIL:
                        u8g.print(OIL_PRESSURE);u8g.print("bar");
                        u8g.setPrintPos(0, 60);
                        u8g.print(int(OIL_TEMP));u8g.print(char(176));u8g.print("C");
                        break;
                case STATE_EXHAUST:
                        u8g.print("Exhausts");
                        u8g.setPrintPos(0, 60);
                        u8g.print(int(EGT));u8g.print(char(176));u8g.print("C");
                        break;
                case STATE_BRAKES:
                u8g.print("Brakes");
                u8g.setPrintPos(0, 60);
                        u8g.print(int(BRAKES_TEMP));u8g.print(char(176));u8g.print("C");
                        break;
                case STATE_VOLT:
                u8g.print("Battery");
                u8g.setPrintPos(0, 60);
                        u8g.print(VOLTAGE);
                        u8g.print("V");
                        break;
                }
          } while( u8g.nextPage() );
        }
        switch (LOGO_STATUS)
        {
              case STATE_OIL:
                      TARGETPOS_L = int(OIL_PRESSURE*3.5); /* 0 -  3  - 6  */
                      TARGETPOS_R = int(OIL_TEMP/40);      /* 0 - 120 - 240 */
                      break;
              case STATE_EXHAUST:
                      TARGETPOS_L = int(AFR*20);           /* 0 - 0.5 - 1   */
                      TARGETPOS_R = int(EGT/100);          /* 0 - 300 - 700 */
                      break;
              case STATE_BRAKES:
                      TARGETPOS_R = int(BRAKES_TEMP/43);   /* 0 - 130 - 300 */
                      break;
              case STATE_VOLT:
                      TARGETPOS_L = int( VOLTAGE / 1.4 );  /* 0 - 14  - 28  */
                      break;
        }
        // Zero negative vaules and fix over values
        if ( TARGETPOS_L < 0 ) TARGETPOS_L = 0;
        if ( TARGETPOS_L > 20) TARGETPOS_L = 20;
        if ( TARGETPOS_R < 0 ) TARGETPOS_R = 0;
        if ( TARGETPOS_R > 7 ) TARGETPOS_R = 7;

        // Arrage positions
        if ( POSITION_L < TARGETPOS_L ) POSITION_L++;
        if ( POSITION_L > TARGETPOS_L ) POSITION_L--;
        if ( POSITION_R < TARGETPOS_R ) POSITION_R++;
        if ( POSITION_R > TARGETPOS_R ) POSITION_R--;

        digitalWrite (SI_PIN, LOW);
        digitalWrite (SCK_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);
        // Walk over gauge indicator positions
        // 5-24 Fuel (righ - left)
        // 31-37 Temp (right - left)

        for (i = 0; i <= 43; i++)
        {
               digitalWrite (SCK_PIN, LOW);
                if ((DRAW_L && (i == 26 )) ||    // Draw left gauge
                    (DRAW_R && (i == 38 || i == 41 )) || // Draw Right gauge
                    (DRAW_R && DRAW_RL && ( i==39 || i == 40 || i == 42 || i == 43)) // Draw Right gauge letters
                    ) digitalWrite (SI_PIN, HIGH);
                if ((i >= 5 && i <= 25) && DRAW_L ) {      // LEFT 20
                        if ((TYPE_L==BAR) && (POSITION_L<=(i-5))) digitalWrite (SI_PIN, HIGH);
                        if ((TYPE_L==NEEDLE) && (POSITION_L==(i-5))) digitalWrite (SI_PIN, HIGH);
                }

                if ((i >= 31 && i <= 37) && DRAW_R ) {     // RIGHT 6
                        if ((TYPE_R==BAR) && (POSITION_R<=(i-31))) digitalWrite (SI_PIN, HIGH);
                        if ((TYPE_R==NEEDLE) && (POSITION_R==(i-31))) digitalWrite (SI_PIN, HIGH);
                }
                digitalWrite (SCK_PIN, HIGH);
                digitalWrite (SI_PIN, LOW);
        }
        // LATCH AND HOLD
        digitalWrite (LH_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);
        // DONE
}

void ReadSensors() {
        int val;
        float vout;
        // Oil temperature with NTC thermocouple
//        val = ads.readADC_SingleEnded(OIL_TEMP_SENSOR);
        val = analogRead(OIL_TEMP_SENSOR);
        val=((1024 * val / R3 ) - val);
        val = log(val);
        OIL_TEMP = 1 / (0.0016 + (0.0002 * val) + (1.0813481443911684e-7 * val * val * val));

        // Oil pressure
//        val = ads.readADC_SingleEnded(OIL_PRESSURE_SENSOR);
        val = analogRead(OIL_PRESSURE_SENSOR);
        val = val * (val/1023.0) / 5;
        OIL_PRESSURE = ((2.1271 * val) + 5.1075 ) * val - 0.2561;

        // Voltmeter
//        val = ads.readADC_SingleEnded(VOLTMETER_SENSOR);
        val = analogRead(VOLTMETER_SENSOR);
        vout = (val * 5.0) / 1024.0;
        VOLTAGE = vout / (R2 / (R1 + R2));

        // AirFuelRatio
        AFR = ads.readADC_SingleEnded(AFR_INPUT);

        // Exhaust Gas Temtrature
        EGT = thermocouple.readCelsius();

        // Brakes temperature
        BRAKES_TEMP=mlx.readObjectTempC();

        // Brakes ambient temperature for datalog only
        val=mlx.readAmbientTempC();

        // Data logging
        time = millis();
        if (time >= timeP)
        {
        Serial.print(time);
        Serial.print(", "); Serial.print(OIL_TEMP);
        Serial.print(", "); Serial.print(OIL_PRESSURE);
        Serial.print(", "); Serial.print(VOLTAGE);
        Serial.print(", "); Serial.print(AFR);
        Serial.print(", "); Serial.print(EGT);
        Serial.print(", "); Serial.print(BRAKES_TEMP);
        Serial.print(", "); Serial.print(val);
        Serial.print("\n");
        timeP=time+LOG_DELAY;
        }
}

/*
    MAIN GAUGES LOOP
 */
void loop() {

        // Read all sensors and dataloging
        ReadSensors();

        // Dimmer signal check
        DIMMER_STATE = digitalRead(DIMMER_PIN);
        if (DIMMER_STATE != DIMMER_PREVSTATE)
        {
                if (DIMMER_STATE == HIGH)
                        u8g.setContrast(70);
                else
                        u8g.setContrast(255);
                DIMMER_PREVSTATE = DIMMER_STATE;
        }

        // Click button operation
        BUTTON_STATE = digitalRead(BUTTON_PIN);
        if (BUTTON_STATE == HIGH)
        {
                if (BUTTON_PREVSTATE != BUTTON_STATE) /* JUST PRESSED */
                {
                        LOGO_STATUS++;
                        if (LOGO_STATUS > MAX_LOGO) LOGO_STATUS = 1;
                        switch (LOGO_STATUS)
                        {
                        case STATE_OIL:     DRAW_R = true; DRAW_RL = true; DRAW_L = true; TYPE_R = BAR; TYPE_L = BAR;break;
                        case STATE_EXHAUST: DRAW_R = true; DRAW_RL = true; DRAW_L = true; TYPE_R = BAR; TYPE_L = NEEDLE;break;
                        case STATE_BRAKES:  DRAW_R = true; DRAW_RL = true; DRAW_L = false;TYPE_R = BAR; TYPE_L = NONE;break;
                        case STATE_VOLT:    DRAW_R = false;DRAW_RL = false;DRAW_L = true; TYPE_R = NONE;TYPE_L = NEEDLE;break;
                        }
                        SHOW_LOGO = true;
                }
        }
        BUTTON_PREVSTATE = BUTTON_STATE;

        //Change logo at OLED display
        if (SHOW_LOGO)
        {
                u8g.firstPage();
                do {
                        switch (LOGO_STATUS) {
                        case STATE_OIL:     u8g.drawXBMP( 0, 0, 128, 64, oil_LOGO); break;
                        case STATE_EXHAUST: u8g.drawXBMP( 0, 0, 128, 64, exhaust_LOGO); break;
                        case STATE_BRAKES:  u8g.drawXBMP( 0, 0, 128, 64, brakes_LOGO); break;
                        case STATE_VOLT:    u8g.drawXBMP( 0, 0, 128, 64, battery_LOGO); break;
                        default:         break;
                        }
                } while( u8g.nextPage() );
                SHOW_LOGO = false;
                timeL=time+LOGO_DELAY;
        }

        //Draw VFD gauges
        DrawGauges();

#ifdef VISUAL_DELAY
        delay(VISUAL_DELAY);
#endif

}
