/*
//      _______ __  _________ _________
//      \_     |  |/  .  \   |   \ ___/
//        / :  |   \  /   \  |   / _>_
//       /      \ | \ |   /  |  /     \
//      /   |___/___/____/ \___/_     /
//      \___/--------TECH--------\___/
//       ==== ABOVE SCIENCE 1994 ====
//
//   Ab0VE-TECH - Honda Prelude Gen4 VFD Gauges controller
*/

#include <SPI.h>
#include <avr/pgmspace.h>
#include <Wire.h>
//#include <Adafruit_SSD1306.h> // platformio lib install "Adafruit SSD1306"
//#include <Adafruit_GFX.h>
#include <U8glib.h> //platformio lib install "U8glib"
#include <Adafruit_ADS1015.h> // platformio lib install "Adafruit ADS1X15"
#include <Adafruit_MLX90614.h> // platformio lib install "Adafruit MLX90614 Library"
#include <max6675.h> //0

#include "img.h"

int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI
Adafruit_ADS1115 ads;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX6675 thermocouple;

#define VISUAL_DELAY  40 // Refresh rate

// SENSORS
/* OIL TRESSURE / OIL TEMPERATURE */
#define OIL_TEMP_SENSOR     A0
float OIL_TEMP = 0;
/*
    OIL TEMP SENSOR CHART
   C°  Ohm
   20  2031 - 0
   30  1286 - 0
   40  844 - COOL 1
   50  570 - 1
   60  388 - 2
   70  278  -2
   80  200 - 2

   90  147 - 3
   100 108  - 3
   110 83 - NORMAL 3
   120 63 - 3
   130 49 - 4
   140 39 - 5
   150 30 - HIGH 6
   160 24 - RED 7 // BUZZER
   170 20 - 7     // BUZZRT
 */

#define OIL_PRESSURE_SENSOR A1
float OIL_PRESSURE = 0;
/*
    OIL PRESSURE CHART
    <50 - 0
    50  - 1
    70kPa  - IDLE     10psi - 2
    350kPa - 3000rpm  50psi - 10
    650kPa - TOP      90psi - 20
 */

/* VOLTMETER    /        - */
#define R1 100000.0 // resistance of R1 (100K) in voltage divider
#define R2 10000.0  // resistance of R2 (10K) in voltage divider
#define VOLTMETER_SENSOR    A2
float VOLTAGE = 0;

/*    -         / BRAKES TEMPERATURE
      SDL-----------A4
      SDA-----------A5
*/
// i2c - PIN A4/A5
float BRAKES_TEMP = 0;

/* 02 LAMDA     / EXHAUST TEMPERATURE */
#define LAMBDA_SENSOR     A3
float AFR = 0;
float EGT = 0;

int DIMMER_PIN = 3; // Dimer input - High +12V!

/*
     GAUGES      arduino
     SI----------8
     SCK---------7
     LH----------6
 */
int SI_PIN  = 8; // Blue
int SCK_PIN = 7; // Green
int LH_PIN  = 6; // Gray

/*
     OLED         arduino
     SDL-----------A4
     SDA-----------A5
 */
int BUTTON_PIN = 12; // BUTTON to Ground

int BUTTON_STATE = 0;
int BUTTON_PREVSTATE = 0;

int DIMMER_STATE = 0;
int DIMMER_PREVSTATE = 0;

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

byte INERTION_L = 10;
byte INERTION_R = 50;

#define STATE_OIL     1
#define STATE_EXHAUST 2
#define STATE_BRAKES  3
#define STATE_VOLT    4
boolean SHOW_LOGO = true;
int LOGO_STATUS = STATE_OIL;
#define MAX_LOGO      4

void SetContrastControl(uint8_t contrast) {
  u8g.setContrast(contrast);
}


void drawlogo(void) {
  u8g.drawXBMP( 0, 0, 128, 64, Prelude_LOGO);
//  u8g.setFont(u8g_font_unifont);
//  u8g.setPrintPos(0, 64);
//  u8g.print("Ab0VE-TECH");
}

void setup() {
        Serial.begin(115200);
        Serial.println("Ab0VE-TECH Honda Prelude Gauges contoller");
        Serial.println("TIME, OIL_T, OIL_P, VOLT, AFR ,EGT, BRAKES");

        Wire.begin(); // Init I2C bus
        ads.begin();
 //                                                     ADS1015 ADS1115
 // ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V 1 bit = 3mV     0.1875mV
 // ads.setGain(GAIN_ONE);        // +/- 4.096V 1 bit = 2mV     0.125mV
 // ads.setGain(GAIN_TWO);        // +/- 2.048V 1 bit = 1mV     0.0625mV
 // ads.setGain(GAIN_FOUR);       // +/- 1.024V 1 bit = 0.5mV   0.03125mV
 // ads.setGain(GAIN_EIGHT);      // +/- 0.512V 1 bit = 0.25mV  0.015625mV
 // ads.setGain(GAIN_SIXTEEN);    // +/- 0.256V 1 bit = 0.125mV 0.0078125mV
        ads.setGain(GAIN_FOUR); // +1.024V MAX!

        thermocouple.begin(thermoCLK, thermoCS, thermoDO);

        mlx.begin();

// init VFD
        pinMode (SI_PIN, OUTPUT);
        pinMode (SCK_PIN, OUTPUT);
        pinMode (LH_PIN, OUTPUT);
        pinMode (BUTTON_PIN, INPUT);
        pinMode (DIMMER_PIN, INPUT);
        digitalWrite (SI_PIN, LOW);
        digitalWrite (SCK_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);

        SetContrastControl(0xff);
        u8g.firstPage();
        u8g.setRot180();
//        display.drawBitmap(0, 0, Prelude_LOGO, 128, 64, WHITE);
//        display.setTextSize(2);
//        display.setTextColor(WHITE);
//        display.setCursor(0, 0);
//        display.println("Ab0VE TECH");
//        display.display();
        do {
            drawlogo();
          } while( u8g.nextPage() );
        delay(1000);

}

// FAKE VAR
int J = 0;

void DrawGauges()
{
        int i;

        // Walk over gauge indicator positions
        // 5-24 Fuel (righ - left)
        // 31-37 Temp (right - left)
        for (i = 0; i < 42; i++)
        {
                digitalWrite (SCK_PIN, LOW);
                if (
                        (DRAW_L && (i == 24 || i == 25)) || // Draw left gauge
                        (DRAW_R && (i == 37 || i == 38 || i == 39)) || // Draw Right gauge
                        (DRAW_R && DRAW_RL && (i == 40 || i == 41)) // Draw Right gauge letters
                        ) digitalWrite (SI_PIN, HIGH);
                else
                { // MAIN Routine
                        if (i >= 5 && i <= 23)
                        { // LEFT

                        } else if (i >= 31 && i <= 36)
                        { // RIGHT

                        }
                        if (i > J) digitalWrite (SI_PIN, HIGH);
                        else digitalWrite (SI_PIN, LOW);
                }
                digitalWrite (SCK_PIN, HIGH);
                digitalWrite (SI_PIN, LOW);
        }
        // LATCH AND HOLD
        digitalWrite (LH_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);
}

void ReadSensors() {
        int val;
        float vout;
        unsigned long time;

        // Oil temperature
        val = analogRead(OIL_TEMP_SENSOR);

        // Oil pressure
        val = analogRead(OIL_PRESSURE_SENSOR);

        // Voltmeter
        val = analogRead(VOLTMETER_SENSOR);
        vout = (val * 5.0) / 1024.0;
        VOLTAGE = vout / (R2 / (R1 + R2));

        // AirFuelRatio
        //val = analogRead(LAMBDA_SENSOR);
        AFR = ads.getLastConversionResults();

        // Exhaust Gas Temtrature
        EGT = thermocouple.readCelsius();

        // Brakes temperature
        BRAKES_TEMP=mlx.readObjectTempC();  // mlx.readAmbientTempC();

        // Data logging
        time = millis();
        Serial.print(time);
        Serial.print(", "); Serial.print(OIL_TEMP);
        Serial.print(", "); Serial.print(OIL_PRESSURE);
        Serial.print(", "); Serial.print(VOLTAGE);
        Serial.print(", "); Serial.print(AFR);
        Serial.print(", "); Serial.print(EGT);
        Serial.print(", "); Serial.print(BRAKES_TEMP);
        Serial.print("\n");


}



/*
    MAIN GAUGES LOOP
 */
void loop() {



        // DIMMER
        DIMMER_STATE = digitalRead(DIMMER_PIN);
        if (DIMMER_STATE != DIMMER_PREVSTATE)
        {
                if (DIMMER_STATE == HIGH)
                  SetContrastControl(70);
                else
                  SetContrastControl(255);
                DIMMER_PREVSTATE = DIMMER_STATE;
        }

        // Read all sensors and dataloging
        ReadSensors();

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
                        case STATE_OIL:
                                DRAW_R = true;
                                DRAW_RL = true; // C - H
                                DRAW_L = true;
                                TYPE_R = BAR;
                                TYPE_L = BAR;
                                break;
                        case STATE_EXHAUST:
                                DRAW_R = true;
                                DRAW_RL = true; // C - H
                                DRAW_L = true;
                                TYPE_R = BAR;
                                TYPE_L = NEEDLE;
                                break;
                        case STATE_BRAKES:
                                DRAW_R = true;
                                DRAW_RL = true; // C - H
                                DRAW_L = false;
                                TYPE_R = BAR;
                                TYPE_L = NONE;
                                break;
                        case STATE_VOLT:
                                DRAW_R = false;
                                DRAW_RL = false; // C - H
                                DRAW_L = true;
                                TYPE_R = NONE;
                                TYPE_L = NEEDLE;
                                break;
                        }
                        SHOW_LOGO = true;
                }
        }
        BUTTON_PREVSTATE = BUTTON_STATE;

        //Change logo at OLED display
        if (SHOW_LOGO)
        {
          Serial.print("LOGO CHANGE!!!");
          Serial.print(", "); Serial.print(LOGO_STATUS);
          Serial.print("\n");

          u8g.firstPage();
          do {
                switch (LOGO_STATUS) {
                case STATE_OIL:     u8g.drawXBMP( 0, 0, 128, 64, oil_LOGO);break;
                case STATE_EXHAUST:   u8g.drawXBMP( 0, 0, 128, 64, exhaust_LOGO);break;
                case STATE_BRAKES:    u8g.drawXBMP( 0, 0, 128, 64, brakes_LOGO);break;
                case STATE_VOLT:      u8g.drawXBMP( 0, 0, 128, 64, battery_LOGO);break;
                default:         break;
                }
              } while( u8g.nextPage() );
          SHOW_LOGO = false;
        }

        //Draw VFD gauges
        DrawGauges();

        delay(VISUAL_DELAY);
}
