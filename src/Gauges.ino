/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //        ==== ABOVE SINCE 1994 ====
   //
   //   Ab0VE TECH - HONDA Prelude Gen4 VFD Gauges controller
 */

/* Controller connections
   //            +-----------+
   //            • TX    Vin •
   //            • RX  A Gnd •  <- GND
   //            • RST R RST •
   //     GND -> • GND D  +5 •  <- +5V Reg. LM2596HV
   //  MAX DO -> • 2   U  A7 •
   //  MAX CS <- • 3   I  A6 •
   // MAX CLK <- • 4   N  A5 • -> OLED/ADS/MLX SCL
   //   Alarm <- • 5   O  A4 • -> OLED/ADS/MLX SDA
   //  VFD LH <- • 6      A3 •
   // VFD SCK <- • 7   N  A2 • <- Battery R1/R2 devider
   //  VFD SI <- • 8   A  A1 • <- OIL Pressure/R4 devider
   //            • 9   N  A0 • <- OIL Temp/R3 devider
   //            • 10  O Arf •
   //            • 11    3V3 •
   //  Button -> • 12 ||| 13 • <- DIMMER
   //            +-----------+
 */

/******** TODO **********
   EGT                   ❏
 *************************/

#include <Arduino.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <U8glib.h> //platformio lib install "U8glib"
#include <Adafruit_ADS1015.h> // platformio lib install "Adafruit ADS1X15"
#include <Adafruit_MLX90614.h> // platformio lib install "Adafruit MLX90614 Library"
#include <max6675.h> // platformio lib install "MAX6675"

#include "img.h" // Mode logos

U8GLIB_SSD1306_128X64_2X u8g(U8G_I2C_OPT_NONE);
Adafruit_ADS1115 ads;
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX6675 thermocouple;

#define VISUAL_DELAY  1 // Refresh delay

//
// SENSORS
//
/* OIL TRESSURE / OIL TEMPERATURE */
#define OIL_TEMP_SENSOR     A0
#define R3 9920.0 // resistance of R3 (10K) in voltage devider
float OIL_TEMP = 0;
#define THERMISTOR_NOMINAL 1400
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURE_NOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define B_COEFFICIENT 3950

#define SENSORS_DELAY 2
#define NUM_SAMPLES  5
uint16_t samples[NUM_SAMPLES];

#define OIL_PRESSURE_SENSOR A1
#define R4 994.0 // resistance of R4 (1K) in voltage devider
float OIL_PRESSURE = 0;

/* VOLTMETER    /        - */
#define VOLTMETER_SENSOR    A2
#define R1 47000.0 // resistance of R1 (47K ) in voltage divider
#define R2 9950.0  // resistance of R2 (10K) in voltage divider
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
   MAX6675
 */
int thermoDO  = 2;
int thermoCS  = 3;
int thermoCLK = 4;

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

int DIMMER_PIN = 13; // Dimer input - High +12V!
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

#define DATALOG_ENABLE        // SERIAL DATALOGING
unsigned long timeP=0;
#define LOG_DELAY 1000

// Delay for data update on OLED after new logo shown
unsigned long timeL=0;
#define LOGO_DELAY 3000

// OLED data update delay
unsigned long timeOLED=0;
#define OLED_DELAY 500

// ALARMS
#define ALARM_PIN 5

#define ALARM_EGT 1000          // Exhaust Temtrature too high
#define ALARM_OIL 0.5           // Oil pressure is too low
#define ALARM_TEMP 140          // Oil temperature is too high
#define ALARM_BRAKES 350        // Brakes temperature is too high
#define ALARM_BATTERY_LOW 12.0 // Alternator output is too low
#define ALARM_BATTERY_HIGH 15.0 // Alternator output it too high

boolean ALARM_STATUS = false;
boolean ALARM_BLINK;

void setup() {
// pins config
        pinMode (SI_PIN, OUTPUT);
        pinMode (SCK_PIN, OUTPUT);
        pinMode (LH_PIN, OUTPUT);
// init VFD
        digitalWrite (SI_PIN, LOW);
        digitalWrite (SCK_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);
        digitalWrite (LH_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);
// configure rest pins
        pinMode (OIL_PRESSURE_SENSOR, INPUT);
        pinMode(OIL_TEMP_SENSOR, INPUT);
        pinMode (BUTTON_PIN, INPUT_PULLUP);
        pinMode (DIMMER_PIN, INPUT_PULLUP);
        pinMode (ALARM_PIN, OUTPUT);
        digitalWrite(ALARM_PIN, LOW);

#ifdef DATALOG_ENABLE
        Serial.begin(115200);
        Serial.println("Ab0VE-TECH Honda Prelude Gauges contoller");
#endif

// OLED INIT
        u8g.setContrast(0xff);
        u8g.firstPage();
        u8g.setRot180(); // Rorate screen with wires goes down
        do {
                u8g.drawXBMP( 0, 0, 128, 64, Prelude_LOGO);
        } while( u8g.nextPage() );

// AD initn
        ads.begin();
        ads.setGain(GAIN_TWO); // ADS1115: +2.048V/0.0625mV

// EGR sensor
        thermocouple.begin(thermoCLK, thermoCS, thermoDO);

// PIR brake rotor sensor
        mlx.begin();

        delay(1500);

#ifdef DATALOG_ENABLE
        Serial.println("TIME, OIL_T, OIL_P, VOLT, AFR ,EGT, BRAKES, AMBIENT");
#endif
}

void LogoSetup() {
        switch (LOGO_STATUS)
        {
        case STATE_OIL:
                DRAW_R = true;
                DRAW_RL = true;
                DRAW_L = true;
                TYPE_R = BAR;
                TYPE_L = BAR;
                break;
        case STATE_EXHAUST:
                DRAW_R = true;
                DRAW_RL = true;
                DRAW_L = true;
                TYPE_R = BAR;
                TYPE_L = NEEDLE;
                break;
        case STATE_BRAKES:
                DRAW_R = true;
                DRAW_RL = true;
                DRAW_L = false;
                TYPE_R = BAR;
                TYPE_L = NONE;
                break;
        case STATE_VOLT:
                DRAW_R = false;
                DRAW_RL = false;
                DRAW_L = true;
                TYPE_R = NONE;
                TYPE_L = NEEDLE;
                break;
        }
}

void DrawGauges()
{
        int i;
        if (time>timeL) // Delay while new LOGO displayed
        {
                if (time>timeOLED) // Delay for DATA update
                {
                        u8g.firstPage();
                        timeOLED=time+OLED_DELAY;
                        do
                        {
                                u8g.setDefaultForegroundColor();
                                if (ALARM_STATUS && ALARM_BLINK) // INVERT SCREEN ON ALARM
                                {
                                        digitalWrite(ALARM_PIN,HIGH);
                                        u8g.drawBox(0, 0, 128, 64);
                                        u8g.setDefaultBackgroundColor();
                                } else digitalWrite(ALARM_PIN,LOW);
                                u8g.setFont(u8g_font_fub20);
                                u8g.setPrintPos(0, 23);
                                // Add extra info to OLED
                                switch (LOGO_STATUS)
                                {
                                case STATE_OIL:
                                        u8g.print(OIL_PRESSURE); u8g.print("bar");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(OIL_TEMP)); u8g.print(char(176)); u8g.print("C");
                                        break;
                                case STATE_EXHAUST:
                                        u8g.print("Exhausts");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(EGT)); u8g.print(char(176)); u8g.print("C");
                                        break;
                                case STATE_BRAKES:
                                        u8g.print("Brakes");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(BRAKES_TEMP)); u8g.print(char(176)); u8g.print("C");
                                        break;
                                case STATE_VOLT:
                                        u8g.print("Battery");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(VOLTAGE);
                                        u8g.print("V");
                                        break;
                                }
                        } while( u8g.nextPage() );
                        ALARM_BLINK=!ALARM_BLINK;
                }
        }
        // Process VFD scale position
        switch (LOGO_STATUS)
        {
        case STATE_OIL:
                TARGETPOS_L = 20-int(OIL_PRESSURE*2);     /* 0 -   5 - 10   */
                TARGETPOS_R = 8;                            /* NONE SHOW */
                if (OIL_TEMP<40) TARGETPOS_R=8;
                else if (OIL_TEMP<80) TARGETPOS_R=7;
                else if (OIL_TEMP<110) TARGETPOS_R=6;       /* CENTER */
                else if (OIL_TEMP<120) TARGETPOS_R=5;
                else if (OIL_TEMP<130) TARGETPOS_R=4;
                else if (OIL_TEMP<140) TARGETPOS_R=3;
                else if (OIL_TEMP<150) TARGETPOS_R=2;
                else TARGETPOS_R=1;                         /* RED */
                break;
        case STATE_EXHAUST:
                TARGETPOS_L = 20-int(AFR*20);                /* 0 - 0.5 - 1   */
                TARGETPOS_R = 7-int(EGT/100);                /* 0 - 300 - 700 */
                break;
        case STATE_BRAKES:
                TARGETPOS_R = 7-int(BRAKES_TEMP/54);         /* 0 - 130 - 300 */
                break;
        case STATE_VOLT:
                TARGETPOS_L = 20-int( VOLTAGE - 8)*2;        /* 0 - 14  - 28  */
                break;
        }

        // Zero negative vaules and fix over values
        if ( TARGETPOS_L < 1 ) TARGETPOS_L = 1;
        if ( TARGETPOS_L > 20) TARGETPOS_L = 20;
        if ( TARGETPOS_R < 1 ) TARGETPOS_R = 1;
        if ( TARGETPOS_R > 8 ) TARGETPOS_R = 8;

        // Arrage positions
        if ( POSITION_L < TARGETPOS_L ) POSITION_L++;
        if ( POSITION_L > TARGETPOS_L ) POSITION_L--;
        if ( POSITION_R < TARGETPOS_R ) POSITION_R++;
        if ( POSITION_R > TARGETPOS_R ) POSITION_R--;

        // INIT VFD display
        digitalWrite (SI_PIN, LOW);
        digitalWrite (SCK_PIN, HIGH);
        digitalWrite (LH_PIN, LOW);

        // Walk over gauge indicator positions
        for (i = 0; i <= 43; i++)
        {
                digitalWrite (SCK_PIN, LOW);
                if ((DRAW_L && (i == 26 || i == 27 || i == 28 )) ||    // Draw left gauge
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
        uint16_t val;
        uint8_t i;
        float vout;

        // Oil temperature with NTC thermocouple
        // take N samples in a row, with a slight delay
        for (i=0; i< NUM_SAMPLES; i++) {
                samples[i] = analogRead(OIL_TEMP_SENSOR);
                delay(SENSORS_DELAY);
        }
        val = 0;
        for (i=0; i< NUM_SAMPLES; i++)
                val += samples[i];
        val /= NUM_SAMPLES;
        OIL_TEMP = R3 * (1024.0 / val - 1);
        OIL_TEMP = OIL_TEMP / THERMISTOR_NOMINAL;         // (R/Ro)
        OIL_TEMP = log(OIL_TEMP);                         // ln(R/Ro)
        OIL_TEMP /= B_COEFFICIENT;                        // 1/B * ln(R/Ro)
        OIL_TEMP += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
        OIL_TEMP = 1.0 / OIL_TEMP;                        // Invert
        OIL_TEMP -= 273.15;
        if (OIL_TEMP > 999) OIL_TEMP=999;

        // Oil pressure
        for (i=0; i< NUM_SAMPLES; i++) {
                samples[i] = analogRead(OIL_PRESSURE_SENSOR);
                delay(SENSORS_DELAY);
        }
        val = 0;
        for (i=0; i< NUM_SAMPLES; i++)
                val += samples[i];
        val /= NUM_SAMPLES;
        int vTimesTen=map(val,10,184,0,100); // linear 1
        OIL_PRESSURE = vTimesTen/10.0;

        // Voltmeter
//        val = ads.readADC_SingleEnded(VOLTMETER_SENSOR);
        for (i=0; i< NUM_SAMPLES; i++) {
                samples[i] = analogRead(VOLTMETER_SENSOR);
                delay(SENSORS_DELAY);
        }
        val = 0;
        for (i=0; i< NUM_SAMPLES; i++)
                val += samples[i];
        val /= NUM_SAMPLES;
        vout = ((val) * 5.0) / 1023.0;
        VOLTAGE = vout / (R2 / (R1 + R2));

        // AirFuelRatio
        //ADS1115: +2.048V/0.0625mV
        AFR = (ads.readADC_SingleEnded(AFR_INPUT) * 0.0625) / 1000;

        // Exhaust Gas Temtrature
        EGT = thermocouple.readCelsius();
        if ((EGT<0) || (EGT>1000)) EGT=0;

        // Brakes temperature
        BRAKES_TEMP=mlx.readObjectTempC();
        if (BRAKES_TEMP > 380) BRAKES_TEMP=380;
        // Brakes ambient temperature for datalog only
        val=mlx.readAmbientTempC();

        // ALARM SYSTEM
        // alrms order from lower to  upper priority
        ALARM_STATUS=false;
        // EGT to High
        if (EGT>ALARM_EGT) {
                LOGO_STATUS=STATE_EXHAUST;
                ALARM_STATUS=true;
        }
        // Brakes temprature to high
        if (BRAKES_TEMP>ALARM_BRAKES) {
                LOGO_STATUS=STATE_BRAKES;
                ALARM_STATUS=true;
        }
        // Overcharge
        if (VOLTAGE>ALARM_BATTERY_HIGH) {
                LOGO_STATUS=VOLTAGE;
                ALARM_STATUS=true;
        }
        // No charge
        if ((VOLTAGE<ALARM_BATTERY_LOW) && (OIL_PRESSURE>1.0)) { // while engine running
                LOGO_STATUS=STATE_VOLT;
                ALARM_STATUS=true;
        }
        // High OIL temperature
        if (OIL_TEMP>ALARM_TEMP) {
                LOGO_STATUS=STATE_OIL;
                ALARM_STATUS=true;
        }
        // Low OIL pressure
        if ((OIL_PRESSURE<ALARM_OIL) && (VOLTAGE>ALARM_BATTERY_LOW)) { // while engine runnin
                LOGO_STATUS=STATE_OIL;
                ALARM_STATUS=true;
        }

        if (ALARM_STATUS) {
//                ALARM_BLINK=true;
                SHOW_LOGO=false;
                LogoSetup();
        }
        else digitalWrite(ALARM_PIN,LOW);

        // Data logging
        time = millis();
#ifdef DATALOG_ENABLE
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
#endif
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
                        u8g.setContrast(50);
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
                        LogoSetup();
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
                        case STATE_OIL:
                                u8g.drawXBMP( 0, 0, 128, 64, oil_LOGO);
                                break;
                        case STATE_EXHAUST:
                                u8g.drawXBMP( 0, 0, 128, 64, exhaust_LOGO);
                                break;
                        case STATE_BRAKES:
                                u8g.drawXBMP( 0, 0, 128, 64, brakes_LOGO);
                                break;
                        case STATE_VOLT:
                                u8g.drawXBMP( 0, 0, 128, 64, battery_LOGO);
                                break;
                        default:
                                break;
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
