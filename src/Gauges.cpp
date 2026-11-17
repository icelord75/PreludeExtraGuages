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

#include <Arduino.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <U8glib.h>            //platformio lib install "U8glib"
#include <Adafruit_ADS1X15.h>  // platformio lib install "Adafruit ADS1X15"
#include <Adafruit_MLX90614.h> // platformio lib install "Adafruit MLX90614 Library"
#include <max6675.h>           // platformio lib install "MAX6675"
#include "img.h"               // Mode logos

#define DATALOG_ENABLE        // SERIAL DATALOGING

U8GLIB_SSD1306_128X64_2X u8g(U8G_I2C_OPT_NONE);
Adafruit_ADS1X15 ads;
// Adafruit_ADS1X15 ads1115(0x48); // construct an ads1115 at address 0x48
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX6675 thermocouple;

//
// SENSORS
//
/* OIL TRESSURE / OIL TEMPERATURE */
#define OIL_TEMP_SENSOR A0
#define R3 9920.0 // exact resistance of R3 (10K) in voltage devider
float OIL_TEMP = 0;
#define THERMISTOR_NOMINAL 1400
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURE_NOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define B_COEFFICIENT 3500


#define OIL_PRESSURE_SENSOR A1
#define R4 199.0                                 // exact resistance of R4 (1K) in voltage devider
#define MIN_OHM_PRESS 10                        // 10ohm - 0 BAR
#define MAX_OHM_PRESS 180                       // 180phm - 10 BAR
#define OIL_SENSOR_TYPE 10.0                     // 10 BAR
// #define OIL_SENSOR_TYPE 20.0                 // 5 BAR
#define MIN_VOLT_PRESS 1023 * (MIN_OHM_PRESS / (MIN_OHM_PRESS + R4))   // 5v=1023 on voltage devider
#define MAX_VOLT_PRESS 1023 * (MAX_OHM_PRESS / (MAX_OHM_PRESS + R4)) // 5v=1023 on voltage devider
float OIL_PRESSURE = 0;

/* VOLTMETER    /        - */
#define VOLTMETER_SENSOR A2
#define R1 47000.0 // exact resistance of R1 (47K ) in voltage divider
#define R2 9950.0  // exact resistance of R2 (10K) in voltage divider
float VOLTAGE = 0;

/*    -         / BRAKES TEMPERATURE
   MLX90614
      SDA-----------A4
      SCL-----------A5
 */
float BRAKES_TEMP = 0;

/* O2 LAMBDA   / EXHAUST TEMPERATURE
   ADS1X15            arduino
     SDA-------------A4
     SCL-------------A5
   MAX6675 (software SPI)
     DO--------------2
     CS--------------3
     CLK-------------4
 */
int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 4;

#define AFR_INPUT 0
float AFR = 0;
float EGT = 0;
unsigned long timeEGT = 0;
#define EGT_DELAY 250 // MAX6675 conversion time is up to 220ms

/*
     VFD Contoller
     μPD6232C    arduino
     SI-------------8 BLue
     SCK------------7 red
     LH-------------6 green
 */
int SI_PIN = 8;  // Blue
int SCK_PIN = 7; // Green
int LH_PIN = 6;  // Gray

// Dimer input - High +12V! Must go through a divider/optocoupler to 5V.
// NOTE: D13 has the onboard LED, so INPUT_PULLUP is weak here - use an external pull-up.
int DIMMER_PIN = 13;
int DIMMER_STATE = 0;
int DIMMER_PREVSTATE = 0;

int BUTTON_PIN = 12; // BUTTON to Ground
int _STATE = 0;
int BUTTON_STATE = 0;
int BUTTON_PREVSTATE = 0;

/*
     OLED DISPLAY  arduino
     SDA-------------A4
     SCL-------------A5
 */
#define NONE 0
#define BAR 1
#define NEEDLE 2

boolean DRAW_R = true;
boolean DRAW_RL = true; // C - H
boolean DRAW_L = true;
byte TYPE_R = BAR;
byte TYPE_L = BAR;

byte POSITION_L = 0;
byte TARGETPOS_L = 0;
byte POSITION_R = 0;
byte TARGETPOS_R = 0;

#define STATE_OIL 1
#define STATE_EXHAUST 2
#define STATE_BRAKES 3
#define STATE_VOLT 4

boolean SHOW_LOGO = true;
int LOGO_STATUS = STATE_OIL;
#define MAX_LOGO 4

unsigned long time = 0;

unsigned long timeP = 0;
#define LOG_DELAY 1000

// Delay for data update on OLED after new logo shown
unsigned long timeL = 0;
#define LOGO_DELAY 2000

// OLED data update delay
unsigned long timeOLED = 0;
#define OLED_DELAY 100

// DISPLAY INERTIA
// Temperatures and oil pressure are shown through smoothed "display" values.
// Every DISPLAY_DELAY ms each display value closes DISPLAY_INERTIA of the gap
// to the measured value, so a sudden jump is shown as a smooth glide
// (with 0.3 about 95% of a step is reached in ~2 seconds).
// Alarms always use the measured values, without this delay.
#define DISPLAY_DELAY 250     // ms between display value updates
#define DISPLAY_INERTIA 0.3f  // 0..1: smaller = smoother and slower
unsigned long timeDisplay = 0;
float DISP_OIL_TEMP = 0;
float DISP_OIL_PRESSURE = 0;
float DISP_EGT = 0;
float DISP_BRAKES_TEMP = 0;

// VFD gauges move one segment per GAUGE_STEP_DELAY ms
// (a full 20-segment sweep takes 20 * 40 = 800 ms).
#define GAUGE_STEP_DELAY 40
unsigned long timeGauge = 0;

// ALARMS
#define ALARM_PIN 5
#define ALARM_EGT 1000           // Exhaust Temtrature too high
#define ALARM_OIL 1              // Oil pressure is too low
#define ALARM_OIL_HIGH 7         // Oil pressure is too low
#define ALARM_TEMP 120           // Oil temperature is too high
#define ALARM_BRAKES 351         // Brakes temperature is too high
#define ALARM_BATTERY_LOW 12.5   // Alternator output is too low
#define ALARM_BATTERY_HIGH 15.0  // Alternator output it too high
#define ALARM_BETTERY_DELAY 2000 // 2.0sec delay for engine start
#define WORK_TEMP 40             // Monitor pressure only at work temperatures

boolean ALARM_STATUS = false;
boolean ALARM_BLINK;
uint32_t ALARM_TIME = 0;
float val_T = 0.0f;
float val_P = 0.0f;
float val_V = 0.0f;

static inline float clampf(float value, float minValue, float maxValue)
{
        if (value < minValue)
                return minValue;
        if (value > maxValue)
                return maxValue;
        return value;
}

static inline float readFilteredAnalog(uint8_t pin, float &filteredValue)
{
        filteredValue += (analogRead(pin) - filteredValue) * 0.1f;
        return filteredValue;
}

void setup()
{
        int i;
        // VFD pins config
        pinMode(SI_PIN, OUTPUT);
        pinMode(SCK_PIN, OUTPUT);
        pinMode(LH_PIN, OUTPUT);

        // init VFD
        digitalWrite(SI_PIN, LOW);
        digitalWrite(SCK_PIN, HIGH);
        digitalWrite(LH_PIN, LOW);
        // clear VFD
        digitalWrite(SI_PIN, LOW);
        for (i = 0; i <= 43; i++)
        {
                digitalWrite(SCK_PIN, LOW);
                digitalWrite(SCK_PIN, HIGH);
        }
        // latch VFD
        digitalWrite(LH_PIN, HIGH);
        digitalWrite(LH_PIN, LOW);

        // configure rest pins
        pinMode(OIL_PRESSURE_SENSOR, INPUT);
        pinMode(OIL_TEMP_SENSOR, INPUT);
        pinMode(BUTTON_PIN, INPUT_PULLUP);
        pinMode(DIMMER_PIN, INPUT_PULLUP);
        pinMode(ALARM_PIN, OUTPUT);
        digitalWrite(ALARM_PIN, LOW);

#ifdef DATALOG_ENABLE
        Serial.begin(115200);
        Serial.println("Ab0VE-TECH Honda Prelude Gauges contoller");
#endif

        // OLED INIT
        u8g.setContrast(0xff);
        u8g.firstPage();
        u8g.setRot180(); // Rorate screen with wires goes down
        do
        {
                DRAW_LOGO(Prelude_LOGO);
        } while (u8g.nextPage());

        // AD init
        ads.begin();
        ads.setGain(GAIN_TWO); // ADS1115: +2.048V/0.0625mV

        // EGR sensor
        thermocouple.begin(thermoCLK, thermoCS, thermoDO);

        // PIR brake rotor sensor
        mlx.begin();

        delay(1500); // Prelude logo delay

#ifdef DATALOG_ENABLE
        Serial.println("TIME, OIL_T, OIL_P, VOLT, AFR ,EGT, BRAKES, AMBIENT");
#endif
}

void LogoSetup()
{
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

// Move a display value a part of the way towards the measured value
static inline void approach(float &displayValue, float measuredValue)
{
        displayValue += (measuredValue - displayValue) * DISPLAY_INERTIA;
}

// Update smoothed display values at a fixed rate
void UpdateDisplayValues()
{
        if (time - timeDisplay < DISPLAY_DELAY)
                return;
        timeDisplay = time;
        approach(DISP_OIL_TEMP, OIL_TEMP);
        approach(DISP_OIL_PRESSURE, OIL_PRESSURE);
        approach(DISP_EGT, EGT);
        approach(DISP_BRAKES_TEMP, BRAKES_TEMP);
}

// fub20r has no degree sign (char 176), so draw it as a small ring
// right after the current print position, then print "C".
void printDegreesC()
{
        u8g_uint_t x = u8g.getPrintCol();
        u8g_uint_t y = u8g.getPrintRow();
        u8g.drawCircle(x + 4, y - 17, 3);
        u8g.drawCircle(x + 4, y - 17, 2);
        u8g.setPrintPos(x + 10, y);
        u8g.print('C');
}

void DrawGauges()
{
        int i;
        if (time > timeL) // Delay while new LOGO displayed
        {
                if (time > timeOLED) // Delay for DATA update
                {
                        u8g.firstPage();
                        timeOLED = time + OLED_DELAY;
                        do
                        {
                                u8g.setDefaultForegroundColor();
                                if (ALARM_STATUS && ALARM_BLINK) // INVERT SCREEN ON ALARM
                                {
                                        digitalWrite(ALARM_PIN, HIGH);
                                        u8g.drawBox(0, 0, 128, 64);
                                        u8g.setDefaultBackgroundColor();
                                }
                                else
                                        digitalWrite(ALARM_PIN, LOW);
                                u8g.setFont(u8g_font_fub20r); // ASCII-only font: ~5KB smaller than full fub20
                                u8g.setPrintPos(0, 23);
                                // Add extra info to OLED
                                switch (LOGO_STATUS)
                                {
                                case STATE_OIL:
                                        u8g.print(DISP_OIL_PRESSURE);
                                        u8g.print("bar");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(DISP_OIL_TEMP));
                                        printDegreesC();
                                        break;
                                case STATE_EXHAUST:
                                        u8g.print("AFR:");
                                        u8g.print(float(AFR));
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(DISP_EGT));
                                        printDegreesC();
                                        break;
                                case STATE_BRAKES:
                                        u8g.print("Brakes");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(int(DISP_BRAKES_TEMP));
                                        printDegreesC();
                                        break;
                                case STATE_VOLT:
                                        u8g.print("Battery");
                                        u8g.setPrintPos(0, 60);
                                        u8g.print(VOLTAGE);
                                        u8g.print("V");
                                        break;
                                }
                        } while (u8g.nextPage());
                        ALARM_BLINK = !ALARM_BLINK;
                }
        }
        // Process VFD scale position
        // Compute in signed int and clamp before storing into byte,
        // otherwise negative values wrap to ~250 and show the gauge as empty.
        int targetL = TARGETPOS_L;
        int targetR = TARGETPOS_R;
        switch (LOGO_STATUS)
        {
        case STATE_OIL:
                targetL = 20 - int(DISP_OIL_PRESSURE * 2); /* 0 -   5 - 10   */
                if (DISP_OIL_TEMP < 40)
                        targetR = 8; /* NONE SHOW */
                else if (DISP_OIL_TEMP < 80)
                        targetR = 7;
                else if (DISP_OIL_TEMP < 110)
                        targetR = 6; /* CENTER */
                else if (DISP_OIL_TEMP < 120)
                        targetR = 5;
                else if (DISP_OIL_TEMP < 130)
                        targetR = 4;
                else if (DISP_OIL_TEMP < 140)
                        targetR = 3;
                else if (DISP_OIL_TEMP < 150)
                        targetR = 2;
                else
                        targetR = 1; /* RED */
                break;
        case STATE_EXHAUST:
                targetL = 20 - int(AFR * 20); /* 0 - 0.5 - 1   */
                targetR = 7 - int(DISP_EGT / 100); /* 0 - 300 - 700 */
                break;
        case STATE_BRAKES:
                targetR = 7 - int(DISP_BRAKES_TEMP / 54); /* 0 - 130 - 300 */
                break;
        case STATE_VOLT:
                targetL = 20 - int(VOLTAGE - 8) * 2; /* 8 - 14  - 18  */
                break;
        }

        // Zero negative vaules and fix over values
        TARGETPOS_L = constrain(targetL, 1, 20);
        TARGETPOS_R = constrain(targetR, 1, 8);

        // Move gauges towards target positions, one segment per GAUGE_STEP_DELAY
        if (time - timeGauge >= GAUGE_STEP_DELAY)
        {
                timeGauge = time;
                if (POSITION_L < TARGETPOS_L)
                        POSITION_L++;
                if (POSITION_L > TARGETPOS_L)
                        POSITION_L--;
                if (POSITION_R < TARGETPOS_R)
                        POSITION_R++;
                if (POSITION_R > TARGETPOS_R)
                        POSITION_R--;
        }

        // INIT VFD display
        digitalWrite(SI_PIN, LOW);
        digitalWrite(SCK_PIN, HIGH);
        digitalWrite(LH_PIN, LOW);

        // Walk over gauge indicator positions
        for (i = 0; i <= 43; i++)
        {
                digitalWrite(SCK_PIN, LOW);
                if ((DRAW_L && (i == 26 || i == 27 || i == 28)) ||                    // Draw left gauge
                    (DRAW_R && (i == 38 || i == 41)) ||                               // Draw Right gauge
                    (DRAW_R && DRAW_RL && (i == 39 || i == 40 || i == 42 || i == 43)) // Draw Right gauge letters
                )
                        digitalWrite(SI_PIN, HIGH);
                if ((i >= 5 && i <= 25) && DRAW_L)
                { // LEFT 20
                        if ((TYPE_L == BAR) && (POSITION_L <= (i - 5)))
                                digitalWrite(SI_PIN, HIGH);
                        if ((TYPE_L == NEEDLE) && (POSITION_L == (i - 5)))
                                digitalWrite(SI_PIN, HIGH);
                }

                if ((i >= 31 && i <= 37) && DRAW_R)
                { // RIGHT 6
                        if ((TYPE_R == BAR) && (POSITION_R <= (i - 31)))
                                digitalWrite(SI_PIN, HIGH);
                        if ((TYPE_R == NEEDLE) && (POSITION_R == (i - 31)))
                                digitalWrite(SI_PIN, HIGH);
                }
                digitalWrite(SCK_PIN, HIGH);
                digitalWrite(SI_PIN, LOW);
        }

        // LATCH AND HOLD
        digitalWrite(LH_PIN, HIGH);
        digitalWrite(LH_PIN, LOW);
}

void ReadSensors()
{
        float vout;

        readFilteredAnalog(OIL_TEMP_SENSOR, val_T);

        OIL_TEMP = R3 * (1024.0f / val_T - 1.0f);
        OIL_TEMP = OIL_TEMP / THERMISTOR_NOMINAL;          // (R/Ro)
        OIL_TEMP = log(OIL_TEMP);                          // ln(R/Ro)
        OIL_TEMP /= B_COEFFICIENT;                         // 1/B * ln(R/Ro)
        OIL_TEMP += 1.0f / (TEMPERATURE_NOMINAL + 273.15f); // + (1/To)
        OIL_TEMP = 1.0f / OIL_TEMP;                       // Invert
        OIL_TEMP -= 273.15f;
        OIL_TEMP = clampf(OIL_TEMP, 0.0f, 200.0f);

        readFilteredAnalog(OIL_PRESSURE_SENSOR, val_P);

        float oilPressureRatio = (val_P - MIN_VOLT_PRESS) / (MAX_VOLT_PRESS - MIN_VOLT_PRESS);
        OIL_PRESSURE = clampf(oilPressureRatio, 0.0f, 1.0f) * OIL_SENSOR_TYPE;

        readFilteredAnalog(VOLTMETER_SENSOR, val_V);
        vout = (val_V * 5.0f) / 1023.0f;
        VOLTAGE = vout / (R2 / (R1 + R2));

        // AirFuelRatio
        //ADS1115: +2.048V/0.0625mV
        AFR = (ads.readADC_SingleEnded(AFR_INPUT) * 0.0625f) / 1000.0f;

        // Exhaust Gas Temtrature
        // MAX6675 needs ~220ms per conversion, so poll it at a lower rate.
        // readCelsius() returns NAN when the thermocouple is open; valid range is 0..1023.75C.
        if (millis() >= timeEGT)
        {
                timeEGT = millis() + EGT_DELAY;
                EGT = thermocouple.readCelsius();
                if (isnan(EGT) || (EGT < 0))
                        EGT = 0;
        }

        // Brakes temperature
        BRAKES_TEMP = mlx.readObjectTempC();
        if (BRAKES_TEMP > 380)
                BRAKES_TEMP = 0;
        // Brakes ambient temperature for datalog only
        //val = mlx.readAmbientTempC();

        // ALARM SYSTEM
        // alrms order from lower to  upper priority
        ALARM_STATUS = false;

        // EGT to High
        if (EGT > ALARM_EGT)
        {
                LOGO_STATUS = STATE_EXHAUST;
                ALARM_STATUS = true;
        }

        // Brakes temprature to high
        if (BRAKES_TEMP > ALARM_BRAKES)
        {
                LOGO_STATUS = STATE_BRAKES;
                ALARM_STATUS = true;
        }

        // Overcharge / No charge (while engine running)
        if ((VOLTAGE > ALARM_BATTERY_HIGH) ||
            ((VOLTAGE < ALARM_BATTERY_LOW) && (OIL_PRESSURE > 1.0f)))
        {
                LOGO_STATUS = STATE_VOLT;
                if (ALARM_TIME == 0)
                        ALARM_TIME = millis();
                if (millis() - ALARM_TIME >= ALARM_BETTERY_DELAY)
                        ALARM_STATUS = true;
        }
        else
                ALARM_TIME = 0; // voltage is back to normal - restart the delay

        // High OIL temperature
        if (OIL_TEMP > ALARM_TEMP)
        {
                LOGO_STATUS = STATE_OIL;
                ALARM_STATUS = true;
        }

        // Low OIL pressure
        if (((OIL_PRESSURE < ALARM_OIL) || (OIL_PRESSURE > ALARM_OIL_HIGH)) && (VOLTAGE > ALARM_BATTERY_LOW) && OIL_TEMP > WORK_TEMP)
        { // while engine runnin
                LOGO_STATUS = STATE_OIL;
                ALARM_STATUS = true;
        }

        if (ALARM_STATUS)
        {
                SHOW_LOGO = false;
                LogoSetup();
        }
        else
                digitalWrite(ALARM_PIN, LOW);

        // Data logging
        time = millis();
#ifdef DATALOG_ENABLE
        if (time >= timeP)
        {
                Serial.print(time);
                Serial.print(", ");
                Serial.print(OIL_TEMP);
                Serial.print(", ");
                Serial.print(OIL_PRESSURE);
                Serial.print(", ");
                Serial.print(VOLTAGE);
                Serial.print(", ");
                Serial.print(AFR);
                Serial.print(", ");
                Serial.print(EGT);
                Serial.print(", ");
                Serial.print(BRAKES_TEMP);
                Serial.print(", ");
                Serial.print("\n");
                timeP = time + LOG_DELAY;
        }
#endif
}

/*
    MAIN GAUGES LOOP
 */
void loop()
{

        // Read all sensors and dataloging
        ReadSensors();

        // Dimmer signal check
        DIMMER_STATE = digitalRead(DIMMER_PIN);
        if (DIMMER_STATE != DIMMER_PREVSTATE)
        {
                if (DIMMER_STATE == HIGH)
                        u8g.setContrast(25);
                else
                        u8g.setContrast(255);
                DIMMER_PREVSTATE = DIMMER_STATE;
        }

        // Click button operation
        BUTTON_STATE = digitalRead(BUTTON_PIN);
        if (BUTTON_STATE == LOW)
        {
                if (BUTTON_PREVSTATE != BUTTON_STATE) /* JUST PRESSED */
                {
                        LOGO_STATUS++;
                        if (LOGO_STATUS > MAX_LOGO)
                                LOGO_STATUS = 1;
                        LogoSetup();
                        SHOW_LOGO = true;
                }
        }
        BUTTON_PREVSTATE = BUTTON_STATE;

        // Change logo at OLED display
        if (SHOW_LOGO)
        {
                u8g.firstPage();
                do
                {
                        switch (LOGO_STATUS)
                        {
                        case STATE_OIL:
                                DRAW_LOGO(oil_LOGO);
                                break;
                        case STATE_EXHAUST:
                                DRAW_LOGO(exhaust_LOGO);
                                break;
                        case STATE_BRAKES:
                                DRAW_LOGO(brakes_LOGO);
                                break;
                        case STATE_VOLT:
                                DRAW_LOGO(battery_LOGO);
                                break;
                        default:
                                break;
                        }
                } while (u8g.nextPage());
                SHOW_LOGO = false;
                timeL = time + LOGO_DELAY;
        }

        // Smooth temperatures and pressure for display
        UpdateDisplayValues();

        // Draw VFD gauges
        DrawGauges();
}
