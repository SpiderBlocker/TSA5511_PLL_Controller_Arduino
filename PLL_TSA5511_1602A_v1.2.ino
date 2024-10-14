/*
DESCRIPTION
  PLL controller for the TSA5511, intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop.
  Using a 3,2 MHz crystal on the TSA5511, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof.
  The practical lower and upper limits will be much tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz.
  Using any other crystal frequency (as far as the TSA5511 may support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.
  It has a built-in station name editor and the station name and last set frequency are stored in EEPROM.

HARDWARE
  • The hardware comprises of an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlighting and contrast adjustment, three
    pushbuttons (DOWN/SET/UP, each with a 100 nF debouncing capacitor across its contact) and an optional PLL lock LED with adequate series resistor. The lock status
    is also shown on the LCD display.
  • LCD backlighting control is availale if you connect it to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital
    pins used for both the LCD backlighting and the PLL lock LED must support PWM. Refer to code to change the brightness levels for both the LCD backlighting and
    PLL lock LED as you wish.
  • Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF-decoupling circuitry, you may want to use lower values for reliable communication,
    like 1 or 2 kΩ.
  • If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as it has been proven that slight voltage fluctuations
    on the TSA5511 will cause a few ppm XTAL frequency deviation accordingly.

USE
  • Verify the actual XTAL frequency and required band edge frequencies under "// PLL settings" and "// VCO frequency settings" below and change if necessary.
  • The TSA5511 charge pump is kept high at all times for the DRFS06 exciter. For other platforms, in function "checkPll()" set "data[0] = PLL_CP_LOW" if required.
  • Change frequency using UP/DOWN-buttons and confirm with SET button. The new frequency will be stored in EEPROM. Changing frequency without confirmation will
    timeout and return to the main screen unchanged. Holding UP/DOWN will auto-scroll through the frequency band with gradual acceleration.
  • Press and hold SET button during startup to enable the station name editor. Select characters using UP/DOWN-buttons and confirm with SET button. The new station
    name will be stored in EEPROM after the last character has been confirmed and the main screen will be displayed.
  • The LCD backlight will dim after a preset time. Press and hold the SET button during normal operation to turn it off completely. The LCD backlight will turn on
    again by pressing any button.
  • In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET button to restart. I²C communication will be retried
    several times before alerting an error.
*/

// version & credits
#define description "PLL Control"
#define version "V1.2"
#define credits "(C)2024 Loenie"

// required libraries 
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// buttons pin mapping
const int downButton = 2;
const int setButton = 3;
const int upButton = 4;

// lock LED pin mapping and brightness
const int pllLockOutput = 5;
const int pllLockBrightness = 20; // lock LED brightness 

// LCD display pin mapping
const int lcdBacklightOutput = 6; // LCD backlight LED anode
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS, E, D4, D5, D6, D7

// LCD brightness and dimmer settings
const long backlightOffDelay = 1500; // LCD backlight turn off delay after holding SET
const long dimDelay = 10000; // LCD brightness dimmer delay
const int dimStepDelay = 5; // LCD gradual brightness dimming speed
const int lcdBackgroundBrightness = 255; // LCD active brightness
const int dimmedBrightness = 50; // LCD quiescent brightness

// EEPROM storage
const int EEPROM_FREQ_ADDR = 0;
const int EEPROM_NAME_ADDR = 10;

// I²C settings
const long i2cClock = 32000; // low I²C clock frequency, more robust through SDA/SCL RF-decoupling circuitry (min. 31,25 kHz for 16 MHz ATmega328P)
const long wireTimeout = 5000; // I²C transmission timeout, avoiding I²C bus crash in some cases
const int maxRetries = 10; // maximum number of retries in case of a failed I²C transmission

// PLL settings
const byte PLL_ADDR = 0x30; // PLL 7-bit I²C address (left-aligned in 8-bit format)
const byte PLL_ADDR_WRITE = (PLL_ADDR << 1); // 8-bit write address
const byte PLL_ADDR_READ = ((PLL_ADDR << 1) | 1); // 8-bit read address
const byte PLL_CP_LOW = 0x8E; // charge pump low
const byte PLL_CP_HIGH = 0xCE; // charge pump high
const byte PLL_ALL_LOW = 0x00; // all outputs (P0-P7) low
const byte PLL_P2_HIGH = 0x04; // P2 high
const byte PLL_P2_P5_HIGH = 0x24; // P2/P5 high
const long PLL_XTAL_FREQ = 3200000; // PLL crystal frequency (Hz)
const int PLL_XTAL_DIVISOR = 512; // PLL crystal divisor
const int PLL_PRESCALER_DIVISOR = 8; // PLL prescaler divisor
const long PLL_REF_FREQ = (PLL_XTAL_FREQ / PLL_XTAL_DIVISOR) * PLL_PRESCALER_DIVISOR; // PLL reference frequency (Hz), also equals the minimum possible VCO frequency and step size
const int PLL_LOCK_BIT = 6; // PLL lock flag bit

// VCO frequency settings
float lowerBandEdge = 80000000; // lower band edge frequency (Hz)
float upperBandEdge = 108000000; // upper band edge frequency (Hz)
long freqStep = PLL_REF_FREQ * 1; // frequency step size (Hz), must be equal to or an exact multiple of PLL_REF_FREQ

// VCO frequency validation
float validatedLowerBandEdge = (lowerBandEdge < upperBandEdge) ? lowerBandEdge : upperBandEdge, // swap lowerBandEdge and upperBandEdge if necessary
      validatedUpperBandEdge = (lowerBandEdge > upperBandEdge) ? lowerBandEdge : upperBandEdge;
long validateFreq(float frequency) {
    if (frequency < PLL_REF_FREQ) { frequency = PLL_REF_FREQ; } // ensure that minimum frequency is not lower than PLL_REF_FREQ
    if (frequency / PLL_REF_FREQ > 0x7FFF) { frequency = 0x7FFF * PLL_REF_FREQ; } // ensure that PLL divisor does not exceed 15 bits, as 1st bit of first PLL divisor byte must be 0
    return round((float)frequency / PLL_REF_FREQ) * PLL_REF_FREQ; // ensure that frequency equals or is an exact multiple of PLL_REF_FREQ
}
const long lowerFreq = validateFreq(validatedLowerBandEdge); // set valid lower band edge frequency
const long upperFreq = validateFreq(validatedUpperBandEdge); // set valid upper band edge frequency

// station name settings
const int maxNameLength = 16;
const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator
char stationName[maxNameLength + 1]; // +1 for null terminator

// general definitions/declarations
const long startupDelay = 2500; // time to show startup message
const long initialPressDelay = 1000; // delay before continuous change when holding button
const long initialPressInterval = 80; // continuous change interval when holding button
const long charScrollInterval = 300; // display character scrolling interval
const long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
long freq;
long currentFreq;
long lastButtonPressTime = 0;
int nameEditPos;
bool initialized = false;
bool nameEditMode = false;
bool freqSetMode = false;
bool buttonDownPressed = false;
bool buttonSetPressed = false;
bool buttonUpPressed = false;
bool pllLock = false;
bool pllWatchdog = false;

// display modi
enum {
    STARTUP = 0,
    STATION_NAME_EDITOR,
    MAIN_INTERFACE,
    SET_FREQUENCY_INTERFACE,
    PLL_LOCK_STATUS,
    I2C_ERROR
};

void setup() {
    pinMode(downButton, INPUT_PULLUP);
    pinMode(setButton, INPUT_PULLUP);
    pinMode(upButton, INPUT_PULLUP);
    pinMode(pllLockOutput, OUTPUT);

    Wire.begin();
    Wire.setClock(i2cClock);
    Wire.setWireTimeout(wireTimeout, true);
    lcd.begin(16, 2);

    display(STARTUP);
    analogWrite(lcdBacklightOutput, lcdBackgroundBrightness);
    delay(startupDelay);
    readStationName();
    readFrequency();

    if (!digitalRead(setButton)) { // enable station name editor when holding SET during startup
        display(STATION_NAME_EDITOR);
        while(!digitalRead(setButton)); // lock cursor position until SET release
        lastButtonPressTime = millis(); // reset LCD backlight dimmer timer
        nameEditMode = true;
    } else { // complete initialization
        configurePll();
        display(MAIN_INTERFACE);
        initialized = true;
    }
}

void loop() {
    bool buttonDownState = !digitalRead(downButton);
    bool buttonSetState = !digitalRead(setButton);
    bool buttonUpState = !digitalRead(upButton);

    handleDisplayBrightness(buttonDownState, buttonSetState, buttonUpState);
    handleNameEditor(buttonDownState, buttonSetState, buttonUpState);
    handleFrequencyChange(buttonDownState, buttonSetState, buttonUpState);
    checkPll();
}

void handleButtonPress(bool buttonState, bool& buttonPressed, int direction, void (*action)(int)) {
    static unsigned long pressStartTime = 0, lastPressTime = 0;
    unsigned long totalPressTime = millis() - pressStartTime, fastPressInterval = initialPressInterval;

    // no change if DOWN and UP are pressed simultaneously
    if (!digitalRead(downButton) && !digitalRead(upButton)) { return; }

    if (buttonState) {
        // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed) { pressStartTime = millis(); }
        if (totalPressTime >= initialPressDelay) {
            if (!nameEditMode) {
                // gradual acceleration
                long postDelayTime = totalPressTime - initialPressDelay;
                fastPressInterval = max(initialPressInterval / (0.7 + (postDelayTime / initialPressDelay)), initialPressInterval / 7);
            }
        }
        if (!buttonPressed || (totalPressTime >= initialPressDelay && millis() - lastPressTime >= fastPressInterval)) {
            lastPressTime = millis();
            buttonPressed = true;
            action(direction);
        }
    } else {
        buttonPressed = false;
    }
}

void handleDisplayBrightness(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    static unsigned long lastDimUpdateTime = 0;
    static unsigned long buttonHoldStartTime = 0;
    static int currentBrightness = 255;
    static bool backlightOff = false;
    static bool disableBrightnessRestore = false;

    // turn off background lighting by pressing and holding SET button
    if (buttonSetState && !nameEditMode) {
        if (buttonHoldStartTime == 0) {
            buttonHoldStartTime = millis();
        } else if (millis() - buttonHoldStartTime > backlightOffDelay && !backlightOff) {
            analogWrite(lcdBacklightOutput, 0);
            backlightOff = true;
            disableBrightnessRestore = true;
        }
    } else {
        buttonHoldStartTime = 0;
        disableBrightnessRestore = false;
    }
    // restore brightness when pressing any button
    if ((buttonDownState || buttonSetState || buttonUpState) && !disableBrightnessRestore) {
        currentBrightness = lcdBackgroundBrightness;
        analogWrite(lcdBacklightOutput, currentBrightness);
        if (backlightOff) { while(!digitalRead(setButton)); }
        lastButtonPressTime = millis();
        backlightOff = false;
    }
    // gradual dimming after timeout
    if ((millis() - lastButtonPressTime > dimDelay) && !backlightOff) {
        if (millis() - lastDimUpdateTime >= dimStepDelay && currentBrightness > dimmedBrightness) {
            currentBrightness--;
            analogWrite(lcdBacklightOutput, currentBrightness);
            lastDimUpdateTime = millis();
        }
    }
}

void handleNameEditor(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    if (nameEditMode) {
        // select character
        auto nameChange = [](int direction) { nameEditorAction(0, direction); };
        handleButtonPress(buttonDownState, buttonDownPressed, -1, nameChange);
        handleButtonPress(buttonUpState, buttonUpPressed, 1, nameChange);
        // confirm selection
        handleButtonPress(buttonSetState, buttonSetPressed, 0, [](int direction) { nameEditorAction(1, direction); });
    } else {
        // complete initialization when returning from station name editor
        if (!initialized) {
            configurePll();
            display(MAIN_INTERFACE);
            initialized = true;
            checkPll();
            while(!digitalRead(setButton));
            lastButtonPressTime = millis(); // reset LCD backlight dimmer timer
        }
    }
}

void nameEditorAction(int action, int direction) {
    if (action == 0) {
        // UP/DOWN action
        int charRange = 127 - 32 + 1; // allowed ASCII character range
        stationName[nameEditPos] = (stationName[nameEditPos] - 32 + direction + charRange) % charRange + 32;
        display(STATION_NAME_EDITOR);
    } else {
        // SET action
        if (nameEditPos >= maxNameLength - 1) { // store station name when last character has been confirmed
            storeStationName();
            nameEditMode = false;
        } else {
            nameEditPos++; // move to next cursor position
            display(STATION_NAME_EDITOR);
            delay(charScrollInterval);
        }
    }
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName);
    stationName[maxNameLength] = '\0'; // null terminator

    // set standard station name if string is invalid
    size_t length = strnlen(stationName, maxNameLength);
    for (size_t i = 0; i < length; i++) {
        if (stationName[i] < 32 || stationName[i] > 127 || stationName[i] == 0xFF) { // no invalid ASCII-character, no 0xFF
            strncpy(stationName, defaultName, maxNameLength); // copy default station name to array
            memset(stationName + strlen(defaultName), 32, maxNameLength - strlen(defaultName)); // fill remaining positions with spaces (ASCII 32)
            break;
        }
    }
}

void storeStationName() {
    char storedStationName[maxNameLength + 1];
    EEPROM.get(EEPROM_NAME_ADDR, storedStationName);

    // avoid unnecessary write operations to protect EEPROM
    if (strncmp(stationName, storedStationName, maxNameLength) != 0) {
        EEPROM.put(EEPROM_NAME_ADDR, stationName);
    }
}

void handleFrequencyChange(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    static unsigned long lastButtonPressTime = 0;
    static bool timedOut = false;

    if (initialized && !nameEditMode) {
        // change frequency
        auto freqChange = [](int direction) { frequencyChangeAction(0, &freq, direction); };
        handleButtonPress(buttonDownState, buttonDownPressed, -1, freqChange);
        handleButtonPress(buttonUpState, buttonUpPressed, 1, freqChange);
        if (buttonDownState || buttonUpState) {
            lastButtonPressTime = millis();
            timedOut = false;
        }
        // confirm frequency
        if (freqSetMode && buttonSetState) {
            frequencyChangeAction(1, &freq, 0);
        } else if (millis() - lastButtonPressTime > freqSetTimeout) { // inactivity timeout
            freqSetMode = false;
            if (!timedOut) { // restore initial status
                freq = currentFreq;
                display(MAIN_INTERFACE);
                timedOut = true;
            }
        }
    }
}

void frequencyChangeAction(int action, long* newFreq, int direction) {
    if (action == 0) {
        // UP/DOWN action
        if (freqSetMode) { *newFreq += (direction * freqStep); }
        *newFreq = (*newFreq < lowerFreq) ? upperFreq : (*newFreq > upperFreq) ? lowerFreq : *newFreq;
        freqSetMode = true;
        display(SET_FREQUENCY_INTERFACE);
    } else {
        // SET action
        configurePll();
        freqSetMode = false;
        display(MAIN_INTERFACE);
    }
}

void readFrequency() {
    if (!freqSetMode) {
        // get last stored frequency from EEPROM
        long storedFreq;
        EEPROM.get(EEPROM_FREQ_ADDR, storedFreq);

        // check if storedFreq lies within the valid range (lowerFreq to upperFreq)
        if (storedFreq < lowerFreq || storedFreq > upperFreq) {
            freq = lowerFreq; // default initial frequency
        } else {
            freq = round(storedFreq / PLL_REF_FREQ) * PLL_REF_FREQ; // round to closest multiple of PLL_REF_FREQ
        }
    }
}

void storeFrequency(long frequency) {
    // avoid unnecessary write operations during startup to protect EEPROM
    if (initialized) { EEPROM.put(EEPROM_FREQ_ADDR, frequency); }
}

void configurePll() {
    if (freq != currentFreq) {
        long divisor = (freq / PLL_REF_FREQ); // calculate divisor
        byte data[4];
        data[0] = (divisor & 0xFF00) >> 8; // extract high divisor byte
        data[1] = divisor & 0x00FF; // extract low divisor byte
        data[2] = PLL_CP_HIGH; // set charge pump
        data[3] = PLL_ALL_LOW; // set output ports
        for (int i = maxRetries; i > 0; i--) {
            Wire.beginTransmission(PLL_ADDR_WRITE);
            Wire.write(data, 4);
            if (Wire.endTransmission() == 0) {
                storeFrequency(freq);
                currentFreq = freq;
                pllWatchdog = true;
                break;
            }
            delay(50);
            if (i == 1) { i2cErrHandler(); }
        }
    }
}

void checkPll() {
    if (pllWatchdog && !nameEditMode) {
        for (int i = maxRetries; i > 0; i--) {
            Wire.requestFrom(PLL_ADDR_READ, (byte)1);
            if (Wire.available()) {
                byte readByte = Wire.read();
                pllLock = (readByte >> PLL_LOCK_BIT) & 0x01;
                display(PLL_LOCK_STATUS);
                break;
            }
            delay(50);
            if (i == 1) { i2cErrHandler(); }
        }
        if (pllLock) {
            byte data[2]; // partial programming, starting with byte 4
            data[0] = PLL_CP_HIGH; // set charge pump
            data[1] = PLL_P2_P5_HIGH; // set output ports
            for (int i = maxRetries; i > 0; i--) {
                Wire.beginTransmission(PLL_ADDR_WRITE);
                Wire.write(data, 2);
                if (Wire.endTransmission() == 0) {
                    pllWatchdog = false; // PLL watchdog not used after lock, as PLL lock flag may fluctuate due to FM modulation
                    break;
                }
                delay(50);
                if (i == 1) { i2cErrHandler(); }
            }
        }
    }
}

void i2cErrHandler() {
    display(I2C_ERROR);
    while (!digitalRead(setButton)); // ensure setButton is released, as it may trigger a premature reset
    while (true) {
        if (!digitalRead(setButton)) {
            delay(30); // alleviate processor loading
            if (!digitalRead(setButton)) { asm volatile ("jmp 0"); } // soft reset
        }
    }
}

void display(int mode) {
    // show and right-align frequency on display
    auto printFreq = []() {
        lcd.print(freq < 10000000 ? "   " : (freq < 100000000 ? "  " : (freq < 1000000000 ? " " : "")));
        lcd.print(freq / 1000000.0);
        lcd.print(" MHz");
    };

    switch(mode){
        case STARTUP:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(description);
            lcd.print(" ");
            lcd.print(version);
            lcd.setCursor(0, 1);
            lcd.print(credits);
            break;

        case STATION_NAME_EDITOR:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET Station Name");
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            lcd.setCursor(nameEditPos, 1);
            lcd.cursor();
            break;

        case MAIN_INTERFACE:
            lcd.noCursor(); // required when returning from case STATION_NAME_EDITOR
            lcd.setCursor(5, 0);
            printFreq();
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            break;

        case SET_FREQUENCY_INTERFACE:
            lcd.setCursor(0, 1);
            lcd.print("SET  ");
            lcd.setCursor(5, 1);
            printFreq();
            break;

        case PLL_LOCK_STATUS:
            lcd.setCursor(0, 0);
            if (pllLock){
                lcd.print("LOCK  ");
                analogWrite(pllLockOutput, pllLockBrightness);
            } else {
                // animation during lock wait
                static unsigned long lastCharScrollTime = 0;
                static int charPos = 0;
                static bool movingRight = true;
                analogWrite(pllLockOutput, 0);
                if (millis() - lastCharScrollTime >= charScrollInterval) {
                    lastCharScrollTime = millis();
                    lcd.print("     ");
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? ">>" : "<<");
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 3 || charPos == 0) { movingRight = !movingRight; }
                }
            }
            break;
        
        case I2C_ERROR:
            lcd.noCursor(); // required when returning from case STATION_NAME_EDITOR
            lcd.clear();
            analogWrite(pllLockOutput, 0);
            lcd.setCursor(0, 0);
            lcd.print("I2C ERROR");
            lcd.setCursor(0, 1);
            lcd.print("SET to restart");
            break;
    }
}
