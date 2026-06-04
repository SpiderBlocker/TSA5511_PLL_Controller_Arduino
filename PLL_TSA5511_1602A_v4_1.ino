/*
DESCRIPTION
  PLL controller for the TSA5511, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop, but it can be used for any other
  TSA5511 based exciter, operating in a VCO frequency range of 64 MHz up to 1,300 MHz as per specification of the TSA5511.
  It features an intuitive menu interface for configuring system settings and a quick menu for frequently used functions, as described in detail below.
  Persistent system settings and user memories are stored in EEPROM and automatically recalled upon restart.

HARDWARE
  • The hardware comprises an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlight and contrast adjustment, three pushbuttons
    (DOWN/SET/UP, each with a 470 nF debouncing capacitor across its contact) and an optional PLL lock LED which also acts as a blinking fault indicator.
    The lock status is also shown on the LCD display.
  • LCD backlight control is available if connected to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital
    pin used for the LCD backlight must support PWM. Currently pin 6 is configured, which is valid on common Arduino boards.
  • The current EEPROM layout requires at least 1 kB EEPROM, as provided by ATmega328P-based Arduino Nano boards.
  • Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF-decoupling circuitry, you may want to use lower values for reliable communication,
    such as 1 or 2 kΩ.
  • If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as slight voltage fluctuations on the TSA5511 supply rail may cause
    a few ppm XTAL frequency deviation.

USAGE
  • Double-clicking SET opens the SYSTEM SETTINGS menu. In system submenus, holding SET returns one level back while keeping pending changes in the temporary menu state
    until they are explicitly saved or discarded. The only exception is the station name editor, where SET is used to confirm characters and can be held to auto-advance
    through character positions. The available system settings are as follows:

    ■ VCO SETTINGS     => • FREQUENCY BAND   > Various predefined frequency bands are available for selection. The last operating frequency will be stored in EEPROM for
                                               each VCO frequency band and XTAL frequency separately. The last selected VCO frequency band will be stored in EEPROM for
                                               each XTAL frequency separately as well.
                          • PRECISION        > This sets the decimal precision at which the VCO frequency can be set and will be displayed. Note that if it is set to a
                                               lower precision than required for the current VCO frequency, confirmation will result in the new VCO frequency being
                                               rounded and set to the nearest possible value. Since the minimum VCO frequency step size is derived from the PLL crystal
                                               frequency and the /8 prescaler (25 kHz @ 1.6 MHz and 50 kHz @ 3.2 MHz), the actual frequency precision will default to the
                                               highest possible resolution automatically, i.e. 3 decimals at 1.6 MHz and 2 decimals at 3.2 MHz respectively. This can be
                                               changed to a lower value if so desired. Refer to additional explanation below at PLL SETTINGS > XTAL FREQUENCY.
                          • RETURN           > Returns to the main settings menu.

    ■ PLL SETTINGS     => • I2C ADDRESS      > This allows selecting the appropriate I²C address based on the actual hardware configuration of the TSA5511.
                                               By DC-biasing pin P3 of the TSA5511, the I²C address can be configured to 0x60, 0x62, or 0x63, while 0x61 is always valid
                                               regardless of the hardware configuration. By default the I²C address is set to 0x61.
                                               Upon confirming a new I²C address, communication is automatically verified. If verification fails, the last known working
                                               I²C address will be restored automatically. In the unlikely event that an incompatible I²C address is stored and can't be
                                               reconfigured through the menu, restart the system while holding SET to restore the default fail-safe I²C address (0x61).
                          • XTAL FREQUENCY   > This setting must match the actual PLL crystal frequency. The default PLL crystal frequency is 3.2 MHz, resulting in a
                                               theoretical upper VCO frequency of 1,638.35 MHz. If a PLL crystal frequency of 1.6 MHz is used, the theoretical upper VCO
                                               frequency will be 819.175 MHz, in which case any upper band limit exceeding this maximum value will be automatically
                                               adjusted accordingly.
                                               Note that compatibility of the TSA5511 with a 1.6 MHz crystal frequency is not officially supported; however, it has been
                                               empirically confirmed to work.
                          • CHARGE PUMP      > This sets the PLL charge-pump current in locked state: high (220 µA), low (50 µA), or disabled. It should be set to high
                                               for the DRFS06 exciter or to low for other platforms if required. Set it to disabled for testing purposes.
                          • PORT MAPPING     > This setting maps corresponding output ports on the TSA5511 to drive an external lock indicator, an external unlock
                                               indicator and the transmitter RF output stage respectively.
                          • RETURN           > Returns to the main settings menu.

    ■ GENERAL SETTINGS => • STATION NAME     > This sets the radio station name that is shown in quiescent condition (locked state). Select characters using UP/DOWN and
                                               confirm each character with SET. Hold UP/DOWN to auto-scroll characters; hold SET to auto-advance through character positions.
                          • BACKLIGHT DIMMER > This toggles the automatic LCD backlight dimmer function (on or off).
                          • SHOW MENU TITLE  > This toggles the animated title screen when entering SYSTEM SETTINGS or QUICK MENU.
                          • FACTORY RESET    > This clears all stored settings and user memories and restores the default settings after double confirmation.
                          • RETURN           > Returns to the main settings menu.

    ■ EXIT SETTINGS    => • save changes     > Stores any changes to EEPROM and returns to the main interface.
                          • discard          > Discards any changes and returns to the main interface.
                          • cancel           > Returns to the first index of the main menu.

  • Press and hold SET from the main interface to open the QUICK MENU, which provides access to frequently used end-user functions. Within the QUICK MENU, holding SET
    returns one level back, or exits the menu from its top level. The QUICK MENU provides the following functions:

    ■ QUICK MENU       => • RECALL MEMORY    > Recalls one of six user-stored VCO frequencies for the current VCO frequency band and XTAL frequency.
                          • SAVE MEMORY      > Saves the current VCO frequency in one of six user memory slots for the current VCO frequency band and XTAL frequency.
                          • CLEAR MEMORY     > Clears one of the user memory slots for the current VCO frequency band and XTAL frequency.
                          • RF DRIVE         > Temporarily enables or disables the RF drive output without storing the state in EEPROM.
                                               When off, the station name alternates with an RF DRIVE: OFF status message.
                          • LCD OFF          > Turns off the LCD backlight until any button is pressed.
                          • EXIT QUICK MENU  > Returns to the main interface.

  • The SYSTEM SETTINGS menu will timeout after a preset period of inactivity, discarding any unsaved changes and returning to the main screen — except when the
    save/discard/cancel exit menu is active, which requires explicit user confirmation.
  • The QUICK MENU will also timeout after a preset period of inactivity and return to the main screen; its actions are applied immediately.
  • Change VCO frequency using UP/DOWN and confirm with SET. Holding UP/DOWN will auto-sweep through the VCO frequency band with gradual acceleration. Changing the
    VCO frequency without confirmation will time out and return to the main screen unchanged.
  • If enabled, the LCD backlight will dim after a preset period in quiescent state (locked). The LCD backlight can be turned off completely from the QUICK MENU
    and will be restored by pressing any button.
  • In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times
    before alerting an error.
*/

// === LIBRARIES ===
    #include <avr/wdt.h>
    #include <EEPROM.h>
    #include <Wire.h>
    #include <LiquidCrystal.h>


// === AVR STARTUP: DISABLE WATCHDOG ===
    #if defined(__AVR__)
    __attribute__((naked, section(".init3")))
    void disableWatchdogEarly() {
        MCUSR = 0; // clear all reset flags
        wdt_disable(); // disable watchdog to prevent reset loop
    }
    #endif


// === SYSTEM CONSTANTS ===
    // version & metadata
    #define description "PLL Control"
    #define version "V4.1"
    #define credits "(C)2026 Loenie"

    // buttons pin mapping
    const uint8_t downButton = 2; // DOWN button to ground
    const uint8_t setButton = 3; // SET button to ground
    const uint8_t upButton = 4; // UP button to ground

    // indicators pin mapping (currently sharing same LED for both indicators)
    const uint8_t lockIndicator = 5; // lock indicator LED anode
    const uint8_t errIndicator = 5; // error indicator LED anode

    // LCD display settings and pin mapping
    const uint8_t lcdColumns = 16; // LCD column count
    const uint8_t lcdRows = 2; // LCD row count
    const uint8_t freqDisplayWidth = 8; // maximum width of formatted VCO frequency values on the LCD
    const uint8_t lockStatusWidth = 5; // fixed LCD column width reserved for PLL lock status
    const uint8_t lockStatusAnimWidth = 2; // character width of the unlocked status animation
    const uint8_t lcdBacklight = 6; // LCD backlight LED anode
    LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS, E, D4, D5, D6, D7

    // LCD brightness and dimmer settings
    const unsigned long dimDelay = 2500; // brightness dimmer delay
    const uint8_t dimStepDelay = 7; // gradual brightness dimming speed
    const uint8_t maxBrightness = 255; // maximum brightness
    const uint8_t lowBrightness = 30; // dimmed brightness
    const bool defaultBacklightDimActive = true; // default LCD backlight dimmer state

    // menu entry title settings
    const uint8_t menuEntryGapWidth = 3; // blank columns around the shrinking menu title before the target menu item appears
    const bool defaultShowMenuTitle = true; // default menu entry title display state

    // EEPROM storage settings
    const uint16_t EEPROM_NAME_ADDR = 10; // station name                                  17 bytes (16 characters + null terminator)
    const uint16_t EEPROM_DIM_ADDR = 30; // backlight dimmer setting                        1 byte
    const uint16_t EEPROM_SHOW_MENU_TITLE_ADDR = 35; // menu entry title display setting    1 byte
    const uint16_t EEPROM_DECIMAL_ADDR = 40; // VCO frequency decimal precision             1 byte
    const uint16_t EEPROM_CP_ADDR = 50; // charge pump setting                              1 byte
    const uint16_t EEPROM_FREQBAND_ADDR = 60; // actual VCO frequency band index            2 bytes (1 byte x 2 XTAL)
    const uint16_t EEPROM_XTAL_ADDR = 70; // 0 = 1.6 MHz, 1 = 3.2 MHz                       1 byte
    const uint16_t EEPROM_I2C_ADDR = 80; // I²C address setting                             1 byte
    const uint16_t EEPROM_OUTPUT_PORTS_ADDR = 86; // PLL output port mapping                3 bytes ([86]=locked, [87]=unlocked, [88]=RF drive)
    const uint16_t EEPROM_BAND_FREQ_BASE_ADDR = 90; // VCO frequency per band, per XTAL    56 bytes (4 bytes x 7 bands x 2 XTAL)
    const uint16_t EEPROM_USER_MEMORY_BASE_ADDR = 512; // user memory base address, requires 1 kB EEPROM and reserves room for future system settings
    const unsigned long USER_MEMORY_EMPTY_FREQ = 0xFFFFFFFFUL; // erased/uninitialized user memory frequency value
    const uint8_t EEPROM_ERASED_BYTE = 0xFF; // erased/uninitialized EEPROM byte value

    // I²C settings
    const unsigned long wireTimeout = 5000; // I²C transmission timeout, preventing I²C bus crash in some cases
    const unsigned long i2cClock = 32000; // low I²C clock frequency, more robust through SDA/SCL RF decoupling circuitry (min. 31.25 kHz for 16 MHz ATmega328P)
    const unsigned long i2cHealthCheckInterval = 2000; // I²C health check interval
    const uint8_t i2cMaxRetries = 10; // maximum number of I²C transmission retries
    const uint8_t i2cRetryDelay = 50; // delay between I²C retries

    // PLL settings
        // charge pump
        enum { CP_LOW = 0, CP_HIGH = 1, CP_DISABLED = 2 }; // charge-pump mode: 0 = low (50 µA), 1 = high (220 µA), 2 = disabled
        const uint8_t defaultChargePumpMode = CP_HIGH; // default charge-pump mode

        // XTAL
        const unsigned long PLL_XTAL_OPTIONS[] = { 1600000UL, 3200000UL }; // PLL crystal frequency options (Hz)
        const uint8_t PLL_XTAL_MAX_DECIMALS[] = { 3, 2 }; // maximum decimal precision per PLL crystal frequency option
        const uint8_t numXTALFreqOptions = sizeof(PLL_XTAL_OPTIONS) / sizeof(PLL_XTAL_OPTIONS[0]); // total number of selectable PLL crystal frequencies
        const uint8_t defaultXTALFreqIndex = 1; // default PLL crystal frequency index = 3.2 MHz
        const uint8_t defaultNumDecimals = 2; // default VCO frequency decimal precision

        // I²C
        const byte PLL_ADDRESSES[] = { 0x60, 0x61, 0x62, 0x63 }; // optional I²C addressing by P3-biasing; refer to TSA5511 datasheet, table 4 (0x61 is always valid)
        const uint8_t numPLLAddresses = sizeof(PLL_ADDRESSES) / sizeof(PLL_ADDRESSES[0]); // total number of selectable PLL I²C addresses
        const uint8_t defaultPLLAddrIndex = 1; // default I²C address index = 0x61

        // output ports
        enum { OUTPUT_PORT_PHASE_LOCKED, OUTPUT_PORT_PHASE_UNLOCKED, OUTPUT_PORT_PHASE_RF_DRIVE, numOutputPortEditPhases }; // output port editing phases
        const uint8_t defaultPortIdxLock = 5; // default output port for an external lock indicator
        const uint8_t defaultPortIdxUnlock = 6; // default output port for an external unlock indicator
        const uint8_t defaultPortIdxRF = 2; // default output port to activate RF output stage in locked state

        // control constants
        const byte PLL_PORT_NONE = 0x08; // no output port assigned
        const byte PLL_BYTE1_FL = 0x40; // bit6 (lock flag)
        const byte PLL_BYTE4_BASE = 0x8E; // CP=0, T1=0, T0=0, OS=0
        const byte PLL_BYTE4_CP = 0x40; // bit6 (CP)
        const byte PLL_BYTE4_OS = 0x01; // bit0 (OS = varicap drive disable)

        // divisor settings
        const uint16_t PLL_XTAL_DIVISOR = 512; // crystal frequency divisor
        const uint16_t PLL_DIVISOR_LIMIT = 0x7FFF; // cap divisor to 15 bits (MSB of high byte must remain zero)
        const uint8_t PLL_PRESCALER_DIVISOR = 8; // prescaler divisor

    // VCO frequency band settings
    const unsigned long hzPerMHz = 1000000UL; // number of Hz in one MHz
    const unsigned long freqBands[][2] = {
        { 65800000UL,   74000000UL }, // OIRT FM broadcast
        { 76000000UL,   95000000UL }, // FM broadcast - Japan
        { 87000000UL,  108000000UL }, // FM broadcast - ITU R1/R2/R3
        { 144000000UL, 148000000UL }, // 2 m amateur band - ITU R1/R2/R3
        { 420000000UL, 450000000UL }, // 70 cm amateur band - ITU R1/R2/R3
        { 470000000UL, 862000000UL }, // UHF band - ITU R1/R2/R3 (with XTAL = 1.6 MHz, upper frequency will be capped to 819.175 MHz)
        { 64000000UL, 1300000000UL }  // full range as per TSA5511 specification (with XTAL = 1.6 MHz, upper frequency will be capped to 819.175 MHz)
    };
    const byte defaultFreqBandIndex = 2; // default VCO frequency band
    const byte numFreqBands = sizeof(freqBands) / sizeof(freqBands[0]); // total number of selectable VCO frequency bands

    // user memory settings
    const byte numUserMemorySlots = 6; // number of user memory slots per VCO frequency band and XTAL frequency

    // station name settings
    const uint8_t maxNameLength = lcdColumns; // maximum station name length
    const uint8_t asciiRange[2] = { 32, 126 }; // printable ASCII character range
    const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator

    // timing parameters
        // UI delays and timeouts
        const unsigned long splashDelay = 2500; // duration to show splash screen
        const unsigned long animInterval = 250; // animation speed during PLL unlocked status
        const unsigned long rfOutputStatusInterval = 1000; // RF drive disabled status alternation interval
        const unsigned long errBlinkRate = 250; // error indicator blink interval
        const unsigned long i2cFallbackDelay = 2500; // duration to show I²C address fallback notification
        const unsigned long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
        const unsigned long menuTimeout = 20000; // inactivity timeout in menu navigation mode
        const unsigned long menuEntryTitleDelay = 1200; // duration to show menu entry title
        const unsigned long menuEntryAnimDelay = 70; // menu entry transition animation step delay

        // button input timings and thresholds
        const unsigned long initialPressDelay = 1000; // delay before auto-repeat when holding button
        const unsigned long setLongPressDelay = 1200; // delay before SET long-press actions are triggered
        const unsigned long charScrollInterval = 200; // interval between character scroll steps
        const unsigned long baseRepeatInterval = 80; // initial repeat interval before acceleration
        const float pressAccelerationBase = 0.7; // base factor for adaptive auto-repeat acceleration when holding UP/DOWN
        const float pressTargetSweepSpeed = 500.0; // target sweep speed (Hz per ms) - high speeds may be limited by loop/display latencies
        const uint16_t setClickInterval = 350; // double-click detection threshold
        const uint8_t buttonTimingTolerance = 50; // minimum time window to suppress unwanted button event

    // menu
    enum {
        MENU_LEVEL_MAIN,
        MENU_LEVEL_VCO_SETTINGS,
        MENU_LEVEL_PLL_SETTINGS,
        MENU_LEVEL_GENERAL_SETTINGS,
        MENU_LEVEL_QUICK_MENU,
        MENU_LEVEL_USER_MEMORY,
        numMenuLevels
    };

    enum {
        SYSTEM_MENU_VCO_SETTINGS,
        SYSTEM_MENU_PLL_SETTINGS,
        SYSTEM_MENU_GENERAL_SETTINGS,
        SYSTEM_MENU_EXIT,
        numSystemMenuItems
    };

    enum {
        VCO_MENU_FREQUENCY_BAND,
        VCO_MENU_PRECISION,
        VCO_MENU_EXIT,
        numVCOMenuItems
    };

    enum {
        PLL_MENU_I2C_ADDRESS,
        PLL_MENU_XTAL_FREQUENCY,
        PLL_MENU_CHARGE_PUMP,
        PLL_MENU_PORT_MAPPING,
        PLL_MENU_EXIT,
        numPLLMenuItems
    };

    enum {
        GENERAL_MENU_STATION_NAME,
        GENERAL_MENU_BACKLIGHT_DIMMER,
        GENERAL_MENU_SHOW_MENU_TITLE,
        GENERAL_MENU_FACTORY_RESET,
        GENERAL_MENU_EXIT,
        numGeneralMenuItems
    };

    enum {
        USER_MENU_RECALL_MEMORY,
        USER_MENU_STORE_MEMORY,
        USER_MENU_CLEAR_MEMORY,
        USER_MENU_RF_DRIVE,
        USER_MENU_LCD_OFF,
        USER_MENU_EXIT,
        numUserMenuItems
    };

    enum {
        EXIT_MENU_SAVE_CHANGES,
        EXIT_MENU_DISCARD,
        EXIT_MENU_CANCEL,
        numExitMenuItems
    };

    // number of selectable menu items per menu level
    const uint8_t menuLength[numMenuLevels] = {
        numSystemMenuItems,       // MENU_LEVEL_MAIN
        numVCOMenuItems,          // MENU_LEVEL_VCO_SETTINGS
        numPLLMenuItems,          // MENU_LEVEL_PLL_SETTINGS
        numGeneralMenuItems,      // MENU_LEVEL_GENERAL_SETTINGS
        numUserMenuItems,         // MENU_LEVEL_QUICK_MENU
        numUserMemorySlots + 1    // MENU_LEVEL_USER_MEMORY, including return option
    };


// === DISPLAY MODES ===
    enum {
        SPLASH_SCREEN,
        MAIN_INTERFACE,
        SET_FREQUENCY_INTERFACE,
        MENU_INTERFACE,
        STATION_NAME_EDITOR,
        PLL_LOCK_STATUS,
        MENU_ENTRY_TITLE,
        LCD_HIBERNATE,
        I2C_ERROR
    };


// === RUNTIME STATE VARIABLES ===
    // I²C
    uint8_t pllAddrIndex = defaultPLLAddrIndex; // current I²C address index
    uint8_t tempPllAddrIndex = pllAddrIndex; // temporary index used for I²C address selection in menu edit mode
    bool i2cFallbackActive = false; // true if SET is pressed during startup (I²C address safe fallback)

    // PLL settings
    uint8_t xtalFreqIndex = defaultXTALFreqIndex; // current PLL crystal frequency index
    uint8_t portIdxLock = defaultPortIdxLock; // output port for an external lock indicator
    uint8_t portIdxUnlock = defaultPortIdxUnlock; // output port for an external unlock indicator
    uint8_t portIdxRF = defaultPortIdxRF; // output port to activate RF output stage in locked state
    uint8_t cpMode = defaultChargePumpMode; // current charge-pump mode
    bool pllLock = false; // true if PLL is locked
    bool pllCheckPending = false; // true if new PLL divisor value is pending
    bool rfOutputEnabled = true; // true if RF drive output is temporarily enabled

    // buttons
    bool buttonDownState = false; // momentary state of DOWN button
    bool buttonSetState = false; // momentary state of SET button
    bool buttonUpState = false; // momentary state of UP button
    bool buttonDownPressed = false; // static state of DOWN button
    bool buttonSetPressed = false; // static state of SET button
    bool buttonUpPressed = false; // static state of UP button

    // station name editor
    char stationName[maxNameLength + 1]; // current station name (null-terminated)
    char stationNameBuffer[maxNameLength + 1]; // buffer used during editing (null-terminated)
    uint8_t editPosition = 0; // actual cursor position in station name editor
    bool stationNameEditMode = false; // true if station name editor is active

    // LCD backlight control
    bool backlightDimActive = defaultBacklightDimActive; // true if dimmer function is enabled
    bool showMenuTitle = defaultShowMenuTitle; // true if menu entry titles are shown when entering menus
    bool backlightOffRequested = false; // true if LCD backlight off was requested from the quick menu
    bool backlightOff = false; // true if backlight is off

    // frequency control
    unsigned long lowerFreq = 0; // lower VCO frequency band edge
    unsigned long upperFreq = 0; // upper VCO frequency band edge
    unsigned long currentFreq = 0; // last successfully programmed VCO frequency
    unsigned long targetFreq = 0; // target VCO frequency
    byte freqBandIndex[numXTALFreqOptions]; // currently selected VCO frequency band index per XTAL frequency
    uint8_t numDecimals = defaultNumDecimals; // VCO frequency decimal precision
    bool freqSetMode = false; // true if frequency set mode is active

    // menu
    unsigned long menuLockStartTime = 0; // time at which temporary menu input lock started
    unsigned long menuInactivityTimer = 0; // tracks last interaction time in menu mode
    uint8_t menuLevel = MENU_LEVEL_MAIN; // selected menu level
    uint8_t menuIndex = SYSTEM_MENU_VCO_SETTINGS; // selected item index within current menu level
    uint8_t prevMainMenuIndex = SYSTEM_MENU_VCO_SETTINGS; // main menu index to restore when returning from a system submenu
    uint8_t outputPortsEditPhase = OUTPUT_PORT_PHASE_LOCKED; // selected output port edit phase
    uint8_t userMemoryAction = USER_MENU_RECALL_MEMORY; // selected user memory action
    bool menuMode = false; // true if menu session is active
    bool menuEditMode = false; // true when editing a setting
    bool menuExitConfirmMode = false; // true if SAVE/DISCARD/CANCEL exit submenu is active
    bool factoryResetConfirm = false; // true when factory reset confirmation is set to yes
    bool factoryResetFinalConfirm = false; // true when final factory reset confirmation is active
    bool ignoreFirstSetInMenu = false; // skip input until SET is released


// === PLL OUTPUT PORT UTILITIES ===
    bool isValidPLLPort(uint8_t portIndex) {
        return portIndex < PLL_PORT_NONE;
    }

    bool isMappedToOtherOutput(uint8_t portIndex) {
        if (!isValidPLLPort(portIndex)) return false; // allow 'none' to be assigned to multiple functions
        return (outputPortsEditPhase != OUTPUT_PORT_PHASE_LOCKED && portIndex == portIdxLock)
            || (outputPortsEditPhase != OUTPUT_PORT_PHASE_UNLOCKED && portIndex == portIdxUnlock)
            || (outputPortsEditPhase != OUTPUT_PORT_PHASE_RF_DRIVE && portIndex == portIdxRF);
    }

    void adjustOutputPort(int8_t direction) {
        uint8_t* portIndex = &portIdxRF;
        if (outputPortsEditPhase == OUTPUT_PORT_PHASE_LOCKED) portIndex = &portIdxLock;
        else if (outputPortsEditPhase == OUTPUT_PORT_PHASE_UNLOCKED) portIndex = &portIdxUnlock;

        for (uint8_t attempts = 0; attempts <= PLL_PORT_NONE; attempts++) {
            if (direction < 0) {
                if (*portIndex == PLL_PORT_NONE) *portIndex = PLL_PORT_NONE - 1;
                else if (*portIndex == 0) *portIndex = PLL_PORT_NONE;
                else (*portIndex)--;
            } else {
                if (*portIndex == PLL_PORT_NONE) *portIndex = 0;
                else if (*portIndex == PLL_PORT_NONE - 1) *portIndex = PLL_PORT_NONE;
                else (*portIndex)++;
            }

            if (!isMappedToOtherOutput(*portIndex)) return;
        }
    }

    bool updateLockedOutputPorts() {
        if (!pllLock) return false;

        uint8_t portsHigh = 0x00;
        if (isValidPLLPort(portIdxLock)) portsHigh |= (1 << portIdxLock);
        if (isValidPLLPort(portIdxRF) && rfOutputEnabled) portsHigh |= (1 << portIdxRF);

        byte data[2]; // partial programming, starting with byte 4 (TSA5511 datasheet, table 1)
        data[0] = PLL_BYTE4_BASE // set charge pump
            | ((cpMode == CP_HIGH) ? PLL_BYTE4_CP : 0)
            | ((cpMode == CP_DISABLED) ? PLL_BYTE4_OS : 0);
        data[1] = portsHigh; // set output ports
        return attemptI2C(false, PLL_ADDRESSES[pllAddrIndex], data, 2);
    }


// === FREQUENCY UTILITIES ===
    // Minimum VCO frequency step size in Hz, derived from the PLL reference frequency and prescaler
    unsigned long getVCOFreqStep() { return (PLL_XTAL_OPTIONS[xtalFreqIndex] * PLL_PRESCALER_DIVISOR) / PLL_XTAL_DIVISOR; }

    uint8_t getMaxNumDecimals() {
        return (xtalFreqIndex < numXTALFreqOptions) ? PLL_XTAL_MAX_DECIMALS[xtalFreqIndex] : defaultNumDecimals;
    }

    // determine step size multiplier for current display precision and XTAL setting
    uint8_t getStepSizeMultiplier() {
        unsigned long displayStepSize = hzPerMHz;  // base display step size in Hz before applying decimal precision
        for (uint8_t i = constrain(numDecimals, 0, getMaxNumDecimals()); i; i--) displayStepSize /= 10;
        return (displayStepSize + getVCOFreqStep() - 1) / getVCOFreqStep(); // multiplier to ensure that display precision is met (integer ceiling)
    }

    // VCO frequency validation
    unsigned long validateFreq(unsigned long frequency, bool alignToStepSize = false, int8_t alignDirection = 0) {
        frequency = constrain(frequency, getVCOFreqStep(), (unsigned long)PLL_DIVISOR_LIMIT * getVCOFreqStep()); // constrain VCO frequency within valid PLL divisor range
        unsigned long step = getVCOFreqStep() * (alignToStepSize ? getStepSizeMultiplier() : 1); // select step size (base PLL step size or visible step size)
        if (alignDirection > 0) return ((frequency + step - 1) / step) * step; // align frequency upward to next valid step
        if (alignDirection < 0) return (frequency / step) * step; // align frequency downward to previous valid step
        return ((frequency + step / 2) / step) * step; // align frequency to nearest valid step using integer rounding
    }


// === MAIN PROGRAM LOGIC ===
void setup() {
    setupHardware();
    initialize();
}

void loop() {
    readButtons();
    handleBacklightControl();
    handleFrequencyChange();
    handleMenu();
    checkPLL();
    checkI2C();
}

void setupHardware() {
    // I/O pins
    pinMode(downButton, INPUT_PULLUP);
    pinMode(setButton, INPUT_PULLUP);
    pinMode(upButton, INPUT_PULLUP);
    pinMode(lockIndicator, OUTPUT);
    pinMode(errIndicator, OUTPUT);
    pinMode(lcdBacklight, OUTPUT);

    // I²C
    Wire.begin();
    Wire.setClock(i2cClock);
    Wire.setWireTimeout(wireTimeout, true);

    // display
    lcd.begin(lcdColumns, lcdRows);
}

void initialize() {
    analogWrite(lcdBacklight, maxBrightness);
    display(SPLASH_SCREEN);
    delay(splashDelay);
    restoreI2CDefaults();
    readSettings();
    configurePLL();
    display(MAIN_INTERFACE);
    display(PLL_LOCK_STATUS);
}

// hold SET during startup to restore safe default I²C address (0x61)
void restoreI2CDefaults() {
    if (digitalRead(setButton)) return;
    pllAddrIndex = defaultPLLAddrIndex;
    storeI2CAddress();
    i2cFallbackActive = true;
    display(I2C_ERROR);
    i2cFallbackActive = false;
    delay(i2cFallbackDelay);
    ignoreFirstSetInMenu = true; // require SET release before allowing menu entry after startup fallback
}

void readSettings() {
    readXTALFreq();
    readNumDecimals();
    readLastFrequencyBand();
    readI2CAddress();
    readChargePump();
    readOutputPorts();
    readStationName();
    readDimmerStatus();
    readShowMenuTitleStatus();
}

void readXTALFreq() {
    uint8_t val = EEPROM.read(EEPROM_XTAL_ADDR);
    xtalFreqIndex = (val < numXTALFreqOptions) ? val : defaultXTALFreqIndex;
}

void readNumDecimals() {
    uint8_t val = EEPROM.read(EEPROM_DECIMAL_ADDR);
    numDecimals = min(val, getMaxNumDecimals()); // limit precision to valid range for current XTAL setting
}

// read selected VCO frequency band from EEPROM and apply new range
void readLastFrequencyBand() {
    // read band index per XTAL frequency
    for (uint8_t i = 0; i < numXTALFreqOptions; i++) {
        uint8_t val = EEPROM.read(EEPROM_FREQBAND_ADDR + i);
        freqBandIndex[i] = (val < numFreqBands) ? val : defaultFreqBandIndex;
    }

    // apply current band for active XTAL frequency
    uint8_t currentBandIndex = freqBandIndex[xtalFreqIndex];
    lowerFreq = validateFreq(freqBands[currentBandIndex][0], true, 1);
    upperFreq = validateFreq(freqBands[currentBandIndex][1], true, -1);

    // load last-used frequency for this band and XTAL frequency if valid, else default to lower band edge
    unsigned long storedFreq = readLastBandFrequency(currentBandIndex, xtalFreqIndex);
    targetFreq = (storedFreq >= lowerFreq && storedFreq <= upperFreq) ? validateFreq(storedFreq, false) : lowerFreq;
}

// read last-used VCO frequency for selected band index and XTAL frequency
unsigned long readLastBandFrequency(byte bandIndex, byte xtalIndex) {
    if (bandIndex >= numFreqBands || xtalIndex >= numXTALFreqOptions) return 0;
    unsigned long freq;
    EEPROM.get(EEPROM_BAND_FREQ_BASE_ADDR + ((bandIndex * numXTALFreqOptions + xtalIndex) * sizeof(unsigned long)), freq);
    return freq;
}

void readI2CAddress() {
    uint8_t val = EEPROM.read(EEPROM_I2C_ADDR);
    pllAddrIndex = (val < numPLLAddresses) ? val : defaultPLLAddrIndex;
}

void readChargePump() {
    uint8_t v = EEPROM.read(EEPROM_CP_ADDR); // read stored charge-pump mode
    cpMode = (v <= CP_DISABLED) ? v : defaultChargePumpMode; // validate and apply default if needed
}

void readOutputPorts() {
    uint8_t b0 = EEPROM.read(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_LOCKED); // read locked index, PLL_PORT_NONE, or EEPROM_ERASED_BYTE if EEPROM is blank
    uint8_t b1 = EEPROM.read(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_UNLOCKED); // read unlocked index, PLL_PORT_NONE, or EEPROM_ERASED_BYTE if EEPROM is blank
    uint8_t b2 = EEPROM.read(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_RF_DRIVE); // read RF drive index, PLL_PORT_NONE, or EEPROM_ERASED_BYTE if EEPROM is blank

    // apply default output port mapping in case of uninitialized/blank EEPROM
    if (b0 == EEPROM_ERASED_BYTE && b1 == EEPROM_ERASED_BYTE && b2 == EEPROM_ERASED_BYTE) {
        portIdxLock = defaultPortIdxLock;
        portIdxUnlock = defaultPortIdxUnlock;
        portIdxRF = defaultPortIdxRF;
        return;
    }

    // clamp to valid range, else 'none'
    portIdxLock = isValidPLLPort(b0) ? b0 : PLL_PORT_NONE; // locked index or 'none'
    portIdxUnlock = isValidPLLPort(b1) ? b1 : PLL_PORT_NONE; // unlocked index or 'none'
    portIdxRF = isValidPLLPort(b2) ? b2 : PLL_PORT_NONE; // RF drive index or 'none'

    // remove duplicate output port assignments from old or corrupted EEPROM data
    if (isValidPLLPort(portIdxUnlock) && portIdxUnlock == portIdxLock) portIdxUnlock = PLL_PORT_NONE;
    if (isValidPLLPort(portIdxRF) && (portIdxRF == portIdxLock || portIdxRF == portIdxUnlock)) portIdxRF = PLL_PORT_NONE;
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName); // read station name from EEPROM
    bool invalid = stationName[maxNameLength] != '\0'; // check if station name has a valid null terminator
    stationName[maxNameLength] = '\0'; // ensure null terminator regardless

    // check for invalid characters or uninitialized EEPROM content
    for (uint8_t i = 0; i < maxNameLength; i++) {
        char c = stationName[i];
        if (c < asciiRange[0] || c > asciiRange[1] || c == EEPROM_ERASED_BYTE) {
            invalid = true;
            break;
        }
    }
    if (invalid) {
        strncpy(stationName, defaultName, maxNameLength); // copy default station name to array
        memset(stationName + strlen(defaultName), ' ', maxNameLength - strlen(defaultName)); // fill remainder with spaces
        stationName[maxNameLength] = '\0'; // ensure null terminator again
    }
}

void readDimmerStatus() {
    uint8_t val = EEPROM.read(EEPROM_DIM_ADDR);
    backlightDimActive = (val == EEPROM_ERASED_BYTE) ? defaultBacklightDimActive : (val != 0); // erased value falls back to default; any other non-zero value reads as true
}

void readShowMenuTitleStatus() {
    uint8_t val = EEPROM.read(EEPROM_SHOW_MENU_TITLE_ADDR);
    showMenuTitle = (val == EEPROM_ERASED_BYTE) ? defaultShowMenuTitle : (val != 0); // erased value falls back to default; any other non-zero value reads as true
}

void readButtons() {
    static unsigned long lastMultiButtonTime = 0;
    unsigned long currentMillis = millis();

    buttonDownState = !digitalRead(downButton);
    buttonSetState  = !digitalRead(setButton);
    buttonUpState   = !digitalRead(upButton);

    if ((buttonDownState + buttonSetState + buttonUpState) > 1) {
        buttonDownState = buttonSetState = buttonUpState = false;
        lastMultiButtonTime = currentMillis;
    } else if (currentMillis - lastMultiButtonTime < buttonTimingTolerance) {
        buttonDownState = buttonSetState = buttonUpState = false;
    }
}

void handleBacklightControl() {
    static unsigned long dimmerTimer = 0;
    static unsigned long lastDimmerUpdateTime = 0;
    static unsigned long lastRFOutputStatusTime = 0;
    static uint8_t currentBrightness = maxBrightness;

    if (backlightOffRequested) {
        backlightOffRequested = false;
        analogWrite(lcdBacklight, 0);
        backlightOff = true;
        display(LCD_HIBERNATE);
        while (!digitalRead(setButton)); // ensure that SET is released, to prevent backlight from being turned on again
        buttonSetState = buttonSetPressed = false;
        return;
    }

    // no LCD backlight control in frequency set mode or menu mode
    if (freqSetMode || menuMode) {
        dimmerTimer = 0;
        return;
    }

    unsigned long currentMillis = millis();

    // restore brightness by pressing any button
    if (buttonDownState || buttonSetState || buttonUpState) {
        currentBrightness = maxBrightness;
        analogWrite(lcdBacklight, currentBrightness);
        if (backlightOff) {
            backlightOff = false;
            display(LCD_HIBERNATE);
            uint8_t btn = buttonDownState ? downButton : buttonUpState ? upButton : setButton; // determine which button triggered the wake-up
            while (!digitalRead(btn)); // wait for button release
            switch (btn) { // clear button state and press flags to suppress immediate action
                case downButton: buttonDownState = buttonDownPressed = false; break;
                case upButton: buttonUpState = buttonUpPressed = false; break;
                case setButton: buttonSetState = buttonSetPressed = false; break;
            }
        }
        dimmerTimer = currentMillis;
    }

    // no LCD backlight dimming while RF drive is temporarily disabled
    if (!rfOutputEnabled) {
        dimmerTimer = 0;
        if (!backlightOff && currentBrightness < maxBrightness) {
            currentBrightness = maxBrightness;
            analogWrite(lcdBacklight, currentBrightness);
        }
        if (!backlightOff && currentMillis - lastRFOutputStatusTime >= rfOutputStatusInterval) {
            lastRFOutputStatusTime = currentMillis;
            display(MAIN_INTERFACE);
            display(PLL_LOCK_STATUS);
        }
        return;
    }

    // no LCD backlight dimming if unlocked
    if (!pllLock) {
        dimmerTimer = 0;
        return;
    }

    // initialize dimmer timer without blocking SET long-press handling
    if (dimmerTimer == 0) {
        dimmerTimer = currentMillis;
    }

    // gradual dimming after timeout
    if (backlightDimActive && (currentMillis - dimmerTimer > dimDelay) && !backlightOff && currentBrightness > lowBrightness) {
        if (currentMillis - lastDimmerUpdateTime >= dimStepDelay) {
            currentBrightness--;
            analogWrite(lcdBacklight, currentBrightness);
            lastDimmerUpdateTime = currentMillis;
        }
    }
}

void handleFrequencyChange() {
    static unsigned long freqSetInactivityTimer = 0;
    static bool timedOut = true;

    if (menuMode) return;
    if (!(buttonDownState || buttonSetState || buttonUpState) && timedOut) return; // avoid redundant processing

    unsigned long currentMillis = millis();

    // change VCO frequency
    auto changeFreq = [](int8_t direction) { applyFrequencyChange(true, &targetFreq, direction); };
    handleButtonInput(buttonDownState, buttonDownPressed, -1, changeFreq);
    handleButtonInput(buttonUpState, buttonUpPressed, 1, changeFreq);
    if (buttonDownState || buttonUpState) {
        freqSetInactivityTimer = currentMillis;
        timedOut = false;
    }

    // confirm new frequency on SET, or cancel after timeout
    if (freqSetMode && buttonSetState) {
        applyFrequencyChange(false, &targetFreq, 0);
        timedOut = true;
    } else if (currentMillis - freqSetInactivityTimer > freqSetTimeout) {
        freqSetMode = false;
        if (!timedOut) {
            targetFreq = currentFreq;
            display(MAIN_INTERFACE);
            timedOut = true;
        }
    }
}

void applyFrequencyChange(bool adjusting, unsigned long* targetFreq, int8_t direction) {
    if (adjusting) {
        // UP/DOWN action
        if (freqSetMode) {
            unsigned long step = min(getStepSizeMultiplier() * getVCOFreqStep(), upperFreq - lowerFreq); // constrain step size within range
            *targetFreq = (direction > 0) ? *targetFreq + step : *targetFreq - step;
        }
        *targetFreq = (*targetFreq < lowerFreq) ? upperFreq : (*targetFreq > upperFreq) ? lowerFreq : *targetFreq;
        display(SET_FREQUENCY_INTERFACE);
        freqSetMode = true;
    } else {
        // SET action
        unsigned long currentMillis = millis();
        configurePLL();
        freqSetMode = false;
        menuLockStartTime = currentMillis; // block menu entry immediately after freqSetMode
        ignoreFirstSetInMenu = true;
        display(MAIN_INTERFACE);
    }
}

void handleMenu() {
    static unsigned long lastShortClickTime = 0;
    static unsigned long clickStartTime = 0;
    static unsigned long setPressStartTime = 0;
    static bool setLongPressHandled = false;
    unsigned long currentMillis = millis();

    auto returnToMainInterface = []() {
        menuMode = false;
        menuEditMode = false;
        menuExitConfirmMode = false;
        stationNameEditMode = false;
        menuLevel = MENU_LEVEL_MAIN;
        menuIndex = SYSTEM_MENU_VCO_SETTINGS;
        userMemoryAction = USER_MENU_RECALL_MEMORY;
        menuLockStartTime = millis();
        ignoreFirstSetInMenu = true;
        display(MAIN_INTERFACE);
        display(PLL_LOCK_STATUS);
    };

    auto returnFromUserMenu = [&]() {
        if (menuLevel == MENU_LEVEL_USER_MEMORY) {
            menuLevel = MENU_LEVEL_QUICK_MENU;
            menuIndex = userMemoryAction;
            display(MENU_INTERFACE);
        } else {
            returnToMainInterface();
        }
    };

    auto returnFromSystemMenu = [&]() {
        if (stationNameEditMode) return; // station name editor has its own SET handling
        if (menuEditMode) {
            menuEditMode = false;
            outputPortsEditPhase = OUTPUT_PORT_PHASE_LOCKED;
            factoryResetConfirm = false;
            factoryResetFinalConfirm = false;
            display(MENU_INTERFACE);
            return;
        }
        if (menuExitConfirmMode) {
            menuExitConfirmMode = false;
            menuLevel = MENU_LEVEL_MAIN;
            menuIndex = SYSTEM_MENU_EXIT;
        } else if (menuLevel == MENU_LEVEL_MAIN) {
            menuExitConfirmMode = true;
            menuIndex = EXIT_MENU_SAVE_CHANGES;
        } else {
            menuLevel = MENU_LEVEL_MAIN;
            menuIndex = prevMainMenuIndex;
        }
        display(MENU_INTERFACE);
    };

    // reset menu timeout on any button press
    if (buttonDownState || buttonUpState || buttonSetState) menuInactivityTimer = currentMillis;

    // discard changes and exit menu on timeout if inactive (excluding exit menu)
    if (menuMode && !menuExitConfirmMode && currentMillis - menuInactivityTimer > menuTimeout) {
        if (menuLevel >= MENU_LEVEL_QUICK_MENU) {
            returnToMainInterface();
            return;
        }
        stationNameEditMode = false;
        restoreSettings();
        display(PLL_LOCK_STATUS);
        menuLockStartTime = currentMillis;
        ignoreFirstSetInMenu = true;
        return;
    }

    // skip input until SET is released
    if (ignoreFirstSetInMenu) {
        if (!buttonSetState) {
            ignoreFirstSetInMenu = false;
            buttonSetPressed = false;
            setPressStartTime = 0;
            setLongPressHandled = false;
        }
        return;
    }

    if (!menuMode) {
        // block menu access if station name editor is active or menu input is temporarily locked
        if (stationNameEditMode || currentMillis - menuLockStartTime < setClickInterval) return;

        // detect SET long-press for quick menu and SET double-click for settings menu
        if (buttonSetState && clickStartTime == 0) clickStartTime = currentMillis;
        if (buttonSetState && clickStartTime != 0 && currentMillis - clickStartTime > setLongPressDelay && !backlightOff) {
            clickStartTime = 0;
            lastShortClickTime = 0;
            menuMode = true;
            menuLevel = MENU_LEVEL_QUICK_MENU;
            menuIndex = USER_MENU_RECALL_MEMORY;
            menuEditMode = false;
            menuExitConfirmMode = false;
            stationNameEditMode = false;
            factoryResetConfirm = false;
            factoryResetFinalConfirm = false;
            userMemoryAction = USER_MENU_RECALL_MEMORY;
            display(MENU_ENTRY_TITLE);
            display(MENU_INTERFACE);
            ignoreFirstSetInMenu = true;
            menuInactivityTimer = millis();
            return;
        }
        if (!buttonSetState && clickStartTime != 0) {
            unsigned long clickDuration = currentMillis - clickStartTime;
            clickStartTime = 0;
            if (clickDuration < setClickInterval) {
                bool validDoubleClick =
                    currentMillis - lastShortClickTime < setClickInterval &&
                    currentMillis - lastShortClickTime >= buttonTimingTolerance;
                lastShortClickTime = currentMillis;
                if (validDoubleClick) { // valid double-click detected: enter settings menu
                    menuMode = true;
                    menuLevel = MENU_LEVEL_MAIN;
                    menuIndex = SYSTEM_MENU_VCO_SETTINGS;
                    menuEditMode = false;
                    menuExitConfirmMode = false;
                    stationNameEditMode = false;
                    factoryResetConfirm = false;
                    factoryResetFinalConfirm = false;
                    tempPllAddrIndex = pllAddrIndex; // initialize temporary I²C address index
                    strncpy(stationNameBuffer, stationName, maxNameLength); // fill buffer with current station name as safe default
                    stationNameBuffer[maxNameLength] = '\0';
                    outputPortsEditPhase = OUTPUT_PORT_PHASE_LOCKED;
                    display(MENU_ENTRY_TITLE);
                    display(MENU_INTERFACE);
                    ignoreFirstSetInMenu = true; // skip until SET release
                    menuInactivityTimer = millis();
                    return;
                }
            }
        }
        return;
    }

    if (stationNameEditMode) {
        // station name editor active as submenu
        handleStationNameEdit();
        return;
    }

    bool setShortAction = false;
    bool setLongAction = false;

    if (buttonSetState) {
        if (!buttonSetPressed) {
            buttonSetPressed = true;
            setPressStartTime = currentMillis;
            setLongPressHandled = false;
        } else if (!setLongPressHandled && currentMillis - setPressStartTime >= setLongPressDelay) {
            setLongPressHandled = true;
            setLongAction = true;
        }
    } else {
        if (buttonSetPressed) {
            if (!setLongPressHandled) setShortAction = true;
            buttonSetPressed = false;
            setPressStartTime = 0;
            setLongPressHandled = false;
        }
    }

    if (menuLevel >= MENU_LEVEL_QUICK_MENU) {
        handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
            if (menuLevel == MENU_LEVEL_USER_MEMORY) {
                menuIndex = (menuIndex == 0) ? (menuLength[menuLevel] - 1) : (menuIndex - 1);
                display(MENU_INTERFACE);
            } else if (menuIndex > 0) {
                menuIndex--;
                display(MENU_INTERFACE);
            }
        });
        handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
            if (menuLevel == MENU_LEVEL_USER_MEMORY) {
                menuIndex = (menuIndex >= menuLength[menuLevel] - 1) ? 0 : (menuIndex + 1);
                display(MENU_INTERFACE);
            } else if (menuIndex < menuLength[menuLevel] - 1) {
                menuIndex++;
                display(MENU_INTERFACE);
            }
        });

        if (setLongAction) {
            returnFromUserMenu();
            return;
        }
        if (setShortAction) {
            if (menuLevel == MENU_LEVEL_QUICK_MENU) {
                if (menuIndex < USER_MENU_RF_DRIVE) {
                    userMemoryAction = menuIndex;
                    menuLevel = MENU_LEVEL_USER_MEMORY;
                    menuIndex = 0;
                    display(MENU_INTERFACE);
                } else if (menuIndex == USER_MENU_RF_DRIVE) {
                    rfOutputEnabled = !rfOutputEnabled;
                    if (pllLock && !updateLockedOutputPorts()) {
                        rfOutputEnabled = !rfOutputEnabled;
                        updateLockedOutputPorts();
                    }
                    display(MENU_INTERFACE);
                } else if (menuIndex == USER_MENU_LCD_OFF) {
                    menuMode = false;
                    menuEditMode = false;
                    menuExitConfirmMode = false;
                    menuLevel = MENU_LEVEL_MAIN;
                    menuIndex = SYSTEM_MENU_VCO_SETTINGS;
                    userMemoryAction = USER_MENU_RECALL_MEMORY;
                    backlightOffRequested = true;
                    menuLockStartTime = millis();
                    ignoreFirstSetInMenu = true;
                } else {
                    returnToMainInterface();
                }
            } else if (menuLevel == MENU_LEVEL_USER_MEMORY) {
                if (menuIndex == numUserMemorySlots) {
                    returnFromUserMenu();
                    return;
                }
                if (userMemoryAction == USER_MENU_RECALL_MEMORY) {
                    unsigned long memoryFreq;
                    if (readUserMemoryFrequency(menuIndex, memoryFreq)) {
                        targetFreq = memoryFreq;
                        configurePLL();
                        returnToMainInterface();
                    } else {
                        display(MENU_INTERFACE);
                    }
                } else if (userMemoryAction == USER_MENU_STORE_MEMORY) {
                    storeUserMemoryFrequency(menuIndex);
                    menuLevel = MENU_LEVEL_QUICK_MENU;
                    menuIndex = userMemoryAction;
                    display(MENU_INTERFACE);
                } else if (userMemoryAction == USER_MENU_CLEAR_MEMORY) {
                    clearUserMemoryFrequency(menuIndex);
                    menuLevel = MENU_LEVEL_QUICK_MENU;
                    menuIndex = userMemoryAction;
                    display(MENU_INTERFACE);
                }
            }
        }
        return;
    }

    // handle user input in the exit confirmation submenu (save / discard / cancel)
    if (menuExitConfirmMode) {
        handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
            if (menuIndex > 0) {
                menuIndex--;
                display(MENU_INTERFACE);
            }
        });
        handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
            if (menuIndex < numExitMenuItems - 1) {
                menuIndex++;
                display(MENU_INTERFACE);
            }
        });

        if (setLongAction) {
            returnFromSystemMenu();
            return;
        }
        if (setShortAction) {
            if (menuIndex == EXIT_MENU_SAVE_CHANGES) { // save changes
                if (!stationNameEditMode) {
                    strncpy(stationName, stationNameBuffer, maxNameLength); // copy new station name from edit buffer
                    stationName[maxNameLength] = '\0'; // ensure null terminator
                    storeStationName();
                }
                uint8_t prevPllAddrIndex = pllAddrIndex;
                if (tempPllAddrIndex != prevPllAddrIndex) {
                    if (attemptI2C(false, PLL_ADDRESSES[tempPllAddrIndex], nullptr, 0)) { // new I²C address is valid
                        pllAddrIndex = tempPllAddrIndex;
                    } else { // new I²C address is invalid
                        pllAddrIndex = prevPllAddrIndex;
                        i2cFallbackActive = true;
                        display(I2C_ERROR);
                        i2cFallbackActive = false;
                        unsigned long startTime = millis();
                        while (millis() - startTime < i2cFallbackDelay) {
                            blinkLed(errIndicator, errBlinkRate);
                        }
                        digitalWrite(errIndicator, LOW);
                    }
                }
                storeBandIndex();
                storeNumDecimals();
                storeI2CAddress();
                storeXTALFreq();
                storeChargePump();
                storeOutputPorts();
                storeDimmerStatus();
                storeShowMenuTitleStatus();
                readLastFrequencyBand();
                configurePLL(); // update PLL configuration
                restoreSettings();
            } else if (menuIndex == EXIT_MENU_DISCARD) { // discard changes
                restoreSettings();
            } else if (menuIndex == EXIT_MENU_CANCEL) { // cancel and return to the main menu
                menuExitConfirmMode = false;
                menuIndex = SYSTEM_MENU_VCO_SETTINGS; // return to the first main menu option
                display(MENU_INTERFACE);
            }
        }
        return;
    }

    // menu navigation mode: use UP/DOWN to select option, SET to confirm
    uint8_t maxMenuIndex = menuLength[menuLevel];

    if (!menuEditMode) {
        if (buttonDownState && !buttonDownPressed) {
            buttonDownPressed = true;
            if (menuIndex > 0) menuIndex--;
            display(MENU_INTERFACE);
        } else if (!buttonDownState) {
            buttonDownPressed = false;
        }
        if (buttonUpState && !buttonUpPressed) {
            buttonUpPressed = true;
            if (menuIndex < maxMenuIndex - 1) menuIndex++;
            display(MENU_INTERFACE);
        } else if (!buttonUpState) {
            buttonUpPressed = false;
        }

        if (setLongAction) {
            returnFromSystemMenu();
            return;
        }
        if (setShortAction) {
            if (menuLevel == MENU_LEVEL_MAIN) {
                switch (menuIndex) {
                    case SYSTEM_MENU_VCO_SETTINGS: prevMainMenuIndex = menuIndex; menuLevel = MENU_LEVEL_VCO_SETTINGS; menuIndex = VCO_MENU_FREQUENCY_BAND; break; // enter VCO settings
                    case SYSTEM_MENU_PLL_SETTINGS: prevMainMenuIndex = menuIndex; menuLevel = MENU_LEVEL_PLL_SETTINGS; menuIndex = PLL_MENU_I2C_ADDRESS; break; // enter PLL settings
                    case SYSTEM_MENU_GENERAL_SETTINGS: prevMainMenuIndex = menuIndex; menuLevel = MENU_LEVEL_GENERAL_SETTINGS; menuIndex = GENERAL_MENU_STATION_NAME; break; // enter general settings
                    case SYSTEM_MENU_EXIT: menuExitConfirmMode = true; menuIndex = EXIT_MENU_SAVE_CHANGES; break; // open exit menu
                }
                display(MENU_INTERFACE);
            } else {
                if (menuIndex == menuLength[menuLevel] - 1) {
                    menuLevel = MENU_LEVEL_MAIN; // return option
                    menuIndex = prevMainMenuIndex;
                    display(MENU_INTERFACE);
                } else if (menuLevel == MENU_LEVEL_GENERAL_SETTINGS && menuIndex == GENERAL_MENU_STATION_NAME) {
                    // station name editor
                    editPosition = 0;
                    stationNameEditMode = true;
                    strncpy(stationNameBuffer, stationName, maxNameLength);
                    stationNameBuffer[maxNameLength] = '\0';
                    display(STATION_NAME_EDITOR);
                    return;
                } else {
                    if (menuLevel == MENU_LEVEL_GENERAL_SETTINGS && menuIndex == GENERAL_MENU_FACTORY_RESET) {
                        factoryResetConfirm = false;
                        factoryResetFinalConfirm = false;
                    }
                    menuEditMode = true;
                    display(MENU_INTERFACE);
                }
            }
        }
    } else {
        // menu edit mode: adjust settings
        if (menuLevel == MENU_LEVEL_VCO_SETTINGS) {
            switch (menuIndex) {
                case VCO_MENU_FREQUENCY_BAND:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        auto& idx = freqBandIndex[xtalFreqIndex];
                        idx = (idx == 0) ? (numFreqBands - 1) : (idx - 1);
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        auto& idx = freqBandIndex[xtalFreqIndex];
                        idx = (idx >= numFreqBands - 1) ? 0 : (idx + 1);
                        display(MENU_INTERFACE);
                    });
                    break;
                case VCO_MENU_PRECISION:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (numDecimals > 0) numDecimals--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        uint8_t maxDecimals = getMaxNumDecimals();
                        if (numDecimals < maxDecimals) numDecimals++;
                        display(MENU_INTERFACE);
                    });
                    break;
            }
        } else if (menuLevel == MENU_LEVEL_PLL_SETTINGS) {
            switch (menuIndex) {
                case PLL_MENU_I2C_ADDRESS:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (tempPllAddrIndex > 0) tempPllAddrIndex--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (tempPllAddrIndex + 1 < numPLLAddresses) tempPllAddrIndex++;
                        display(MENU_INTERFACE);
                    });
                    break;
                case PLL_MENU_XTAL_FREQUENCY:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (xtalFreqIndex > 0) xtalFreqIndex--;
                        numDecimals = getMaxNumDecimals();
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (xtalFreqIndex + 1 < numXTALFreqOptions) xtalFreqIndex++;
                        numDecimals = getMaxNumDecimals();
                        display(MENU_INTERFACE);
                    });
                    break;
                case PLL_MENU_CHARGE_PUMP:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (cpMode == CP_DISABLED) cpMode = CP_LOW;
                        else if (cpMode == CP_LOW) cpMode = CP_HIGH;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (cpMode == CP_HIGH) cpMode = CP_LOW;
                        else if (cpMode == CP_LOW) cpMode = CP_DISABLED;
                        display(MENU_INTERFACE);
                    });
                    break;
                case PLL_MENU_PORT_MAPPING: // output ports: function → output port mapping (locked → unlocked → RF drive)
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        adjustOutputPort(-1);
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        adjustOutputPort(1);
                        display(MENU_INTERFACE);
                    });
                    break;
            }
        } else if (menuLevel == MENU_LEVEL_GENERAL_SETTINGS) {
            switch (menuIndex) {
                case GENERAL_MENU_BACKLIGHT_DIMMER:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        backlightDimActive = false;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        backlightDimActive = true;
                        display(MENU_INTERFACE);
                    });
                    break;
                case GENERAL_MENU_SHOW_MENU_TITLE:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        showMenuTitle = false;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        showMenuTitle = true;
                        display(MENU_INTERFACE);
                    });
                    break;
                case GENERAL_MENU_FACTORY_RESET:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        factoryResetConfirm = false;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        factoryResetConfirm = true;
                        display(MENU_INTERFACE);
                    });
                    break;
            }
        }

        // SET confirms and advances phase or returns to menu navigation mode; SET long-press leaves edit mode
        if (setLongAction) {
            returnFromSystemMenu();
            return;
        }
        if (setShortAction) {
            if (menuLevel == MENU_LEVEL_PLL_SETTINGS && menuIndex == PLL_MENU_PORT_MAPPING) {
                // advance edit phase: locked -> unlocked -> RF drive -> exit edit mode
                if (outputPortsEditPhase + 1 < numOutputPortEditPhases) {
                    outputPortsEditPhase++;
                    display(MENU_INTERFACE);
                    return; // stay in edit mode
                } else {
                    outputPortsEditPhase = OUTPUT_PORT_PHASE_LOCKED; // reset for next time
                }
            } else if (menuLevel == MENU_LEVEL_GENERAL_SETTINGS && menuIndex == GENERAL_MENU_FACTORY_RESET) {
                if (factoryResetConfirm) {
                    if (factoryResetFinalConfirm) {
                        factoryReset();
                        menuLockStartTime = millis();
                        ignoreFirstSetInMenu = true;
                        return;
                    }
                    factoryResetFinalConfirm = true;
                    factoryResetConfirm = false;
                    display(MENU_INTERFACE);
                    return;
                }
                factoryResetConfirm = false;
                factoryResetFinalConfirm = false;
            }
            menuEditMode = false;
            display(MENU_INTERFACE);
        }
    }
}

// restore all system settings from EEPROM, realign target frequency and return to main interface
void restoreSettings() {
    readSettings();
    targetFreq = validateFreq(targetFreq, true);
    menuExitConfirmMode = false;
    factoryResetConfirm = false;
    factoryResetFinalConfirm = false;
    menuMode = false;
    menuLevel = MENU_LEVEL_MAIN;
    menuLockStartTime = millis();
    ignoreFirstSetInMenu = true;
    display(MAIN_INTERFACE);
    display(PLL_LOCK_STATUS);
}

// perform factory reset by clearing entire EEPROM and reloading defaults
void factoryReset() {
    for (uint16_t addr = 0; addr < EEPROM.length(); addr++) {
        EEPROM.update(addr, EEPROM_ERASED_BYTE);
    }
    readSettings();
    targetFreq = validateFreq(targetFreq, true);
    tempPllAddrIndex = pllAddrIndex;
    strncpy(stationNameBuffer, stationName, maxNameLength);
    stationNameBuffer[maxNameLength] = '\0';
    outputPortsEditPhase = OUTPUT_PORT_PHASE_LOCKED;
    rfOutputEnabled = true;
    backlightOffRequested = false;
    configurePLL();
    menuMode = false;
    menuEditMode = false;
    menuExitConfirmMode = false;
    factoryResetConfirm = false;
    factoryResetFinalConfirm = false;
    menuLevel = MENU_LEVEL_MAIN;
    menuIndex = SYSTEM_MENU_VCO_SETTINGS;
    menuLockStartTime = millis();
    ignoreFirstSetInMenu = true;
    display(MAIN_INTERFACE);
    display(PLL_LOCK_STATUS);
}

void handleStationNameEdit() {
    if (!stationNameEditMode) return;

    // select character
    auto editCharacter = [](int8_t direction) { applyStationNameEdit(true, direction); };
    handleButtonInput(buttonDownState, buttonDownPressed, -1, editCharacter);
    handleButtonInput(buttonUpState, buttonUpPressed, 1, editCharacter);

    // confirm selection
    handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t direction) {
        applyStationNameEdit(false, direction);
    });
}

void applyStationNameEdit(bool editCharacter, int8_t direction) {
    if (editCharacter) {
        // UP/DOWN action
        uint8_t range = asciiRange[1] - asciiRange[0] + 1;
        stationNameBuffer[editPosition] = (stationNameBuffer[editPosition] - asciiRange[0] + direction + range) % range + asciiRange[0];
    } else {
        // SET action
        unsigned long currentMillis = millis();
        if (editPosition >= maxNameLength - 1) { // finish station name edit when last character is confirmed
            stationNameEditMode = false; // exit station name editor
            editPosition = 0; // reset edit position after confirmation
            menuEditMode = false; // reset edit flag after editor exit
            menuExitConfirmMode = false; // reset exit submenu state
            stationNameBuffer[maxNameLength] = '\0'; // ensure null terminator in edit buffer
            strncpy(stationName, stationNameBuffer, maxNameLength); // update working copy for menu re-entry
            stationName[maxNameLength] = '\0'; // ensure null terminator in working copy
            display(MENU_INTERFACE); // show menu again
            menuLockStartTime = currentMillis; // block menu entry immediately after exit
            ignoreFirstSetInMenu = true; // wait for SET release before accepting new input in menu
            return; // prevent further display call
        } else {
            editPosition++; // move to next character
        }
    }
    display(STATION_NAME_EDITOR);
}

void handleButtonInput(bool buttonState, bool& buttonPressed, int8_t direction, void (*action)(int8_t)) {
    static unsigned long pressStartTime = 0, lastActionTime = 0;
    unsigned long currentMillis = millis();
    unsigned long totalPressTime = currentMillis - pressStartTime;
    unsigned long fastPressInterval = baseRepeatInterval;

    if (buttonState) {
        // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed) {
            pressStartTime = currentMillis;
            lastActionTime = currentMillis;
            buttonPressed = true;
            action(direction);
        } else {
            // adaptive gradual acceleration in frequency SET mode
            if (freqSetMode && totalPressTime >= initialPressDelay) {
                fastPressInterval =
                    (getStepSizeMultiplier() * getVCOFreqStep() / pressTargetSweepSpeed) /
                    (pressAccelerationBase + ((float)(totalPressTime - initialPressDelay) / initialPressDelay));
            }
            // auto-repeat action after initialPressDelay
            if (totalPressTime >= initialPressDelay &&
                currentMillis - lastActionTime >= fastPressInterval &&
                (!menuMode || stationNameEditMode)) {
                // fixed scroll speed in station name editor
                if (!stationNameEditMode || currentMillis - lastActionTime >= charScrollInterval) {
                    lastActionTime = currentMillis;
                    action(direction);
                }
            }
        }
    } else {
        buttonPressed = false; // reset on button release
    }
}

void configurePLL() {
    targetFreq = validateFreq(targetFreq, true); // align target frequency to current visible step size before programming PLL
    if (targetFreq == currentFreq && !menuMode) return;
    unsigned long divisor = (targetFreq / getVCOFreqStep()); // calculate divisor
    byte data[4]; // full programming (TSA5511 datasheet, table 1)
    data[0] = (divisor & 0xFF00) >> 8; // extract high divisor byte
    data[1] = divisor & 0x00FF; // extract low divisor byte
    data[2] = PLL_BYTE4_BASE // set charge pump high during acquisition, unless set to disabled
        | ((cpMode != CP_DISABLED) ? PLL_BYTE4_CP : 0)
        | ((cpMode == CP_DISABLED) ? PLL_BYTE4_OS : 0);
    data[3] = isValidPLLPort(portIdxUnlock) ? _BV(portIdxUnlock) : 0; // assert unlocked if assigned; otherwise all ports low
    if (attemptI2C(false, PLL_ADDRESSES[pllAddrIndex], data, 4)) {
        storeBandFrequency(freqBandIndex[xtalFreqIndex], xtalFreqIndex, targetFreq);
        currentFreq = targetFreq;
        pllCheckPending = true;
    }
}

void checkPLL() {
    if (!pllCheckPending) return;
    byte readByte;
    if (attemptI2C(true, PLL_ADDRESSES[pllAddrIndex], &readByte, 1)) {
        pllLock = (readByte & PLL_BYTE1_FL) != 0;
        if (!menuMode) {
            display(PLL_LOCK_STATUS);
        }

        // update output port bitmap for current lock state
        if (pllLock) {
            if (updateLockedOutputPorts()) {
                pllCheckPending = false; // stop polling after lock, as lock flag may flicker due to FM modulation
            }
        }
        digitalWrite(lockIndicator, pllLock ? HIGH : LOW);
    }
}

// store system settings
    void storeBandIndex() {
        for (uint8_t i = 0; i < numXTALFreqOptions; i++) {
            EEPROM.update(EEPROM_FREQBAND_ADDR + i, freqBandIndex[i]);
        }
    }

    void storeNumDecimals() {
        EEPROM.update(EEPROM_DECIMAL_ADDR, numDecimals);
    }

    void storeBandFrequency(byte bandIndex, byte xtalIndex, unsigned long frequency) {
        if (bandIndex < numFreqBands && xtalIndex < numXTALFreqOptions && frequency != currentFreq) { // avoid unnecessary EEPROM access
            EEPROM.put(EEPROM_BAND_FREQ_BASE_ADDR + ((bandIndex * numXTALFreqOptions + xtalIndex) * sizeof(unsigned long)), frequency); // calculate EEPROM address from band and XTAL index
        }
    }

    void storeI2CAddress() {
        EEPROM.update(EEPROM_I2C_ADDR, pllAddrIndex);
    }

    void storeXTALFreq() {
        EEPROM.update(EEPROM_XTAL_ADDR, xtalFreqIndex);
    }

    void storeChargePump() {
        EEPROM.update(EEPROM_CP_ADDR, cpMode);
    }

    void storeOutputPorts() {
        auto enc = [](uint8_t p) { return isValidPLLPort(p) ? p : PLL_PORT_NONE; }; // encode port index into storage format or 'none'
        EEPROM.update(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_LOCKED, enc(portIdxLock));
        EEPROM.update(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_UNLOCKED, enc(portIdxUnlock));
        EEPROM.update(EEPROM_OUTPUT_PORTS_ADDR + OUTPUT_PORT_PHASE_RF_DRIVE, enc(portIdxRF));
    }

    void storeDimmerStatus() {
        EEPROM.update(EEPROM_DIM_ADDR, backlightDimActive);
    }

    void storeShowMenuTitleStatus() {
        EEPROM.update(EEPROM_SHOW_MENU_TITLE_ADDR, showMenuTitle);
    }

    void storeStationName() {
        stationName[maxNameLength] = '\0'; // ensure null terminator

        // validate characters before storing
        bool valid = true;
        for (uint8_t i = 0; i < maxNameLength; i++) {
            char c = stationName[i];
            if (c < asciiRange[0] || c > asciiRange[1] || c == EEPROM_ERASED_BYTE) {
                valid = false;
                break;
            }
        }

        // fallback to default name if invalid
        if (!valid) {
            strncpy(stationName, defaultName, maxNameLength); // copy default station name
            memset(stationName + strlen(defaultName), ' ', maxNameLength - strlen(defaultName)); // pad with spaces
            stationName[maxNameLength] = '\0';
        }
        char storedStationName[maxNameLength + 1]; // +1 for null terminator
        EEPROM.get(EEPROM_NAME_ADDR, storedStationName);

        // avoid unnecessary EEPROM access
        if (strncmp(stationName, storedStationName, maxNameLength + 1) != 0) {
            EEPROM.put(EEPROM_NAME_ADDR, stationName);
        }
    }

// user memory EEPROM access
    uint16_t getUserMemoryAddress(byte slotIndex) {
        uint16_t bankIndex = freqBandIndex[xtalFreqIndex] * numXTALFreqOptions + xtalFreqIndex;
        return EEPROM_USER_MEMORY_BASE_ADDR + ((bankIndex * numUserMemorySlots + slotIndex) * sizeof(unsigned long));
    }

    bool readUserMemoryFrequency(byte slotIndex, unsigned long& frequency) {
        if (slotIndex >= numUserMemorySlots) return false;

        EEPROM.get(getUserMemoryAddress(slotIndex), frequency);
        return frequency != USER_MEMORY_EMPTY_FREQ && frequency >= lowerFreq && frequency <= upperFreq;
    }

    void storeUserMemoryFrequency(byte slotIndex) {
        if (slotIndex >= numUserMemorySlots) return;

        EEPROM.put(getUserMemoryAddress(slotIndex), currentFreq);
    }

    void clearUserMemoryFrequency(byte slotIndex) {
        if (slotIndex >= numUserMemorySlots) return;

        uint16_t startAddr = getUserMemoryAddress(slotIndex);
        for (uint8_t i = 0; i < sizeof(unsigned long); i++) {
            EEPROM.update(startAddr + i, EEPROM_ERASED_BYTE);
        }
    }

void checkI2C() {
    static unsigned long lastI2cCheckTime = 0;
    if (menuMode) return; // no interference allowed with menu session

    unsigned long currentMillis = millis();

    if (currentMillis - lastI2cCheckTime >= i2cHealthCheckInterval) {
        lastI2cCheckTime = currentMillis;
        attemptI2C(false, PLL_ADDRESSES[pllAddrIndex], nullptr, 0); // I²C bus is operational if transmission succeeds
    }
}

bool attemptI2C(bool isRead, byte address, byte* buffer, byte length) {
    for (uint8_t i = i2cMaxRetries; i > 0; i--) {
        if (isRead) {
            if (Wire.requestFrom(address, length) == length) {
                for (byte j = 0; j < length; j++) buffer[j] = Wire.read();
                return true;
            }
        } else {
            Wire.beginTransmission(address);
            if (length > 0) Wire.write(buffer, length);
            if (Wire.endTransmission() == 0) return true;
        }
        delay(i2cRetryDelay);
    }
    if (!menuMode) {
        handleI2CError();
    }
    return false;
}

void handleI2CError() {
    digitalWrite(lockIndicator, LOW);
    display(I2C_ERROR);
    while (true) {
        while (!digitalRead(setButton)) { blinkLed(errIndicator, errBlinkRate); } // ensure that SET is released, to prevent premature reset
        do { blinkLed(errIndicator, errBlinkRate); } while (digitalRead(setButton)); // initiate system reset after confirmation with SET
        while (!digitalRead(setButton)) blinkLed(errIndicator, errBlinkRate); // continue blinking error indicator while SET is pressed
        digitalWrite(errIndicator, LOW); // reset error indicator
        wdt_enable(WDTO_15MS); // reset system via watchdog
        while (true) {} // wait for watchdog to expire and reset system
    }
}

void blinkLed(uint8_t ledPin, unsigned long interval) {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;
    unsigned long currentMillis = millis();

    if (currentMillis - lastBlinkTime >= interval) {
        lastBlinkTime = currentMillis;
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
    }
}

void display(uint8_t mode) {
    // format right-aligned VCO frequency with optional prefix/suffix and dynamic decimal precision
    auto printFreq = [](uint8_t row, unsigned long frequency, const __FlashStringHelper* prefix, const char* suffix) {
        char buffer[freqDisplayWidth + 1]; // buffer for formatted frequency string and null terminator
        dtostrf((double)frequency / hzPerMHz, freqDisplayWidth, numDecimals, buffer); // format VCO frequency as right-aligned string in MHz
        uint8_t col = lcdColumns - (strlen(buffer) + strlen_P(suffix)); // compute starting column for right-alignment
        if (prefix) {
            lcd.setCursor(0, row);
            lcd.print(prefix);
        }
        lcd.setCursor(col, row);
        lcd.print(buffer);
        lcd.print((const __FlashStringHelper*)suffix);
    };

    // build a fixed-width LCD line from a flash string
    auto buildLine = [](char* line, const char* text, bool centered) {
        memset(line, ' ', lcdColumns);
        line[lcdColumns] = '\0';
        uint8_t len = strlen_P(text);
        if (len > lcdColumns) len = lcdColumns;
        uint8_t col = centered ? (lcdColumns - len) / 2 : 0;
        memcpy_P(line + col, text, len);
    };

    // print a complete prebuilt LCD line without relying on residual characters
    auto printLine = [](uint8_t row, const char* line) {
        lcd.setCursor(0, row);
        lcd.print(line);
    };

    // clear a complete LCD line in a single print operation
    auto clearLine = [&](uint8_t row) {
        char line[lcdColumns + 1];
        memset(line, ' ', lcdColumns);
        line[lcdColumns] = '\0';
        printLine(row, line);
    };

    // print the submenu return option
    auto printReturnMenuItem = []() {
        lcd.print(F("RETURN TO MENU"));
        lcd.setCursor(0, 1);
        lcd.print(F("SET to confirm"));
    };

    // animate the transition from the menu entry title to the first menu item
    auto animateMenuEntryTitle = [&](const char* titleText, const char* targetText) {
        char titleLine[lcdColumns + 1];
        char targetLine[lcdColumns + 1];
        char frameLine[lcdColumns + 1];

        buildLine(titleLine, titleText, true);
        buildLine(targetLine, targetText, false);
        lcd.clear();
        printLine(0, titleLine);
        delay(menuEntryTitleDelay);
        for (uint8_t step = 1; step <= (lcdColumns / 2) + menuEntryGapWidth; step++) {
            memcpy(frameLine, titleLine, lcdColumns + 1);
            uint8_t eraseCount = step;
            if (eraseCount > lcdColumns / 2) eraseCount = lcdColumns / 2;
            for (uint8_t i = 0; i < eraseCount; i++) {
                frameLine[i] = ' ';
                frameLine[lcdColumns - 1 - i] = ' ';
            }
            if (step > menuEntryGapWidth) {
                uint8_t revealCount = step - menuEntryGapWidth;
                if (revealCount > lcdColumns / 2) revealCount = lcdColumns / 2;
                for (uint8_t i = 0; i < revealCount; i++) {
                    frameLine[i] = targetLine[i];
                    frameLine[lcdColumns - 1 - i] = targetLine[lcdColumns - 1 - i];
                }
            }
            printLine(0, frameLine);
            delay(menuEntryAnimDelay);
        }
    };

    // cursor only active in station name editor
    if (mode != STATION_NAME_EDITOR) lcd.noCursor();

    // display modes
    switch(mode) {
        case SPLASH_SCREEN:
            lcd.clear();
            lcd.print(F(description));
            lcd.print(F(" "));
            lcd.print(F(version));
            lcd.setCursor(0, 1);
            lcd.print(F(credits));
            break;

        case MAIN_INTERFACE:
            printFreq(0, targetFreq, nullptr, PSTR(" MHz"));
            lcd.setCursor(0, 1);
            if (!rfOutputEnabled && ((millis() / rfOutputStatusInterval) % 2)) {
                clearLine(1); // clear row before showing the centered RF drive override status
                lcd.setCursor((lcdColumns - (sizeof("RF DRIVE: OFF") - 1)) / 2, 1);
                lcd.print(F("RF DRIVE: OFF"));
            } else {
                lcd.print(stationName);
            }
            break;

        case SET_FREQUENCY_INTERFACE:
            if (!freqSetMode) { // avoid time-consuming repeated LCD instructions during auto-repeat VCO frequency change
                lcd.setCursor(0, 1);
                lcd.print(F("SET "));
            }
            printFreq(1, targetFreq, nullptr, PSTR(" MHz"));
            break;

        case MENU_INTERFACE:
            lcd.clear();
            if (menuExitConfirmMode) {
                lcd.setCursor(0, 0);
                lcd.print(F("EXIT SETTINGS"));
                lcd.setCursor(0, 1);
                lcd.print(F("> "));
                switch (menuIndex) {
                    case EXIT_MENU_SAVE_CHANGES: lcd.print(F("save changes")); break;
                    case EXIT_MENU_DISCARD: lcd.print(F("discard")); break;
                    case EXIT_MENU_CANCEL: lcd.print(F("cancel")); break;
                }
                break;
            }
            if (menuLevel == MENU_LEVEL_QUICK_MENU) {
                switch (menuIndex) {
                    case USER_MENU_RECALL_MEMORY: lcd.print(F("RECALL MEMORY")); break;
                    case USER_MENU_STORE_MEMORY: lcd.print(F("SAVE MEMORY")); break;
                    case USER_MENU_CLEAR_MEMORY: lcd.print(F("CLEAR MEMORY")); break;
                    case USER_MENU_RF_DRIVE:
                        lcd.print(F("RF DRIVE: "));
                        lcd.print(rfOutputEnabled ? F("ON") : F("OFF"));
                        break;
                    case USER_MENU_LCD_OFF: lcd.print(F("LCD OFF")); break;
                    case USER_MENU_EXIT: lcd.print(F("EXIT QUICK MENU")); break;
                }
                lcd.setCursor(0, 1);
                if (menuIndex == USER_MENU_RF_DRIVE) {
                    lcd.print(F("SET to toggle"));
                } else if (menuIndex == USER_MENU_EXIT || menuIndex == USER_MENU_LCD_OFF) {
                    lcd.print(F("SET to confirm"));
                } else {
                    lcd.print(F("SET to select"));
                }
            } else if (menuLevel == MENU_LEVEL_USER_MEMORY) {
                switch (userMemoryAction) {
                    case USER_MENU_RECALL_MEMORY: lcd.print(F("RECALL MEMORY")); break;
                    case USER_MENU_STORE_MEMORY: printFreq(0, currentFreq, F("SAVE "), PSTR(" MHz")); break;
                    case USER_MENU_CLEAR_MEMORY: lcd.print(F("CLEAR MEMORY")); break;
                }
                lcd.setCursor(0, 1);
                if (menuIndex == numUserMemorySlots) {
                    lcd.print(F("> return"));
                } else {
                    unsigned long memoryFreq = 0;
                    bool hasMemory = readUserMemoryFrequency(menuIndex, memoryFreq);
                    if (hasMemory) {
                        memoryFreq = validateFreq(memoryFreq, true);

                        const char* suffix = PSTR(" MHz");
                        char buffer[freqDisplayWidth + 1]; // buffer for formatted frequency string and null terminator
                        dtostrf((double)memoryFreq / hzPerMHz, 0, numDecimals, buffer); // format VCO frequency as variable-width string in MHz

                        uint8_t valueLength = strlen(buffer) + strlen_P(suffix);
                        uint8_t spacerCol = (valueLength < lcdColumns) ? lcdColumns - valueLength - 1 : 0; // reserve one leading space before the frequency

                        lcd.print(spacerCol < (sizeof("> M1:") - 1) ? F(">M") : F("> M"));
                        lcd.print(menuIndex + 1);
                        lcd.print(F(":"));
                        lcd.setCursor(spacerCol, 1);
                        lcd.print(F(" "));
                        lcd.print(buffer);
                        lcd.print((const __FlashStringHelper*)suffix);
                    } else {
                        lcd.print(F("> M"));
                        lcd.print(menuIndex + 1);
                        lcd.print(F(":"));
                        lcd.setCursor(lcdColumns - (sizeof("<empty>") - 1), 1);
                        lcd.print(F("<empty>"));
                    }
                }
            } else if (menuLevel == MENU_LEVEL_MAIN) {
                switch (menuIndex) {
                    case SYSTEM_MENU_VCO_SETTINGS:
                        lcd.print(F("VCO SETTINGS"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("SET to enter"));
                        break;
                    case SYSTEM_MENU_PLL_SETTINGS:
                        lcd.print(F("PLL SETTINGS"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("SET to enter"));
                        break;
                    case SYSTEM_MENU_GENERAL_SETTINGS:
                        lcd.print(F("GENERAL SETTINGS"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("SET to enter"));
                        break;
                    case SYSTEM_MENU_EXIT:
                        lcd.print(F("EXIT SETTINGS"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("SET to proceed"));
                        break;
                }
            } else if (menuLevel == MENU_LEVEL_VCO_SETTINGS) {
                switch (menuIndex) {
                    case VCO_MENU_FREQUENCY_BAND:
                        lcd.print(F("FREQUENCY BAND"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            unsigned long low = validateFreq(freqBands[freqBandIndex[xtalFreqIndex]][0], true, 1);
                            unsigned long high = validateFreq(freqBands[freqBandIndex[xtalFreqIndex]][1], true, -1);
                            lcd.print(F("> "));
                            lcd.print(low / hzPerMHz);
                            lcd.print(F("-"));
                            lcd.print(high / hzPerMHz);
                            lcd.print(F(" MHz"));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case VCO_MENU_PRECISION:
                        lcd.print(F("PRECISION"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(numDecimals);
                            lcd.print(numDecimals == 1 ? F(" decimal") : F(" decimals"));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case VCO_MENU_EXIT:
                        printReturnMenuItem();
                        break;
                }
            } else if (menuLevel == MENU_LEVEL_PLL_SETTINGS) {
                switch (menuIndex) {
                    case PLL_MENU_I2C_ADDRESS:
                        lcd.print(F("I2C ADDRESS"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> 0x"));
                            lcd.print(PLL_ADDRESSES[tempPllAddrIndex], HEX);
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case PLL_MENU_XTAL_FREQUENCY:
                        lcd.print(F("XTAL FREQUENCY"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(PLL_XTAL_OPTIONS[xtalFreqIndex] / hzPerMHz);
                            lcd.print(F("."));
                            lcd.print((PLL_XTAL_OPTIONS[xtalFreqIndex] % hzPerMHz) / (hzPerMHz / 10));
                            lcd.print(F(" MHz"));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case PLL_MENU_CHARGE_PUMP:
                        lcd.print(F("CHARGE PUMP"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(cpMode == CP_LOW ? F("low ") : (cpMode == CP_HIGH ? F("high") : F("disabled")));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case PLL_MENU_PORT_MAPPING:
                        lcd.print(F("PORT MAPPING"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            if (outputPortsEditPhase == OUTPUT_PORT_PHASE_LOCKED) {
                                lcd.print(F("locked: "));
                                if (isValidPLLPort(portIdxLock)) {
                                    lcd.print(F("P"));
                                    lcd.print(portIdxLock);
                                } else {
                                    lcd.print(F("none"));
                                }
                            } else if (outputPortsEditPhase == OUTPUT_PORT_PHASE_UNLOCKED) {
                                lcd.print(F("unlocked: "));
                                if (isValidPLLPort(portIdxUnlock)) {
                                    lcd.print(F("P"));
                                    lcd.print(portIdxUnlock);
                                } else {
                                    lcd.print(F("none"));
                                }
                            } else {
                                lcd.print(F("RF drive: "));
                                if (isValidPLLPort(portIdxRF)) {
                                    lcd.print(F("P"));
                                    lcd.print(portIdxRF);
                                } else {
                                    lcd.print(F("none"));
                                }
                            }
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case PLL_MENU_EXIT:
                        printReturnMenuItem();
                        break;
                }
            } else if (menuLevel == MENU_LEVEL_GENERAL_SETTINGS) {
                switch (menuIndex) {
                    case GENERAL_MENU_STATION_NAME:
                        lcd.print(F("STATION NAME"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("SET to edit"));
                        break;
                    case GENERAL_MENU_BACKLIGHT_DIMMER:
                        lcd.print(F("BACKLIGHT DIMMER"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(backlightDimActive ? F("on ") : F("off"));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case GENERAL_MENU_SHOW_MENU_TITLE:
                        lcd.print(F("SHOW MENU TITLE"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(showMenuTitle ? F("on ") : F("off"));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case GENERAL_MENU_FACTORY_RESET:
                        lcd.print(factoryResetFinalConfirm ? F("CLEAR ALL DATA?") : F("FACTORY RESET"));
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print(F("> "));
                            lcd.print(factoryResetConfirm ? F("yes") : F("no "));
                        } else {
                            lcd.print(F("SET to select"));
                        }
                        break;
                    case GENERAL_MENU_EXIT:
                        printReturnMenuItem();
                        break;
                }
            }
            break;

        case STATION_NAME_EDITOR:
            lcd.setCursor(0, 1);
            lcd.print(stationNameBuffer);
            lcd.setCursor(editPosition, 1);
            lcd.cursor();
            break;

        case PLL_LOCK_STATUS:
            static unsigned long lastCharScrollTime = 0;
            static uint8_t charPos = 0;
            static bool movingRight = true;

            lcd.setCursor(0, 0);
            if (pllLock) {
                lcd.print(F("LOCK "));
            } else {
                // animation if unlocked
                unsigned long currentMillis = millis();

                if (currentMillis - lastCharScrollTime >= animInterval) {
                    lastCharScrollTime = currentMillis;
                    for (uint8_t i = 0; i < lockStatusWidth; i++) lcd.print(F(" "));
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? F(">>") : F("<<"));
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 0 || charPos == lockStatusWidth - lockStatusAnimWidth) movingRight = !movingRight;
                }
            }
            break;

        case MENU_ENTRY_TITLE:
            if (showMenuTitle) {
                animateMenuEntryTitle(
                    menuLevel >= MENU_LEVEL_QUICK_MENU ? PSTR("QUICK MENU") : PSTR("SYSTEM SETTINGS"),
                    menuLevel >= MENU_LEVEL_QUICK_MENU ? PSTR("RECALL MEMORY") : PSTR("VCO SETTINGS")
                );
            }
            break;

        case LCD_HIBERNATE:
            backlightOff ? lcd.clear() : (display(MAIN_INTERFACE), display(PLL_LOCK_STATUS));
            break;

        case I2C_ERROR:
            analogWrite(lcdBacklight, maxBrightness);
            lcd.clear();
            lcd.print(i2cFallbackActive ? F("I2C FALLBACK") : F("I2C ERROR"));
            lcd.setCursor(0, 1);
            if (i2cFallbackActive) {
                lcd.print(F("restoring 0x"));
                lcd.print(PLL_ADDRESSES[pllAddrIndex], HEX);
            } else {
                lcd.print(F("SET to restart"));
            }
            break;
    }
}
