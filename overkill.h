#define MIDI_USB 0
#define MIDI_EXT 1

#define SENDMIDI_USB(x) Midi.SendData(x)
#define SENDMIDI_MIDI(x,y) Serial2.write(x,y)

#define DEBUG(s) Serial1.print((char*)s)

#define PIN_RX0 0        // programmer (UART0)
#define PIN_TX0 1        // programmer (UART0)

#define PIN_D2  2 // unused
#define PIN_D3  3 // unused
#define PIN_D4  4 // unused

#define PIN_D5    5      // unused
#define PIN_D6    6      // unused
#define PIN_D7    7      // unused
#define PIN_D8    8      // unused

#define PIN_USB_SHIELD_INT 9   // USB host shield
#define PIN_USB_SHIELD_SS  10  // USB host shield

#define PIN_D11    11    // unused
#define PIN_CLKOUT 12    // CLK OUT

#define PIN_LED 13       // LED

#define PIN_TX3 14       // MIDI OUT 2 (UART 3)
#define PIN_RX3 15       // unused (UART 3)
#define PIN_TX2 16       // MIDI OUT 1 (UART 2)
#define PIN_RX2 17       // MIDI IN (UART 2)
#define PIN_TX1 18       // USB serial monitor; INTERRUPT 5
#define PIN_RX1 19       // USB serial monitor; INTERRUPT 4

#define PIN_CLKIN    20  // nominally TWI_SDA/INTERRUPT 3
#define PIN_D21      21  // nominally TWI_SCL/INTERRUPT 2

#define PIN_TRIGGER0 22  // trigger output
#define PIN_TRIGGER1 23  //
#define PIN_TRIGGER2 24  //
#define PIN_TRIGGER3 25  //
#define PIN_TRIGGER4 26  //
#define PIN_TRIGGER5 27  //
#define PIN_TRIGGER6 28  //
#define PIN_TRIGGER7 29  //

#define PIN_LCD_RS 30    // LCD control
#define PIN_LCD_EN 31    //
#define PIN_LCD_D4 32    //
#define PIN_LCD_D5 33    //
#define PIN_LCD_D6 34    //
#define PIN_LCD_D7 35    //

#define PIN_SPI_MICROSD 36 // microSD select 

#define PIN_GATE0     37 // analog gate output
#define PIN_SPI_CV_X  38 // sequenced CV X (pitch)
#define PIN_SPI_CV_Y  39 // sequenced CV Y (filter)
#define PIN_SPI_CV_Z  40 // sequenced CV Z (resonance)

#define PIN_SPI_CV0   41 // SPI CV0..7
#define PIN_SPI_CV1   42 //
#define PIN_SPI_CV2   43 //
#define PIN_SPI_CV3   44 //
#define PIN_SPI_CV4   45 //
#define PIN_SPI_CV5   46 //
#define PIN_SPI_CV6   47 //
#define PIN_SPI_CV7   48 //

#define PIN_D49 49

#define PIN_MEGA_MISO 50 // SPI control pins
#define PIN_MEGA_MOSI 51 //
#define PIN_MEGA_SCK  52 //

#define PIN_USB_SHIELD 53 // USB shield; nominally SPI SS signal (USB shield).  Apparently must be left alone for all SPI.

#define NUM_TRACKS          8
#define NUM_STEPS          16
#define NUM_TRIGGERS        8
#define NUM_MIDI_TRACKS     0

#ifdef REMOVE_THIS
#define TET_USB_NOTE_OFF      0
#define TET_MIDI_NOTE_OFF     1
#define TET_ANALOG_LOW        2
#endif

// Channel Voice messages
#define MIDI_NOTE_OFF 0x80    // key, vel
#define MIDI_NOTE_ON  0x90    // key, vel
#define MIDI_PKP      0xA0    // key, pressure
#define MIDI_CC       0xB0    // cc, val (0-77 = ctrl, 78+ channel mode)
#define MIDI_PC       0xC0    // program
#define MIDI_AT       0xD0    // aftertouch
#define MIDI_PB       0xE0    // LSB, MSB

#define MIDI_LOCAL   0x7A

// System Common (alter running status)
#define MIDI_SYSEX        0xF0
#define MIDI_TIME_CODE    0xF1  // val
#define MIDI_SPP          0xF2  // LSB, MSB
#define MIDI_SONG_SELECT  0xF3  // val
#define MIDI_TUNE_REQUEST 0xF6  // <none>
#define MIDI_EOX          0xF7  // <none>

// System Real Time (do not affect running status)
#define MIDI_CLOCK  0xF8
#define MIDI_START  0xFA
#define MIDI_CONT   0xFB
#define MIDI_STOP   0xFC
#define MIDI_ACTIVE 0xFE
#define MIDI_RESET  0xFF

// Control mappings
#define OVERKILL_START_BUTTON        ( LIVID_F1 )
#define OVERKILL_RESET_BUTTON        ( LIVID_F2 )
#define OVERKILL_BPM_BUTTON          ( LIVID_F3 )
#define OVERKILL_PATTERN_COPY        ( LIVID_F4 )
#define OVERKILL_PATTERN_PASTE       ( LIVID_F5 )
#define OVERKILL_TRACK_MUTE_BUTTON   ( LIVID_F8 )

#define OVERKILL_MAGIC_COOKIE      0x07345177
#define OVERKILL_STATE_VERSION     0x0001

typedef enum 
{ 
    CLOCK_MENU_INTERNAL, CLOCK_MENU_EXTERNAL
} ClockSource_t;
