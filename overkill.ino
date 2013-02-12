// TODO: trigger outputs
// TODO: gate length control
// TODO: pattern select
// TODO: analog clock input
// TODO: MIDI IN (for clocking)?
// TODO: CV outputs (manual)
// TODO: CV outputs (sequenced)
// TODO: SD card
// TODO: LCD support
#include <avr/pgmspace.h>
#include "livid.h"

#define DOUSB 1

#ifdef DOUSB
#include <Usb.h>
#include <usbh_midi.h>
#endif

char dbg_buffer[ 128 ];

#define MIDI_USB 0
#define MIDI_EXT 1

#ifdef DOUSB
#define SENDMIDI_USB(x) Midi.SendData(x)
#define SENDMIDI_MIDI(x,y) Serial2.write(x,y)
#else
#define SENDMIDI(x) Serial.write(x)
#endif

#define DEBUG(s) Serial1.print((char*)s)

#define PIN_RX0 0   // programmer
#define PIN_TX0 1   // programmer
#define PIN_D2  2   // unused
#define PIN_D3  3   // unused
#define PIN_D4  4   // unused

#define PIN_D5    5   // unused
#define PIN_D6    6   // unused
#define PIN_D7    7   // unused

#define PIN_D8    8   // START SEQ
#define PIN_D9    9   // STOP SEQ
#define PIN_D10  10   // RESET SEQ
#define PIN_D11  11   // CLK IN
#define PIN_D12  12   // CLK OUT

#define PIN_LED 13    // LED

#define PIN_TX3 14
#define PIN_RX3 15
#define PIN_TX2 16   // MIDI OUT
#define PIN_RX2 17
#define PIN_TX1 18   // USB serial monitor; INTERRUPT 5
#define PIN_RX1 19   // USB serial monitor; INTERRUPT 4

#define PIN_TWI_SDA 20 // currently unused; INTERRUPT 3
#define PIN_TWI_SCL 21 // currently unused; INTERRUPT 2

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

#define PIN_D36   36     // unused
#define PIN_D37   37     // unused

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

#define PIN_D49       49 // unused

#define PIN_MEGA_MISO 50 // SPI control pins
#define PIN_MEGA_MOSI 51 //
#define PIN_MEGA_SCK  52 //
#define PIN_MEGA_SS   53 // (unused, PIN_SPI_CV0..CV7 are the SS)

#define NUM_TRACKS         12
#define NUM_STEPS          16
#define NUM_TRIGGERS        8
#define NUM_REALTIME_CV     8
#define NUM_SEQUENCED_CV    3

#define NUM_ENCODERS       12

#define TET_USB_NOTE_OFF      0
#define TET_MIDI_NOTE_OFF     1
#define TET_ANALOG_LOW        2

#define MAX_TIMER_EVENTS 32

struct MnTimerEvent
{
    uint8_t       te_type;
    unsigned long te_trigger_time_ms;
    uint8_t       te_param0;
    uint8_t       te_param1;
};

struct MnBitSet16
{
    uint16_t bs16_values;

    bool isBitSet( uint8_t const kWhich )
    {
        uint8_t const kBitVal  = kWhich & 0x0F;

        return ( ( bs16_values >> kBitVal ) & 1 ) != 0;
    }

    void setBit( uint8_t const kWhich, bool const kOn )
    {
        uint8_t const kBitVal  = kWhich & 0x0F;

        if ( kOn )
        {
            bs16_values |= 1 << kBitVal;
        }
        else
        {
            bs16_values &= ~( 1 << kBitVal );
        }
    }
};

struct MnTrack
{
    bool            mnt_track_muted;
    uint8_t         mnt_step_notes[ NUM_STEPS ];

    MnBitSet16      mnt_step_enabled;
    MnBitSet16      mnt_step_accent;

    bool isStepEnabled( uint8_t const kStep )
    {
        return mnt_step_enabled.isBitSet( kStep );
    }
    void enableStep( uint8_t const kStep, bool const kEnable )
    {
        mnt_step_enabled.setBit( kStep, kEnable );
    }
    bool isStepAccent( uint8_t const kStep )
    {
        return mnt_step_accent.isBitSet( kStep );
    }
    void accentuateStep( uint8_t const kStep, bool const kAccent )
    {
        return mnt_step_accent.setBit( kStep, kAccent );
    }
};

#define MODE_CONTROL 0
#define MODE_TRACK   1
#define MODE_MUTE    2
#define MODE_PITCH   3

struct MnState
{
    MnTimerEvent  mns_timer_events[ MAX_TIMER_EVENTS ];
    uint8_t       mns_button_state[ 8 ]; // Assumes up to 64 buttons (CNTRL:R has almost that many)

    uint8_t getButtonState( uint8_t const kControlNumber )
    {
        uint8_t const kByteVal = kControlNumber >> 3;
        uint8_t const kBitVal  = kControlNumber & 7;

        if ( kByteVal >= sizeof( mns_button_state ) )
        {
            return 0;
        }

        return ( mns_button_state[ kByteVal ] >> kBitVal ) & 1;
    }
    void setButtonState( uint8_t const kControlNumber, bool const kOn )
    {
        uint8_t const kByteVal = kControlNumber >> 3;
        uint8_t const kBitVal  = kControlNumber & 7;

        if ( kByteVal >= sizeof( mns_button_state ) )
        {
            return;
        }

        if ( kOn )
        {
            mns_button_state[ kByteVal ] |= ( 1 << kBitVal );
        }
        else
        {
            mns_button_state[ kByteVal ] &= ~( 1 << kBitVal );
        }
    }

    uint8_t       mns_mode;
    uint8_t       mns_bpm;
    uint8_t       mns_active_track;

    int8_t        mns_edit_step; // Step currently being edited, -1 if none

    unsigned int  mns_beat;
    unsigned long mns_last_beat_time;

    bool          mns_running;

    uint8_t       mns_led_button_backbuffer[ 60 ];   // These are what we update
    uint8_t       mns_led_button_frontbuffer[ 60 ];  // These are what we're currently showing
    uint8_t       mns_led_encoder_backbuffer[ NUM_ENCODERS ];  // These are what we update
    uint8_t       mns_led_encoder_frontbuffer[ NUM_ENCODERS ]; // These are what we're currently showing

    MnTrack       mns_tracks[ NUM_TRACKS ];

    uint8_t       getStep( void ) { return mns_beat & ( NUM_STEPS - 1 ); }

    bool trackHasAnyData( uint8_t const kTrack ) 
    {
        for ( int8_t i = 0; i < NUM_STEPS; i++ )
        {
            if ( getTrackNote( kTrack, i ) )
            {
                return true;
            }
        }
        return false;
    }

    // Returns value irrespective of mute
    uint8_t getTrackNote( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_notes[ kStep ];
    }
    void setTrackNote( uint8_t const kTrack, uint8_t const kStep, uint8_t const kValue )
    {
        mns_tracks [ kTrack ].mnt_step_notes[ kStep ] = kValue;
    }
    bool isTrackStepEnabled( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks[ kTrack ].isStepEnabled( kStep );
    }
    bool isTrackStepAudible( uint8_t const kTrack, uint8_t const kStep )
    {
        return isTrackStepEnabled( kTrack, kStep ) && getTrackNote( kTrack, kStep ) > 0;
    }


    // Toggles the beat
    void toggleBeat( uint8_t const kTrack, uint8_t const kStep )
    {
        bool const kEnabled = mns_tracks[ kTrack ].isStepEnabled( kStep );

        mns_tracks[ kTrack ].enableStep( kStep, !kEnabled );

        // Fill in with a default value if necessary
        // If this is a trigger track use '1'
        if ( getTrackNote( kTrack, kStep ) == 0 )
        {
            if ( kTrack < 8 )
            {
                setTrackNote( kTrack, kStep, 1 );
            }
            // Otherwise MIDI tracks use a default note value
            else
            {
                setTrackNote( kTrack, kStep, 0x30 );
            }
        }
    }

    // Set sane default track values
    MnState()
    {
        mns_edit_step = -1;
        mns_running = false;
    }
};

MnState g_state;

#ifdef DOUSB
USB   Usb;
MIDI  Midi(&Usb);
int   g_usb_connected;
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

unsigned char 
MIDI_numBytesForCMD( int cmd )
{
    switch( cmd & 0xF0 )
    {
    case MIDI_SPP:
    case MIDI_NOTE_OFF:
    case MIDI_NOTE_ON:
    case MIDI_PKP:
    case MIDI_CC:
    case MIDI_PB:
        return 2;
    case MIDI_TIME_CODE:
    case MIDI_SONG_SELECT:
    case MIDI_AT:
    case MIDI_PC:
        return 1;
    }
    return 0;
}

void FE_SetButtonLED( uint8_t const kWhich, uint8_t const kCC )
{
    g_state.mns_led_button_backbuffer[ kWhich ] = kCC;
}
void FE_SetEncoderLED( uint8_t const kWhich, uint8_t const kCC )
{
    g_state.mns_led_encoder_backbuffer[ kWhich-LIVID_ENCODER00 ] = kCC;
}

void FE_SwapBuffers()
{
    // Update buttons
    for ( uint8_t i = LIVID_PAD00; i <= LIVID_ENCODER23; i++ )
    {
        if ( g_state.mns_led_button_backbuffer[ i ] != g_state.mns_led_button_frontbuffer[ i ] )
        {
            uint8_t b = g_state.mns_led_button_backbuffer[ i ];

            g_state.mns_led_button_frontbuffer[ i ] = b;
            if ( b )
                MIDI_noteOn( i, b, MIDI_USB );
            else
                MIDI_noteOff( i, 0, MIDI_USB );
        }
    }

    // Update encoders
/*
    for ( uint8_t i = 0; i < NUM_ENCODERS; i++ )
    {
        if ( g_state.mns_led_encoder_backbuffer[ i ] != g_state.mns_led_encoder_frontbuffer[ i ] )
        {
            uint8_t b = g_state.mns_led_encoder_backbuffer[ i ];

            g_state.mns_led_encoder_frontbuffer[ i ] = b;
            MIDI_cc( i + LIVID_ENCODER00, b, MIDI_USB );
        }
    }
*/
}

void MIDI_noteOn( uint8_t n, uint8_t vel, uint8_t port )
{
    unsigned char buffer[3] =
    {
        MIDI_NOTE_ON, n, vel
    };

    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
/*
    sprintf( dbg_buffer, "Note on %d,%d,%ld\n", (int)n,(int)vel,micros() );
    DEBUG( dbg_buffer );
*/
    delayMicroseconds(100); // This is unfortunately but apparently necessary when using this USB library
}
void MIDI_cc( uint8_t n, uint8_t val, uint8_t port )
{
    unsigned char buffer[3] =
    {
        MIDI_CC, n, val
    };

    if ( port == MIDI_USB )
    {
        sprintf( dbg_buffer, "[%ld] CC 0x%x 0x%x\n", micros(), (int)n,(int)val );
        DEBUG( dbg_buffer );
        SENDMIDI_USB(buffer);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
    delayMicroseconds(100); // This is unfortunately but apparently necessary when using this USB library
}
void MIDI_noteOff( unsigned char n, unsigned char vel, uint8_t port )
{
    unsigned char buffer[3] =
    {
        MIDI_NOTE_OFF, n, vel
    };
    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
//    sprintf( dbg_buffer, "Note off %d,%d,%ld\n", (int)n,(int)vel,micros() );
//    DEBUG( dbg_buffer );
    delayMicroseconds(100); // This is unfortunately but apparently necessary when using this USB library
}

void FE_ClearAllLEDs()
{
    // Push encoders
    for ( int i = 48; i < 60; i++ )
    {
        MIDI_noteOn( i, 0x10, MIDI_USB );
    }

    // all buttons (including encoders)
    for ( int i = LIVID_PAD00; i <= LIVID_ENCODER23; i++ )
    {
        MIDI_noteOn( i, 0, MIDI_USB );
    }
}

void FE_SetActiveStep( unsigned char const kStep )
{
    FE_SetButtonLED( kStep + 16, 1 );
}

void MnAddTimerEvent( uint8_t const kType, uint8_t const kParam0, uint8_t const kParam1, uint32_t const kWhen )
{
    // Find an empty slot
    uint8_t free_slot = 0;
    for ( free_slot = 0; free_slot < MAX_TIMER_EVENTS; free_slot++ )
    {
        if ( g_state.mns_timer_events[ free_slot ].te_trigger_time_ms == 0 )
        {
            break;
        }
    }

    if ( free_slot == MAX_TIMER_EVENTS )
    {
        DEBUG("Out of timer events!\n" );
        return;
    }

    MnTimerEvent &e = g_state.mns_timer_events[ free_slot ];

    e.te_type = kType;
    e.te_trigger_time_ms = kWhen;
    e.te_param0 = kParam0;
    e.te_param1 = kParam1;
}

void MnLoadState()
{
    g_state.mns_bpm = 120;
/*
    g_state.mns_tracks[ 0 ].enableStep( 0, true );
    g_state.setTrackNote( 0, 0, 1 );
    g_state.mns_tracks[ 0 ].enableStep( 4, true );
    g_state.setTrackNote( 0, 4, 1 );
    g_state.mns_tracks[ 0 ].enableStep( 8, true );
    g_state.setTrackNote( 0, 8, 1 );
    g_state.mns_tracks[ 0 ].enableStep( 12, true );
    g_state.setTrackNote( 0, 12, 1 );
*/
}

void MnSaveState()
{
    // Save patterns to SD
}

void FE_Setup()
{
    FE_ClearAllLEDs();
}

#if !defined DOUSB
static
unsigned char
SerialRead()
{
    unsigned char cmd = Serial.read();

    return cmd;
}

int MnCheckMIDI( unsigned char msg[3] )
{
    int ndx = 0;

    if ( !Serial.available() )
        return 0;

    unsigned char cmd = SerialRead();

    if ( cmd == MIDI_SYSEX )
    {
        DEBUG("Sysex!\n" );
        unsigned char prev = 0;
        while ( 1 )
        {
            if ( Serial.available() )
            {
                unsigned char c = SerialRead();
                DEBUG(c);
                if ( c == MIDI_EOX )
                    break;
            }
        }
        if ( prev == 0x7F )
        {
            msg[ 0 ] = MIDI_SYSEX;
            msg[ 1 ] = 0x7F;
            return 2;
        }
        return 0;
    }

    unsigned char num_bytes_to_read = MIDI_numBytesForCMD( cmd );

    msg[ ndx ] = cmd;
    ndx++;

    if ( ( cmd & 0x80 ) == 0 )
    {
        if ( g_midi.mnm_running_status )
        {
            msg[ 0 ] = g_midi.mnm_running_status;
            msg[ 1 ] = cmd;
            ndx = 2;
            num_bytes_to_read = MIDI_numBytesForCMD( g_midi.mnm_running_status ) - 1;
        }
    }

    while ( num_bytes_to_read )
    {
        while ( !Serial.available() )
            ;

        msg[ ndx ] = SerialRead();
        ndx++;
        num_bytes_to_read--;
    }

    if ( msg[ 0 ] >= 0x80 && msg[ 0 ] < 0xF0 )
    {
        g_midi.mnm_running_status = msg[ 0 ];
    }
    else if ( msg[ 0 ] >= 0xF0 && msg[ 0 ] < 0xF8 )
    {
        g_midi.mnm_running_status = 0;
    }

    // Note On with velocity = 0 maps to a Note Off
    if ( ( msg[ 0 ] & 0xF0 ) == MIDI_NOTEON )
    {
        if ( msg[ 2 ] == 0 )
        {
            msg[ 0 ] = MIDI_NOTEOFF | ( msg[ 0 ] & 0x0F );
        }
    }
    return ndx;
}

void MnUpdateMIDI()
{
    unsigned char msg[ 3 ];  

    while ( MnCheckMIDI( msg ) )
    {
        // Don't report active sensing
        if ( msg[ 0 ] == MIDI_ACTIVE )
        {
        }
        else
        {
            char buffer[ 64 ];
            sprintf( buffer, "%ld: 0x%x,0x%x,0x%x\n", millis(), (int)msg[ 0 ], (int)msg[ 1 ], (int)msg[ 2 ] );
            DEBUG(buffer);

            if ( msg[ 0 ] == MIDI_NOTEON )
            {
                DEBUG("NOTEON");
                if ( msg[ 1 ] == 0 )
                {
                    if ( msg[ 2 ] == 0 )
                    {
                        DEBUG("Hello\n" );
                    }
                    else
                    {
                        DEBUG("World\n" );
                    }
                }
            }

            SENDMIDI( msg, sizeof( msg ) );
            msg[ 0 ] = msg[ 1 ] = msg[ 2 ] = 0;
        }
    }
}
#endif

uint8_t MnTrackToPad( uint8_t const kTrack )
{
    switch ( kTrack )
    {
    case 0: return LIVID_PAD00;
    case 1: return LIVID_PAD01;
    case 2: return LIVID_PAD02;
    case 3: return LIVID_PAD03;
    case 4: return LIVID_PAD10;
    case 5: return LIVID_PAD11;
    case 6: return LIVID_PAD12;
    case 7: return LIVID_PAD13;
    case 8: return LIVID_PAD20;
    case 9: return LIVID_PAD21;
    case 10: return LIVID_PAD22;
    case 11: return LIVID_PAD23;
    case 12: return LIVID_PAD30;
    case 13: return LIVID_PAD31;
    case 14: return LIVID_PAD32;
    case 15: return LIVID_PAD33;
    }
    return 0;
}

int8_t MnFaderIndexFromCC( uint8_t const kCC )
{
    int8_t index = -1;

    switch ( kCC )
    {
    case LIVID_FADER0: index = 0; break;
    case LIVID_FADER1: index = 1; break;
    case LIVID_FADER2: index = 2; break;
    case LIVID_FADER3: index = 3; break;
    case LIVID_FADER4: index = 4; break;
    case LIVID_FADER5: index = 5; break;
    case LIVID_FADER6: index = 6; break;
    case LIVID_FADER7: index = 7; break;
    default:
        break;
    }
    return index;
}

void MnFaderOut( uint8_t const kWhich, uint8_t const kValue )
{
    // TODO: send out analog data via the DAC
}

int8_t MnPadToTrack( uint8_t const kPadValue )
{
    switch ( kPadValue )
    {
    case LIVID_PAD00: return 0;
    case LIVID_PAD01: return 1;
    case LIVID_PAD02: return 2;
    case LIVID_PAD03: return 3;
    case LIVID_PAD10: return 4;
    case LIVID_PAD11: return 5;
    case LIVID_PAD12: return 6;
    case LIVID_PAD13: return 7;
    case LIVID_PAD20: return 8;
    case LIVID_PAD21: return 9;
    case LIVID_PAD22: return 10;
    case LIVID_PAD23: return 11;
    }
    return -1;
}

void MnUpdateState( unsigned long kMillis )
{
    uint8_t kStep = g_state.getStep();

    // Clear buttons
    for ( int i = LIVID_PAD00; i <= LIVID_ENCODER23; i++ )
    {
        FE_SetButtonLED( i, LIVID_COLOR_OFF );
    }
    // Clear encoders
    for ( int i = LIVID_ENCODER00; i <= LIVID_ENCODER23; i++ )
    {
        FE_SetEncoderLED( i, 0 );
    }

    // Advance our controller's beat
    FE_SetActiveStep( kStep );

    // Make sure our mode is up to date
    FE_SetButtonLED( LIVID_PAD30, LIVID_COLOR_YELLOW );
    FE_SetButtonLED( LIVID_PAD31, LIVID_COLOR_YELLOW );
    FE_SetButtonLED( LIVID_PAD32, LIVID_COLOR_YELLOW );
    FE_SetButtonLED( LIVID_PAD33, LIVID_COLOR_YELLOW );

    switch ( g_state.mns_mode )
    {
    case MODE_CONTROL:
        FE_SetButtonLED( LIVID_PAD30, LIVID_COLOR_BLUE );
        break;
    case MODE_TRACK:
        FE_SetButtonLED( LIVID_PAD31, LIVID_COLOR_BLUE );
        break;
    case MODE_MUTE:
        FE_SetButtonLED( LIVID_PAD32, LIVID_COLOR_BLUE );
        break;
    case MODE_PITCH:
        FE_SetButtonLED( LIVID_PAD33, LIVID_COLOR_BLUE );
        break;
    }

    // Update encoder
    if ( g_state.mns_edit_step >= 0 )
    {
        FE_SetEncoderLED( LIVID_ENCODER00, g_state.getTrackNote( g_state.mns_active_track, g_state.mns_edit_step ) );
//        sprintf( dbg_buffer, "setting encoder: 0x%x\n", (int) g_state.getTrackNote( g_state.mns_active_track, g_state.mns_edit_step ) );
//        DEBUG(dbg_buffer);

    }
    else
    {
    }

    // Update keypad
    if ( g_state.mns_mode == MODE_CONTROL )
    {
        // Blink start if we're running
        if ( g_state.mns_running && ( g_state.mns_beat & 3 ) )
        {
            FE_SetButtonLED( LIVID_PAD00, LIVID_COLOR_OFF );  // start
        }
        else
        {
            FE_SetButtonLED( LIVID_PAD00, LIVID_COLOR_GREEN );  // start
        }
        FE_SetButtonLED( LIVID_PAD01, LIVID_COLOR_RED );    // reset
    }
    else if ( g_state.mns_mode == MODE_TRACK )
    {
        // Any tracks with data should be in yellow and blink if active beat
        for ( uint8_t i = 0; i < NUM_TRACKS; i++ )
        {
            if ( g_state.trackHasAnyData( i ) )
            {
                if ( g_state.isTrackStepEnabled( i, g_state.getStep() ) )
                {
                    FE_SetButtonLED( MnTrackToPad( i ), LIVID_COLOR_MAGENTA );
                }
                else
                {
                    FE_SetButtonLED( MnTrackToPad( i ), LIVID_COLOR_WHITE );
                }
            }
        }

        FE_SetButtonLED( MnTrackToPad( g_state.mns_active_track ), LIVID_COLOR_BLUE );
    }
    else if ( g_state.mns_mode == MODE_MUTE )
    {
        // Any tracks with data should be in yellow and blink if active beat
        for ( uint8_t i = 0; i < NUM_TRACKS; i++ )
        {
            bool muted = g_state.mns_tracks[ i ].mnt_track_muted;
            uint8_t color = muted ? LIVID_COLOR_RED : LIVID_COLOR_OFF;

            if ( !muted )
            {
                if ( g_state.trackHasAnyData( i ) )
                {
                    if ( g_state.isTrackStepEnabled( i, g_state.getStep() ) )
                    {
                        if ( i == g_state.mns_active_track )
                            color = LIVID_COLOR_CYAN;
                        else
                            color = LIVID_COLOR_WHITE;
                    }
                    else
                    {
                        if ( i == g_state.mns_active_track )
                            color = LIVID_COLOR_BLUE;
                        else
                            color = LIVID_COLOR_GREEN;
                    }
                }
            }

            FE_SetButtonLED( MnTrackToPad( i ), color );
        }
    }

    // Update sequencer
    for ( uint8_t step = 0; step < 16; step++ )
    {
        uint8_t color = LIVID_COLOR_GREEN;
        
        if ( g_state.mns_tracks[ g_state.mns_active_track ].mnt_track_muted )
            color = LIVID_COLOR_RED;
        else if ( g_state.mns_mode != MODE_TRACK )
            color = LIVID_COLOR_MAGENTA;

        if ( g_state.isTrackStepAudible(g_state.mns_active_track, step ))
        {
            FE_SetButtonLED( LIVID_SEQ_ROW1 + step, color );
        }
    }
}

void MnCheckSequence( unsigned long const kMicrosPerSixteenth )
{
    if ( !g_state.mns_running )
       return;

    unsigned long const kNow = micros();
    unsigned long const kNowMS = millis();
    
    // If this is the first time, fill in the time and return.  This means
    // there will be a one beat delay between pressing play and actually playing
    if ( g_state.mns_last_beat_time == 0 )
    {
        g_state.mns_last_beat_time = kNow;
        return;
    }

    long int const kDeltaTime = kNow - g_state.mns_last_beat_time;
    if ( kDeltaTime < kMicrosPerSixteenth-20 )
        return;

    {
        uint8_t const kStep = g_state.getStep();

        // Advance our controller's beat
//        sprintf(dbg_buffer,"%ld: step %d delta %ld\n", micros(), (int)kStep, kDeltaTime  );
//        DEBUG(dbg_buffer);

        FE_SetActiveStep( kStep );

        for ( unsigned char track = 0; track < NUM_TRACKS; track++ )
        {
            MnTrack const &kTrack = g_state.mns_tracks[ track ];

            if ( kTrack.mnt_track_muted )
                continue;

            if ( !g_state.isTrackStepEnabled( track, kStep ) )
                continue;

            uint8_t const kNote = g_state.getTrackNote( track, kStep );

            // Tracks 0-7 are TRIGGER
            if ( track < 8 )
            {
/*
                sprintf( dbg_buffer, "HIGH @ %ld\r\n", millis() );
                DEBUG(dbg_buffer);
*/
                digitalWrite( PIN_TRIGGER0 + track, HIGH );
                MnAddTimerEvent( TET_ANALOG_LOW, PIN_TRIGGER0 + track, 0, kNowMS + 20 );
            }
            else
            {
                MIDI_noteOn( kNote, 0x40, MIDI_EXT );
                MnAddTimerEvent( TET_MIDI_NOTE_OFF, kNote, 0, kNowMS + 5 );
            }
        }

        g_state.mns_beat++;
        g_state.mns_last_beat_time = kNow;
    }
}

void MnStartSequencer()
{
    DEBUG( "Start\n" );
    g_state.mns_running = 1;
    digitalWrite(PIN_LED,HIGH);
}

void MnStopSequencer()
{
    g_state.mns_running = 0;
    digitalWrite(PIN_LED,LOW);
}

void MnResetSequencer()
{
    DEBUG( "Reset\n" );
    MnStopSequencer();
    g_state.mns_beat = 0;

}

void MnPitchAdjust( int8_t const kAmount )
{
    if ( g_state.mns_edit_step < 0 )
        return;

    int v = g_state.getTrackNote( g_state.mns_active_track, g_state.mns_edit_step );

    v += kAmount;

    if ( v > 0x7F ) 
        v = 0x7F;
    else if ( v < 1 )
        v = 1;

    sprintf( dbg_buffer, "pitch adjust: 0x%x (%d)\n", (int) v, (int) kAmount );
    DEBUG(dbg_buffer);

    g_state.mns_tracks[ g_state.mns_active_track ].mnt_step_notes[ g_state.mns_edit_step ] = (uint8_t)v;
}

void MnHandleMessage( uint8_t const _msg[3] )
{
    uint8_t msg[ 3 ] = { _msg[ 0 ], _msg[ 1 ], _msg[ 2 ] };

    // Treat note on with velocity 0 as note off
    if ( msg[ 0 ] == MIDI_NOTE_ON && msg[ 2 ] == 0 )
    {
        msg[ 0 ] = MIDI_NOTE_OFF;
    }

    // Always make sure global button state is valid
    if ( msg[ 0 ] == MIDI_NOTE_ON )
    {
        g_state.setButtonState( msg[ 1 ], 1 );
    }
    else if ( msg[ 0 ] == MIDI_NOTE_OFF )
    {
        g_state.setButtonState( msg[ 1 ], 0 );
    }

    // Real-time CV controls
    if ( msg[ 0 ] == MIDI_CC )
    {
    }

    // BPM dial
    if ( msg[ 0 ] == MIDI_CC && msg[ 1 ] == LIVID_ROTARY00 )
    {
        short int bpm = msg[ 2 ];

        bpm *= 3;
        bpm >>= 1;
        bpm += 30;

        g_state.mns_bpm = static_cast<uint8_t>(bpm);
    }

    if ( msg[ 0 ] == MIDI_CC )
    {
        int8_t const kFaderIndex = MnFaderIndexFromCC( msg[ 1 ] );

        if ( msg[ 1 ] == LIVID_ENCODER00 )
        {
            int8_t const kPitchScale = g_state.getButtonState( LIVID_ENCODER00 ) ? 12 : 1;

            if ( msg[ 2 ] == 1 )
            {
                MnPitchAdjust( kPitchScale );
            }
            else
            {
                MnPitchAdjust( -kPitchScale );
            }
        }
        else if ( kFaderIndex >= 0 )
        {
            uint8_t const kValue = msg[ 2 ];
            MnFaderOut( kFaderIndex, kValue );
        }
    }
    else if ( msg[ 0 ] == MIDI_NOTE_ON )
    {
        switch ( msg[ 1 ] )
        {
        case LIVID_PAD30:
            g_state.mns_mode = MODE_CONTROL;
            break;
        case LIVID_PAD31:
            g_state.mns_mode = MODE_TRACK;
            break;
        case LIVID_PAD32:
            g_state.mns_mode = MODE_MUTE;
            break;
        case LIVID_PAD33:
            g_state.mns_mode = MODE_PITCH;
            break;
        default:
            break;
        }

        if ( g_state.mns_mode == MODE_CONTROL )
        {
            if ( msg[ 1 ] == LIVID_PAD00 && g_state.mns_mode == MODE_CONTROL )
            {
                g_state.mns_running = !g_state.mns_running;
            }
            else if ( msg[ 1 ] == LIVID_PAD01 && g_state.mns_mode == MODE_CONTROL )
            {
                MnResetSequencer();
            }
        }
        else if ( ( g_state.mns_mode == MODE_TRACK || g_state.mns_mode == MODE_MUTE ) && 
                  ( msg[ 1 ] >= LIVID_SEQ_ROW1 && msg[ 1 ] < LIVID_SEQ_ROW1 + NUM_STEPS ) )
        {
            int8_t beat = msg[ 1 ] - LIVID_SEQ_ROW1;

            // toggle beat
            if ( beat >= 0 && beat < NUM_STEPS )
            {
                g_state.toggleBeat( g_state.mns_active_track, beat );
            }
        }
        else if ( g_state.mns_mode == MODE_TRACK )
        {
            if ( msg[ 1 ] >= LIVID_PAD00 && msg[ 1 ] <= LIVID_PAD33 )
            {
                int const kTrack = MnPadToTrack( msg[ 1 ] );

                if ( kTrack >= 0 )
                {
                    g_state.mns_active_track = kTrack;
                }
            }
        }
        else if ( g_state.mns_mode == MODE_MUTE )
        {
            if ( msg[ 1 ] >= LIVID_PAD00 && msg[ 1 ] <= LIVID_PAD33 )
            {
                int const kTrack = MnPadToTrack( msg[ 1 ] );

                if ( kTrack >= 0 )
                {
                    g_state.mns_tracks[ kTrack ].mnt_track_muted = !g_state.mns_tracks[ kTrack ].mnt_track_muted;
                }
            }
        }

        if ( msg[ 1 ] >= LIVID_SEQ_ROW0 && msg[ 1 ] < LIVID_SEQ_ROW0 + NUM_STEPS )
        {
            g_state.mns_edit_step = msg[ 1 ] - LIVID_SEQ_ROW0;
            sprintf( dbg_buffer, "Edit step now %d\n", g_state.mns_edit_step );
            DEBUG(dbg_buffer);
        }
    }
    else if ( msg[ 0 ] == MIDI_NOTE_OFF )
    {
        if ( msg[ 1 ] >= LIVID_SEQ_ROW0 && msg[ 1 ] < LIVID_SEQ_ROW0 + NUM_STEPS )
        {
            g_state.mns_edit_step = -1;
            sprintf( dbg_buffer, "Edit step off\n" );
            DEBUG(dbg_buffer);
        }
    }
}

#ifdef DOUSB
void MnReconnectUSB()
{
    DEBUG("Reconnecting USB...\n" );
    Serial.begin(31500);

    DEBUG("USB.Init()..." );
    if (Usb.Init() == -1) 
    {
        DEBUG("failed\n" );
        return;
    }
    DEBUG("success\n" );

    delay( 200 );
    int attempts = 10;

    DEBUG("Waiting on USB to run");
    while ( Usb.getUsbTaskState() != USB_STATE_RUNNING )
    {
        DEBUG(".");
        Usb.Task();
        delay(1000);
        if ( --attempts == 0 )
            break;
    }

    if ( Usb.getUsbTaskState() != USB_STATE_RUNNING )
    {
        DEBUG("failed\n" );
        return;
    }

    DEBUG("USB running!\n" );
    g_usb_connected = 1;

    while ( 0 )
    {
        uint8_t ptr[ 32 ];
        uint16_t num_bytes = 0;
        uint8_t retval;

        if ( ( retval = Midi.RcvData( &num_bytes, ptr ) ) != 0 )
        {
            sprintf( dbg_buffer, "rval: %d\n", ( uint8_t ) retval );
            DEBUG(dbg_buffer);
            for ( int i = 0; i < num_bytes; i++ )
            {
                sprintf( dbg_buffer, "0x%x\n", ptr[ i ] );
                DEBUG(dbg_buffer);
            }
        }
        delay(100);
    }

    FE_Setup();
    delay(10);

#if 1
    DEBUG( "Sending CNTRL:R sysex\n" );

#if 1
    // Set encoders to fill mode
    {
        uint8_t sysex_fill_mode[] =
        { LIVID_SYSEX, 
        0x1D,
        0x1, 0x3, 0x7, 0x00,
        MIDI_EOX
        };

        SENDMIDI_USB(sysex_fill_mode);
        delay(10);
    }
#else
    {
        uint8_t cmd[] = { 
            MIDI_CC, 0x75,1
        };
        SENDMIDI_USB(cmd);
        delay(10);
    }
#endif

    // Set encoding mode to relative (inc/dec)
#if 1
    {
        uint8_t sysex_encosion_mode[] =
        { LIVID_SYSEX, 
        0x11,
        0x7F, 0x01, 0x0F, 0x00,
        MIDI_EOX
        };

        SENDMIDI_USB(sysex_encosion_mode);
        delay(10);
    }
#endif

    // Local encoder off
#if 1
    {
        uint8_t sysex_local_off_mode[] =
        { LIVID_SYSEX, 
        0x20,
        0x20,
        MIDI_EOX
        };
        SENDMIDI_USB(sysex_local_off_mode);
        delay(10);
    }
#endif

#endif

    // Clear backbuffers
    memset( &g_state.mns_led_button_backbuffer, 0, sizeof( g_state.mns_led_button_backbuffer ) );
    memset( &g_state.mns_led_encoder_backbuffer, 0, sizeof( g_state.mns_led_encoder_backbuffer ) );

    g_usb_connected = 1;
}

void MnCheckUSB()
{
    static long s_last_check_time;
    long const kNow = millis();

    if ( !g_usb_connected )
    {
        MnReconnectUSB();
        return;
    }

    Usb.Task();

    if ( Usb.getUsbTaskState() == USB_STATE_RUNNING )
    {
        byte msgs[ 32 ] = { 0, 0, 0 };
        uint16_t bytes_received = 3;

        if ( kNow - s_last_check_time > 1000 )
        {
            sprintf( dbg_buffer, "%ld: USB running\n", kNow );
            DEBUG( dbg_buffer );
        }
//        while ( Midi.RcvData(&bytes_received,msgs) == 0 )
        if ( Midi.RcvData(msgs) )
        {
            if ( bytes_received )
            {
                sprintf( dbg_buffer, "%ld[%d]: ", (long)millis(), (int)bytes_received);
                DEBUG( dbg_buffer );
                for ( int i = 0; i < bytes_received; i++ )
                {
                    sprintf( dbg_buffer, "0x%x ", (int)msgs[i] );
                    DEBUG(dbg_buffer);
                }
                DEBUG("\n" );

                MnHandleMessage( msgs );
            }
        }
    }
    else
    {
        if ( g_usb_connected )
        {
            DEBUG("USB connection lost!\n" );
            g_usb_connected = 0;
        }

        if ( kNow - s_last_check_time > 1000 )
        {
            sprintf( dbg_buffer, "%ld: USB NOT FOUND\n", kNow );
            DEBUG( dbg_buffer );
        }
    }

    s_last_check_time = kNow;
}
#endif

void MnCheckLocalControls()
{
}

void MnFireTimerEvent( uint8_t const kIndex )
{
    MnTimerEvent &kEvent = g_state.mns_timer_events[ kIndex ];

    switch ( kEvent.te_type )
    {
    case TET_MIDI_NOTE_OFF:
        MIDI_noteOff( kEvent.te_param0, 0, MIDI_EXT );
        break;
    case TET_ANALOG_LOW:
/*
        sprintf( dbg_buffer, "LOW %d @ %ld\n", (int)kEvent.te_param0, millis() );
        DEBUG( dbg_buffer );
*/
        digitalWrite( kEvent.te_param0, LOW );
        break;
    default:
        break;
    }
}

void MnCheckTimers( unsigned long const kNowMS )
{
    for ( uint8_t i = 0; i < MAX_TIMER_EVENTS; i++ )
    {
        if ( g_state.mns_timer_events[ i ].te_trigger_time_ms == 0 )
            continue;
        if ( g_state.mns_timer_events[ i ].te_trigger_time_ms <= kNowMS )
        {
            g_state.mns_timer_events[ i ].te_trigger_time_ms = 0;
            MnFireTimerEvent( i );
        }
    }
}


// the setup routine runs once when you press reset:
void setup() 
{
    MnLoadState();

    // initialize the digital pin as an output.
    pinMode( PIN_LED, OUTPUT);
    digitalWrite( PIN_LED,HIGH);
  
    for ( unsigned char i = 0; i < NUM_TRIGGERS; i++ )
    {
        pinMode( PIN_TRIGGER0 + i, OUTPUT );
        digitalWrite( PIN_TRIGGER0 + i, LOW );
    }

    // This is debug monitor
    Serial1.begin(57600);

    // This is the MIDI OUT port
    DEBUG( "Initializing MIDI OUT\n" );
    Serial2.begin( 31250 );

    sprintf( dbg_buffer, "sizeof( g_state ) == %d\n", sizeof( g_state ) );
    DEBUG( dbg_buffer );
}

void loop() 
{
    unsigned long const kBeatDurationMicros = 60000000 / g_state.mns_bpm;
    unsigned long const kMicrosPerSixteenth = kBeatDurationMicros >> 2;
    unsigned long const kNow = millis();

    MnCheckSequence( kMicrosPerSixteenth );

    MnCheckTimers( kNow );

    // Go through all of our state and make sure the LEDs
    // reflect what's going on
    MnUpdateState( kNow );

#ifndef DOUSB
    MnUpdateMIDI();
#endif

    MnCheckLocalControls();

#ifdef DOUSB
    MnCheckUSB();
#endif

    FE_SwapBuffers();
}
