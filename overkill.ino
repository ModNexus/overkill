// Schematics:
//
// TODO: Solder extension headers on USB shield
// TODO: Add back button
// TODO: SD shield
// TODO: Double check relative encoder stuff
// TODO: MIDI channel/controller configuration
// TODO: CV outputs (manual)
// TODO: CV outputs (sequenced)
// TODO: analog clock input/output/reset
// TODO: MIDI IN (for clocking)?
// TODO: chord stuff?
#include <SD.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <Usb.h>
#include <usbh_midi.h>
#include "livid.h"
#include "overkill.h"

#define CLAMP(a,b,c) ((a)=min(max(a,b),c))

volatile uint8_t g_encoder;
volatile uint8_t g_last_encoder;

char  dbg_buffer[ 128 ];
USB   Usb;
MIDI  Midi(&Usb);

File fp_config;

#define MIDI_INTERVAL 2

LiquidCrystal lcd( PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7 );

void MIDI_noteOn( uint8_t n, uint8_t vel, uint8_t port )
{
    uint8_t  buffer[3] =
    {
        MIDI_NOTE_ON, n, vel
    };

    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
        delay(1);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
        sprintf( dbg_buffer, "[%08lu] Note on  0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
        DEBUG( dbg_buffer );
    }
}

void MIDI_cc( uint8_t n, uint8_t val, uint8_t port )
{
    uint8_t buffer[3] =
    {
        MIDI_CC, n, val
    };

    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
        delay(1);
//        sprintf( dbg_buffer, "[%08lu] CC 0x%x 0x%x\n", millis(), (int)n,(int)val );
//        DEBUG( dbg_buffer );
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
}
void MIDI_noteOff( uint8_t n, uint8_t vel, uint8_t port )
{
    uint8_t buffer[3] =
    {
        MIDI_NOTE_OFF, n, vel
    };
    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
        delay(1);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
        sprintf( dbg_buffer, "[%08lu] Note off 0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
        DEBUG( dbg_buffer );
    }
}

struct LividCNTRLR
{
    uint8_t lc_led_button_backbuffer[ 60 ];   // These are what we update
    uint8_t lc_led_button_frontbuffer[ 60 ];  // These are what we're currently showing
    uint8_t lc_led_encoder_backbuffer[ LIVID_NUM_ENCODERS ];  // These are what we update
    uint8_t lc_led_encoder_frontbuffer[ LIVID_NUM_ENCODERS ]; // These are what we're currently showing

    void resetBuffers( void )
    {
        DEBUG("Resetting buffers\n");
        memset( &lc_led_button_backbuffer,    0, sizeof( lc_led_button_backbuffer ) );
        memset( &lc_led_encoder_backbuffer,   0, sizeof( lc_led_encoder_backbuffer ) );
        memset( &lc_led_button_frontbuffer, 0x1, sizeof( lc_led_button_frontbuffer ) );
        memset( &lc_led_encoder_frontbuffer, 0x1, sizeof( lc_led_encoder_frontbuffer ) );
    }

    void clearScreen( void )
    {
        memset( &lc_led_button_backbuffer, 0, sizeof( lc_led_button_backbuffer ) );
        memset( &lc_led_encoder_backbuffer, 0, sizeof( lc_led_encoder_backbuffer ) );
    }

    void setButtonLED( uint8_t const kWhich, uint8_t const kCC )
    {
        lc_led_button_backbuffer[ kWhich ] = kCC;
    }
    void setEncoderLED( uint8_t const kWhich, uint8_t const kCC )
    {
        lc_led_encoder_backbuffer[ kWhich-LIVID_ENCODER00 ] = kCC;
    }

    void swapBuffers()
    {
        // Update buttons
        for ( uint8_t i = LIVID_PAD00; i <= LIVID_ENCODER23; i++ )
        {
            if ( lc_led_button_backbuffer[ i ] != lc_led_button_frontbuffer[ i ] )
            {
                uint8_t const b = lc_led_button_backbuffer[ i ];

                lc_led_button_frontbuffer[ i ] = b;
                if ( b )
                    MIDI_noteOn( i, b, MIDI_USB );
                else
                    MIDI_noteOn( i, 0, MIDI_USB ); // we send note on with 0
            }
        }

        for ( uint8_t i = 0; i < LIVID_NUM_ENCODERS; i++ )
        {
            if ( lc_led_encoder_backbuffer[ i ] != lc_led_encoder_frontbuffer[ i ] )
            {
                uint8_t b = lc_led_encoder_backbuffer[ i ];

                lc_led_encoder_frontbuffer[ i ] = b;

                MIDI_cc( i + LIVID_ENCODER00, b, MIDI_USB );
            }
        }
    }

    uint8_t trackToPad( uint8_t const kTrack )
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

    int8_t faderIndexFromCC( uint8_t const kCC )
    {
        switch ( kCC )
        {
        case LIVID_FADER0: return 0; break;
        case LIVID_FADER1: return 1; break;
        case LIVID_FADER2: return 2; break;
        case LIVID_FADER3: return 3; break;
        case LIVID_FADER4: return 4; break;
        case LIVID_FADER5: return 5; break;
        case LIVID_FADER6: return 6; break;
        case LIVID_FADER7: return 7; break;
        default:
            break;
        }
        return -1;
    }
};

LividCNTRLR g_device;

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
    
    // gate durations are 32nd notes, so 128 * 1/32 = 4 whole notes max
    uint8_t         mnt_step_gate[ NUM_STEPS ];

    uint8_t         mnt_step_cv0[ NUM_STEPS ];
    uint8_t         mnt_step_cv1[ NUM_STEPS ];
    uint8_t         mnt_step_cv2[ NUM_STEPS ];

    unsigned long   mnt_gate_on_time;  // When did the gate start
    unsigned long   mnt_gate_off_time; // When is the gate supposed to stop

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

struct MnRunningState
{
    long     rs_lcd_restore_time;

    uint8_t  rs_encoder_down;
    uint8_t  rs_back_down;

    MainMenu_t rs_main_menu;
    uint8_t    rs_temp_clock_source;
    int8_t     rs_temp_track_select;
    bool       rs_is_editing_config;

    uint8_t    rs_usb_connected;
    uint16_t   rs_beat;
    uint32_t   rs_last_beat_time;
    bool       rs_running;
    uint8_t    getStep( void ) { return rs_beat & ( NUM_STEPS - 1 ); }

    //
    // button stuff
    //
    uint8_t rs_button_state[ 8 ]; // Assumes up to 64 buttons (CNTRL:R has almost that many)
    uint8_t getButtonState( uint8_t const kControlNumber )
    {
        uint8_t const kByteVal = kControlNumber >> 3;
        uint8_t const kBitVal  = kControlNumber & 7;

        if ( kByteVal >= sizeof( rs_button_state ) )
        {
            return 0;
        }

        return ( rs_button_state[ kByteVal ] >> kBitVal ) & 1;
    }
    void setButtonState( uint8_t const kControlNumber, bool const kOn )
    {
        uint8_t const kByteVal = kControlNumber >> 3;
        uint8_t const kBitVal  = kControlNumber & 7;

        if ( kByteVal >= sizeof( rs_button_state ) )
        {
            return;
        }

        if ( kOn )
        {
            rs_button_state[ kByteVal ] |= ( 1 << kBitVal );
        }
        else
        {
            rs_button_state[ kByteVal ] &= ~( 1 << kBitVal );
        }
    }

    MnRunningState()
    {
        rs_temp_track_select = -1;
    }


};

MnRunningState g_rs;

struct MnState
{
    uint32_t mns_cookie;
    uint16_t mns_version;

    ClockSource_t mns_clock_source;
    TrackConfig_t mns_track_config;

    //
    // misc state
    //
    uint8_t       mns_mode;

    uint16_t      mns_bpm_times_10;  // scaled BPM
    uint8_t       mns_active_track;

    int8_t        mns_gate_value;
    int8_t        mns_cv0_value, mns_cv1_value, mns_cv2_value;
    bool          mns_is_editing_step[ NUM_STEPS ]; // Is step being edited?

    uint8_t       mns_step_mode;

    uint8_t       mns_select_mode;
    uint8_t       mns_shift_select_mode;
    bool          mns_shift;

    bool          mns_editing_gate, mns_editing_cv2, mns_editing_cv0, mns_editing_cv1;

    bool isEditingAnyStep( void )
    {
        for ( uint8_t i = 0; i < NUM_STEPS; i++ )
        {
            if ( isEditingStep( i ) )
                return true;
        }
        return false;
    }
    bool isEditingStep( int8_t const kStep )
    {
        return mns_is_editing_step[ kStep ];
    }

    MnTrack       mns_tracks[ NUM_TRACKS ];

    bool trackHasAnyData( uint8_t const kTrack ) 
    {
        for ( int8_t i = 0; i < NUM_STEPS; i++ )
        {
            if ( isTrackStepEnabled( kTrack, i ) )
            {
                return true;
            }
        }
        return false;
    }

    // Returns value irrespective of mute
    uint8_t getTrackGate( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_gate[ kStep ];
    }
    bool isTrackNoteOn( uint8_t const kTrack )
    {
        return mns_tracks [ kTrack ].mnt_gate_off_time > 0;
    }
    uint8_t getTrackCV0( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_cv0[ kStep ];
    }
    uint8_t getTrackCV1( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_cv1[ kStep ];
    }
    uint8_t getTrackCV2( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_cv2[ kStep ];
    }
    void setTrackCV0( uint8_t const kTrack, uint8_t const kStep, uint8_t const kValue )
    {
        mns_tracks[ kTrack ].mnt_step_cv0[ kStep ] = kValue;
    }
    void setTrackCV1( uint8_t const kTrack, uint8_t const kStep, uint8_t const kValue )
    {
        mns_tracks[ kTrack ].mnt_step_cv1[ kStep ] = kValue;
    }
    void setTrackCV2( uint8_t const kTrack, uint8_t const kStep, uint8_t const kValue )
    {
        mns_tracks[ kTrack ].mnt_step_cv2[ kStep ] = kValue;
    }
    void setTrackGate( uint8_t const kTrack, uint8_t const kStep, uint8_t const kValue )
    {
        mns_tracks[ kTrack ].mnt_step_gate[ kStep ] = kValue;
    }

    bool isTrackStepEnabled( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks[ kTrack ].isStepEnabled( kStep );
    }

    // Toggles the beat
    void toggleBeat( uint8_t const kTrack, uint8_t const kStep )
    {
        bool const kEnabled = mns_tracks[ kTrack ].isStepEnabled( kStep );

        mns_tracks[ kTrack ].enableStep( kStep, !kEnabled );
    }

    void editGate( int8_t const kValue )
    {
        for ( int8_t step = 0; step < NUM_STEPS; step++ )
        {
            if ( isEditingStep( step ) )
            {
                setTrackGate( mns_active_track, step, kValue );
            }
        }
    }
    void editCV0( int8_t const kValue )
    {
        for ( int8_t step = 0; step < NUM_STEPS; step++ )
        {
            if ( isEditingStep( step ) )
            {
                setTrackCV0( mns_active_track, step, kValue );
            }
        }
    }
    void editCV1( int8_t const kValue )
    {
        for ( int8_t step = 0; step < NUM_STEPS; step++ )
        {
            if ( isEditingStep( step ) )
            {
                setTrackCV1( mns_active_track, step, kValue );
            }
        }
    }
    void editCV2( int8_t const kValue )
    {
        for ( int8_t step = 0; step < NUM_STEPS; step++ )
        {
            if ( isEditingStep( step ) )
            {
                setTrackCV2( mns_active_track, step, kValue );
            }
        }
    }

    // Set sane default track values
    MnState() : mns_cookie( OVERKILL_MAGIC_COOKIE ), mns_version( OVERKILL_STATE_VERSION )
    {
    }
};

MnState g_state;

#if 0
//
// Timer stuff
//
MnTimerEvent  mns_timer_events[ MAX_TIMER_EVENTS ];
void addTimerEvent( uint8_t const kType, uint8_t const kParam0, uint8_t const kParam1, uint32_t const kWhen );

bool removeTimerEvent( uint8_t const kType, uint8_t const kParam0 )
{
    uint8_t s;
    bool found = false;

    for ( s = 0; s < MAX_TIMER_EVENTS; s++ )
    {
        if ( ( mns_timer_events[ s ].te_type == kType ) &&
            ( mns_timer_events[ s ].te_param0 == kParam0 ) &&
            ( mns_timer_events[ s ].te_trigger_time_ms > 0 ) )
        {
            mns_timer_events[ s ].te_trigger_time_ms = 0;
            found = true;
        }
    }

    return found;
}

void MnState::addTimerEvent( uint8_t const kType, uint8_t const kParam0, uint8_t const kParam1, uint32_t const kWhen )
{
    // Find an empty slot
    uint8_t free_slot = 0;

    for ( free_slot = 0; free_slot < MAX_TIMER_EVENTS; free_slot++ )
    {
        if ( mns_timer_events[ free_slot ].te_trigger_time_ms == 0 )
        {
            break;
        }
    }

    if ( free_slot == MAX_TIMER_EVENTS )
    {
        DEBUG("Out of timer events!\n" );
        return;
    }

    MnTimerEvent *e = &mns_timer_events[ free_slot ];

    e->te_type = kType;
    e->te_trigger_time_ms = kWhen;
    e->te_param0 = kParam0;
    e->te_param1 = kParam1;
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

#endif

void MnLoadState()
{
    g_state.mns_bpm_times_10 = 1200;

    // Set default gate length for drum tracks to 
    for ( uint8_t t = 0; t < NUM_TRIGGERS; t++ )
    {
        for ( uint8_t s = 0; s < NUM_STEPS; s++ )
        {
            g_state.setTrackGate( t, s, 1 ); // 1/32nd note
        }
    }

    // Set default gate length for pitch tracks to 1/4 note
    for ( uint8_t t = 8; t < NUM_TRACKS; t++ )
    {
        for ( uint8_t s = 0; s < NUM_STEPS; s++ )
        {
            g_state.setTrackGate( t, s, 1 ); // 1/4 note
        }
    }

    // Set default pitch
    for ( uint8_t t = 8; t < NUM_TRACKS; t++ )
    {
        for ( uint8_t s = 0; s < NUM_STEPS; s++ )
        {
            g_state.setTrackCV0( t, s, 0x3C ); // 1/4 note
        }
    }
}

void MnSaveState()
{
    // Save patterns to SD
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
    uint8_t const kStep = g_rs.getStep();

    g_device.clearScreen();

    // Update encoders
    g_device.setEncoderLED( LIVID_ENCODER00, g_state.getTrackGate( g_state.mns_active_track, kStep ) );
    g_device.setEncoderLED( LIVID_ENCODER01, g_state.getTrackCV0( g_state.mns_active_track, kStep ) );
    g_device.setEncoderLED( LIVID_ENCODER02, g_state.getTrackCV1( g_state.mns_active_track, kStep ) );
    g_device.setEncoderLED( LIVID_ENCODER03, g_state.getTrackCV2( g_state.mns_active_track, kStep ) );

    //
    // Render control row
    //
    g_device.setButtonLED( OVERKILL_SHIFT_BUTTON, LIVID_COLOR_OFF  ); // shift key
    g_device.setButtonLED( OVERKILL_SELECT_BUTTON, LIVID_COLOR_CYAN ); // active/!active, shift = sel all/none

    if ( g_state.mns_step_mode == MODE_STEP_EDIT )
        g_device.setButtonLED( OVERKILL_EDIT_STEP_BUTTON, LIVID_COLOR_RED ); // edit vs. select
    else
        g_device.setButtonLED( OVERKILL_EDIT_STEP_BUTTON, LIVID_COLOR_CYAN ); // edit vs. select

    g_device.setButtonLED( OVERKILL_RANGE_SELECT_BUTTON, LIVID_COLOR_CYAN ); // range select

    g_device.setButtonLED( OVERKILL_GATE_BUTTON, LIVID_COLOR_GREEN ); // gate
    g_device.setButtonLED( OVERKILL_CV0_BUTTON, LIVID_COLOR_GREEN ); // pitch
    g_device.setButtonLED( OVERKILL_CV1_BUTTON, LIVID_COLOR_GREEN ); // CV0
    g_device.setButtonLED( OVERKILL_CV2_BUTTON, LIVID_COLOR_GREEN ); // CV1

    g_device.setButtonLED( OVERKILL_ALL_OFF_BUTTON, LIVID_COLOR_BLUE );        // all off
    g_device.setButtonLED( LIVID_SEQ_ROW1 + 9, LIVID_COLOR_BLUE );      // (unused)
    g_device.setButtonLED( OVERKILL_COPY_PASTE_BUTTON, LIVID_COLOR_BLUE );     // copy/paste
    g_device.setButtonLED( OVERKILL_SELECT_PATTERN_BUTTON, LIVID_COLOR_BLUE ); // pattern select

    if ( g_state.mns_mode == MODE_TRACK_SELECT )
    {
        g_device.setButtonLED( OVERKILL_TRACK_MUTE_BUTTON, LIVID_COLOR_OFF ); // track select
        g_device.setButtonLED( OVERKILL_TRACK_SELECT_BUTTON, LIVID_COLOR_YELLOW ); // track mute
    }
    else
    {
        g_device.setButtonLED( OVERKILL_TRACK_MUTE_BUTTON, LIVID_COLOR_YELLOW ); // track select
        g_device.setButtonLED( OVERKILL_TRACK_SELECT_BUTTON, LIVID_COLOR_OFF ); // track mute
    }

    g_device.setButtonLED( OVERKILL_RESET_BUTTON, LIVID_COLOR_MAGENTA ); // track select
    g_device.setButtonLED( OVERKILL_START_BUTTON, LIVID_COLOR_MAGENTA ); // track select

    // Update encoder
    if ( g_state.isEditingAnyStep() )
    {
        g_device.setButtonLED( LIVID_ENCODER00, 1 );
        g_device.setButtonLED( LIVID_ENCODER01, 1 );
        g_device.setButtonLED( LIVID_ENCODER02, 1 );
        g_device.setButtonLED( LIVID_ENCODER03, 1 );
    }

    //
    // Blink start if we're running
    //
    if ( g_rs.rs_running )
    {
        if  ( g_rs.getStep() & 3 ) 
        {
            g_device.setButtonLED( OVERKILL_START_BUTTON, LIVID_COLOR_OFF );  // start
            g_device.setButtonLED( LIVID_ENCODER20, LIVID_COLOR_OFF );
        }
        else
        {
            g_device.setButtonLED( LIVID_ENCODER20, LIVID_COLOR_GREEN );
        }
    }

    //
    // Draw 16x16
    //
    if ( g_state.mns_editing_cv0 )
    {
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 0, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 1, LIVID_COLOR_OFF );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 2, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 3, LIVID_COLOR_OFF );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 4, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 5, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 6, LIVID_COLOR_OFF );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 7, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 8, LIVID_COLOR_OFF );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 9, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 10, LIVID_COLOR_OFF );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 11, LIVID_COLOR_WHITE );
        g_device.setButtonLED( LIVID_SEQ_ROW0 + 12, LIVID_COLOR_WHITE );

        // Show active one
        // Show octave
    }
    else if ( g_state.mns_editing_gate || g_state.mns_editing_cv1 || g_state.mns_editing_cv2 )
    {
        int8_t kNum = 0;

        if ( g_state.mns_editing_gate )
        {
            kNum = ( g_state.mns_gate_value == 0 ) ? 0 : ( ( g_state.mns_gate_value  ) >> 2 ) + 1;
        }
        else if ( g_state.mns_editing_cv1 )
        {
            kNum = ( g_state.mns_cv1_value == 0 ) ? 0 : ( ( g_state.mns_cv1_value  ) >> 2 ) + 1;
        }
        else if ( g_state.mns_editing_cv2 )
        {
            kNum = ( g_state.mns_cv2_value == 0 ) ? 0 : ( ( g_state.mns_cv2_value  ) >> 2 ) + 1;
        }

        // Light up pads
        for ( int8_t i = 0; i < 16; i++ )
        {
            g_device.setButtonLED( g_device.trackToPad(i), i < kNum ? LIVID_COLOR_GREEN : LIVID_COLOR_OFF );
        }

        // And sequencer rows
        for ( int8_t i = 0; i < 16; i++ )
        {
            g_device.setButtonLED( LIVID_SEQ_ROW0 + i,  i+16 < kNum ? LIVID_COLOR_GREEN : LIVID_COLOR_OFF );
        }
    }
    else if ( g_state.mns_mode == MODE_TRACK_SELECT )
    {
        // Any tracks with data should be in yellow and blink if active beat
        for ( uint8_t i = 0; i < NUM_TRACKS; i++ )
        {
            if ( g_state.trackHasAnyData( i ) )
            {
                if ( g_state.isTrackNoteOn( i ) )
                {
                    g_device.setButtonLED( g_device.trackToPad( i ), LIVID_COLOR_MAGENTA );
                }
                else
                {
                    g_device.setButtonLED( g_device.trackToPad( i ), LIVID_COLOR_WHITE );
                }
            }
        }

        g_device.setButtonLED( g_device.trackToPad( g_state.mns_active_track ), LIVID_COLOR_BLUE );
    }
    // Track mutes
    else if ( g_state.mns_mode == MODE_TRACK_MUTE )
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
                    // If track is active
                    if ( g_state.isTrackNoteOn( i ) )
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

            g_device.setButtonLED( g_device.trackToPad( i ), color );
        }
    }

    //
    // Update sequencer
    //
    if ( !( g_state.mns_editing_cv0 || g_state.mns_editing_cv1 || g_state.mns_editing_cv2 || g_state.mns_editing_gate ) )
    {
        for ( uint8_t step = 0; step < 16; step++ )
        {
            // Advance our controller's beat
            if ( step == kStep )
            {
                g_device.setButtonLED( kStep + LIVID_SEQ_ROW0, LIVID_COLOR_WHITE );
            }
            else
            {
                if ( g_state.mns_step_mode == MODE_STEP_MUTE )
                {
                    if ( g_state.isTrackStepEnabled( g_state.mns_active_track, step ) )
                    {
                        g_device.setButtonLED( LIVID_SEQ_ROW0 + step, LIVID_COLOR_GREEN );
                    }
                }
                else if ( g_state.mns_step_mode == MODE_STEP_EDIT )
                {
                    if ( g_state.isEditingStep( step ) )
                    {
                        g_device.setButtonLED( LIVID_SEQ_ROW0 + step, LIVID_COLOR_MAGENTA );
                    }
                    else
                    {
                    }
                }
            }
        }
    }
}

void MnCheckSequence( unsigned long const kMicrosPer32nd )
{
    if ( !g_rs.rs_running )
       return;

    unsigned long const kNow = micros();
    unsigned long const kNowMS = millis();
    unsigned int kMillisPer32nd = kMicrosPer32nd / 1000;
    unsigned int kMillisPerGate = ( kMillisPer32nd );

    // If this is the first time, fill in the time and return.  This means
    // there will be a one beat delay between pressing play and actually playing
    if ( g_rs.rs_last_beat_time == 0 )
    {
        g_rs.rs_last_beat_time = kNow;
        return;
    }

    long int const kDeltaTime = kNow - g_rs.rs_last_beat_time;

    // If we've now advanced into the next beat, flag it
    bool const kIsNewBeat = kDeltaTime > (kMicrosPer32nd*2)-20;

    if ( kIsNewBeat )
    {
        g_rs.rs_beat++;
        g_rs.rs_last_beat_time = kNow;

        sprintf( dbg_buffer, "[%08ld] Step %d\n", kNowMS, g_rs.getStep() );
        DEBUG( dbg_buffer );
    }

    uint8_t const kStep = g_rs.getStep();

    // Step over all the tracks and see if we have to do anything
    for ( uint8_t track = 0; track < NUM_TRACKS; track++ )
    {
        MnTrack &kTrack = g_state.mns_tracks[ track ];

        uint8_t const kGate = g_state.getTrackGate( track, kStep ); // 0 = off
        uint8_t const kCV0  = g_state.getTrackCV0( track, kStep );
        uint8_t const kCV1  = g_state.getTrackCV1( track, kStep );
        uint8_t const kCV2  = g_state.getTrackCV2( track, kStep );

        // 1 - 128 :: 1/8th beat to 16 beats, then divide by two so it's a 50% duty cycle
        int const kGateDurationMS = ( kGate * kMillisPerGate );

        // Tracks 0-7 are TRIGGER
        if ( track < 8 )
        {
            // See if we have to trigger gate on
            if ( kGate && kIsNewBeat && kTrack.isStepEnabled( kStep ) )
            {
                DEBUG("Pin high\n" );
                digitalWrite( PIN_TRIGGER0 + track, HIGH );
                kTrack.mnt_gate_on_time = kNowMS;
                kTrack.mnt_gate_off_time = kNowMS + kGateDurationMS;
            }
            // Time to gate off?
            else if ( kTrack.mnt_gate_off_time > 0 && kTrack.mnt_gate_off_time < kNowMS )
            {
                DEBUG("Pin low\n" );
                digitalWrite( PIN_TRIGGER0 + track, LOW );
                kTrack.mnt_gate_off_time = kTrack.mnt_gate_on_time = 0;
            }
        }
        // Tracks 8+ are MIDI or CV
        else
        {
            // See if we have to trigger gate on
            if ( kGate && kIsNewBeat && kTrack.isStepEnabled( kStep ) )
            {
                // Already a note on?
                // TODO: Only turn off note if it's the same as this one!
                if ( g_state.isTrackNoteOn( track ) )
                {
                    MIDI_noteOff( kCV0, 0x40, MIDI_EXT );
                }

                MIDI_noteOn( kCV0, 0x40, MIDI_EXT );
                MIDI_cc( 14, kCV1, MIDI_EXT );
                MIDI_cc( 15, kCV2, MIDI_EXT );
                kTrack.mnt_gate_on_time = kNowMS;
                kTrack.mnt_gate_off_time = kNowMS + kGateDurationMS;
            }
            // Time to gate off?
            else if ( kTrack.mnt_gate_off_time > 0 && kTrack.mnt_gate_off_time < kNowMS )
            {
                MIDI_noteOff( kCV0, 0x40, MIDI_EXT );
                kTrack.mnt_gate_off_time = kTrack.mnt_gate_on_time = 0;
            }
        }
    }
}

void MnStartSequencer()
{
    DEBUG( "Start\n" );
    g_rs.rs_running = 1;
    digitalWrite(PIN_LED,HIGH);
}

void MnStopSequencer()
{
    g_rs.rs_running = 0;
    digitalWrite(PIN_LED,LOW);

    // Send all notes off
    MIDI_cc( 123, 0, MIDI_EXT );
}

void MnResetSequencer()
{
    DEBUG( "Reset\n" );
    MnStopSequencer();
    g_rs.rs_beat = 0;
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
        g_rs.setButtonState( msg[ 1 ], 1 );
    }
    else if ( msg[ 0 ] == MIDI_NOTE_OFF )
    {
        g_rs.setButtonState( msg[ 1 ], 0 );

        if ( msg[ 1 ] == OVERKILL_SHIFT_BUTTON )
        {
            g_state.mns_shift = false;
        }
        else if ( msg[ 1 ] == OVERKILL_GATE_BUTTON )
        {
            g_state.mns_editing_gate = false;
        }
        else if ( msg[ 1 ] == OVERKILL_CV2_BUTTON )
        {
            g_state.mns_editing_cv2 = false;
        }
        else if ( msg[ 1 ] == OVERKILL_CV0_BUTTON )
        {
            g_state.mns_editing_cv0 = false;
        }
        else if ( msg[ 1 ] == OVERKILL_CV1_BUTTON )
        {
            g_state.mns_editing_cv1 = false;
        }
    }

    //
    // Real-time CV controls
    //
    if ( msg[ 0 ] == MIDI_CC )
    {
        if ( g_state.mns_step_mode == MODE_STEP_EDIT )
        {
            if ( msg[ 1 ] == LIVID_ENCODER00 )
            {
                int8_t dV = msg[ 2 ] == 1 ? 1 : -1;

                if ( g_state.mns_editing_gate )
                {
                    int x = g_state.mns_gate_value + dV;
                    CLAMP( x, 0, 127 );
                    g_state.mns_gate_value = static_cast<int8_t>(x);
                    g_state.editGate( g_state.mns_gate_value );
                }
                if ( g_state.mns_editing_cv0 )
                {
                    int x = g_state.mns_cv0_value + dV;
                    CLAMP( x, 0, 127 );
                    g_state.mns_cv0_value = static_cast<int8_t>(x);
                    g_state.editCV0( g_state.mns_cv0_value );
                }
                if ( g_state.mns_editing_cv1 )
                {
                    int x = g_state.mns_cv1_value + dV;
                    CLAMP( x, 0, 127 );
                    g_state.mns_cv1_value = static_cast<int8_t>(x);
                    g_state.editCV1( g_state.mns_cv1_value );
                }
                if ( g_state.mns_editing_cv2 )
                {
                    int x = g_state.mns_cv2_value + dV;
                    CLAMP( x, 0, 127 );
                    g_state.mns_cv2_value = static_cast<int8_t>(x);
                    g_state.editCV2( g_state.mns_cv2_value );
                }
            }
        }
    }

    // Sysex?
    if ( msg[ 0 ] == MIDI_SYSEX )
    {
    }

    // BPM dial
    if ( msg[ 0 ] == MIDI_CC && msg[ 1 ] == LIVID_ENCODER20 )
    {
        int8_t dV = msg[ 2 ] == 1 ? 1 : -1;

        g_state.mns_bpm_times_10 += dV * ( g_rs.getButtonState( LIVID_ENCODER20 ) ? 1 : 10 );

        if ( g_state.mns_bpm_times_10 < 300 )
            g_state.mns_bpm_times_10 = 300;
        else if ( g_state.mns_bpm_times_10 > 2500 )
            g_state.mns_bpm_times_10 = 2500;

        char buffer[ 32 ];
        sprintf( buffer, "BPM: %d.%d", g_state.mns_bpm_times_10/10, g_state.mns_bpm_times_10%10);

        lcd.clear();
        lcd.home();
        lcd.print( buffer );

        g_rs.rs_lcd_restore_time = millis() + 2000;
    }

    if ( msg[ 0 ] == MIDI_CC )
    {
        int8_t const kFaderIndex = g_device.faderIndexFromCC( msg[ 1 ] );

        if ( kFaderIndex >= 0 )
        {
            uint8_t const kValue = msg[ 2 ];
            MnFaderOut( kFaderIndex, kValue );
        }
    }
    else if ( msg[ 0 ] == MIDI_NOTE_ON )
    {
        switch ( msg[ 1 ] )
        {
        case OVERKILL_GATE_BUTTON:
            g_state.mns_editing_gate = true;
            break;
        case OVERKILL_CV2_BUTTON:
            g_state.mns_editing_cv2 = true;
            break;
        case OVERKILL_CV0_BUTTON:
            g_state.mns_editing_cv0 = true;
            break;
        case OVERKILL_CV1_BUTTON:
            g_state.mns_editing_cv1 = true;
            break;

        case OVERKILL_SHIFT_BUTTON:
            g_state.mns_shift = true;
            break;
        case OVERKILL_EDIT_STEP_BUTTON:
            g_state.mns_step_mode = g_state.mns_step_mode == MODE_STEP_EDIT ? MODE_STEP_MUTE : MODE_STEP_EDIT;
            break;
        case OVERKILL_SELECT_BUTTON:
            if ( g_state.mns_shift )
            {
                g_state.mns_shift_select_mode = !g_state.mns_shift_select_mode;

                bool const kEnable = g_state.mns_shift_select_mode == MODE_SHIFT_SELECT_ALL ? true : false;

                for ( uint8_t step = 0; step < NUM_STEPS; step++ )
                {
                    g_state.mns_is_editing_step[ step ] = kEnable;
                }
            }
            else
            {
                g_state.mns_select_mode = !g_state.mns_select_mode;

                for ( uint8_t step = 0; step < NUM_STEPS; step++ )
                {
                    if ( g_state.isTrackStepEnabled( g_state.mns_active_track, step ) )
                    {
                        g_state.mns_is_editing_step[ step ] = ( g_state.mns_select_mode == MODE_SELECT_ACTIVE ) ? 1 : 0;
                    }
                    else
                    {
                        g_state.mns_is_editing_step[ step ] = ( g_state.mns_select_mode == MODE_SELECT_ACTIVE ) ? 0 : 1;
                    }
                }
            }

            break;
        case OVERKILL_TRACK_MUTE_BUTTON:
            g_state.mns_mode = MODE_TRACK_MUTE;
            break;
        case OVERKILL_TRACK_SELECT_BUTTON:
            g_state.mns_mode = MODE_TRACK_SELECT;
            break;
        case OVERKILL_START_BUTTON:
            g_rs.rs_running = !g_rs.rs_running;
            break;
        case OVERKILL_RESET_BUTTON:
            MnResetSequencer();
            break;
        default:
            break;
        }

        if ( g_state.mns_mode == MODE_TRACK_SELECT )
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
        else if ( g_state.mns_mode == MODE_TRACK_MUTE )
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
            int8_t const kEditStep = msg[ 1 ] - LIVID_SEQ_ROW0;

            if ( g_state.mns_step_mode == MODE_STEP_EDIT )
            {

                g_state.mns_is_editing_step[ kEditStep ] = !g_state.isEditingStep( kEditStep );

                // If we just enabled it we need grab its values
                if ( g_state.isEditingStep( kEditStep ) )
                {
                    g_state.mns_gate_value = g_state.getTrackGate( g_state.mns_active_track, kEditStep );
                    g_state.mns_cv2_value  = g_state.getTrackCV2( g_state.mns_active_track, kEditStep );
                    g_state.mns_cv0_value  = g_state.getTrackCV0( g_state.mns_active_track, kEditStep );
                    g_state.mns_cv1_value  = g_state.getTrackCV1( g_state.mns_active_track, kEditStep );
                }
            }
            else if ( g_state.mns_step_mode == MODE_STEP_MUTE )
            {
                g_state.toggleBeat( g_state.mns_active_track, kEditStep );
            }
        }
    }
}


void MnInitCNTRLR()
{
    int const DELAY_TIME = 100;

    DEBUG( "Sending CNTRL:R sysex\n" );

#if 0
    // Factory reset
    DEBUG( "Sending factory reset\n" );
    {
        uint8_t sysex_factory_reset[] = { 
            LIVID_SYSEX, 0x06, MIDI_EOX
        };
        SENDMIDI_USB(sysex_factory_reset);
        delayLoop(1000);
    }
#endif

#if 1
    // Disable MIDI merge
    DEBUG( "Disabling MIDI merge\n" );
    {
        uint8_t sysex_factory_reset[] = { 
            LIVID_SYSEX, 0x0D, 0x00, MIDI_EOX
        };
        SENDMIDI_USB(sysex_factory_reset);
        delay(100);
    }
#endif


    // Map analog inputs

#if 0
    {
        uint8_t sysex_map_analog_inputs[] =
        {
            LIVID_SYSEX, 
            0x0A,

            0x01, 0x60, 0x01, 0x61, 0x01, 0x62, 0x01, 0x63,
            0x01, 0x64, 0x01, 0x65, 0x01, 0x66, 0x01, 0x67,
            0x01, 0x68, 0x01, 0x69, 0x01, 0x6A, 0x01, 0x6B,
            0x01, 0x6C, 0x01, 0x6D, 0x01, 0x6E, 0x01, 0x6F,

            0x01, 0x00, 0x01, 0x01, 0x01, 0x02, 0x01, 0x03,
            0x01, 0x04, 0x01, 0x05, 0x01, 0x06, 0x01, 0x07,
            0x01, 0x08, 0x01, 0x09, 0x01, 0x0A, 0x01, 0x0B,
            0x01, 0x0C, 0x01, 0x0D, 0x01, 0x0E, 0x01, 0x0F,

            MIDI_EOX
        };

        SENDMIDI_USB( sysex_map_analog_inputs );
        delay(DELAY_TIME);
    }
#endif

#if 1
    DEBUG("Setting encoders to fill mode\n" );
    // Set encoders to fill mode
    {
        uint8_t sysex_fill_mode[] =
        { LIVID_SYSEX, 
        0x1D,
        0x7F, 0x1, 0xF, 0x00,
        MIDI_EOX
        };

        SENDMIDI_USB(sysex_fill_mode);
        delay(100);
    }
#endif

    // Set encoding mode to relative (inc/dec)
#if 1
    DEBUG("Setting encoders to relative mode\n" );
    {
        uint8_t sysex_encosion_mode[] =
        { LIVID_SYSEX, 
        0x11,
        0x7F, 0x01, 0x0F, 0x00,
        MIDI_EOX
        };

        SENDMIDI_USB(sysex_encosion_mode);
        delay(100);
    }
#endif

    // Local encoder off
#if 1
    {
        DEBUG("Disabling encoder local off\n" );     
        uint8_t sysex_local_off_mode[] =
        { LIVID_SYSEX, 
        0x20, // Local ring control
        0x3,  // 0 = enabled
        MIDI_EOX
        };

        SENDMIDI_USB(sysex_local_off_mode);
        delay(100);
    }
#endif


#if 0
    int q = 0;
    while ( 1 )
    {
        q++;
        uint8_t table[8] = { 
            LIVID_COLOR_OFF,
            LIVID_COLOR_RED, 
            LIVID_COLOR_GREEN,
            LIVID_COLOR_BLUE,
            LIVID_COLOR_CYAN,
            LIVID_COLOR_YELLOW,
            LIVID_COLOR_MAGENTA,
            LIVID_COLOR_WHITE
        };

        // Update buttons
        for ( uint8_t i = LIVID_PAD00; i <= LIVID_PAD30; i++ )
        {
            MIDI_noteOn( i, table[q&0x7], MIDI_USB );
        }
//        MIDI_noteOn( LIVID_PAD33, 0, MIDI_USB );
        delay(1000);
    }
#elif 0

    int q = 2;

    do 
    {
        uint8_t sysex[] =
        {
            LIVID_SYSEX,
            0x04,
            0x7f,0x7f,0x7f,0x7f,0x3f,0x3f,0x3f,0x3f,
            0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
            0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
            0x52,0x52,0x52,0x52,0x52,0x52,
            MIDI_EOX 
        };
        SENDMIDI_USB(sysex);
        delayLoop(1000);
    } while ( --q );
    DEBUG("Done\n" );
/*
    while ( 1 ) 
    {
        Usb.Task();
    }
*/
#endif

}

void MnReconnectUSB()
{
    if ( Usb.getUsbTaskState() == USB_STATE_RUNNING )
        return;

    DEBUG("Reconnecting USB...\n" );
    Serial.begin(31500);

#if 0
    //workaround for non UHS2.0 Shield
    pinMode(7,OUTPUT);
    digitalWrite(7,HIGH);
#endif

    DEBUG("USB.Init()..." );
    if (Usb.Init() == -1) 
    {
        DEBUG("failed\n" );
        return;
    }
    DEBUG("success\n" );

    int attempts = 100;

    DEBUG("Waiting on USB to run");
    while ( Usb.getUsbTaskState() != USB_STATE_RUNNING )
    {
        DEBUG(".");
        Usb.Task();
        delay(100);
        if ( --attempts == 0 )
            break;
    }

    if ( Usb.getUsbTaskState() != USB_STATE_RUNNING )
    {
        DEBUG("failed\n" );
        return;
    }

    DEBUG("success!\n" );

    g_rs.rs_usb_connected = 1;
    MnInitCNTRLR();

    // Clear back and front buffers to alternate values which will force an update
    // of all backbuffer values
    g_device.resetBuffers();

    DEBUG("USB reconnection finished\n" );
}

void MnCheckUSB()
{
    static long s_last_check_time;
    long const kNow = millis();

    if ( !g_rs.rs_usb_connected )
    {
        MnReconnectUSB();
        return;
    }

    Usb.Task();

    if ( Usb.getUsbTaskState() == USB_STATE_RUNNING )
    {
        byte msgs[ 64 ];
        memset( msgs, 0, sizeof( msgs ) );
        uint16_t bytes_received = 0;

        if ( ( bytes_received = Midi.RcvData(msgs) ) != 0 )
        {
            sprintf( dbg_buffer, "IN: %ld[%d]: ", (long)millis(), (int)bytes_received);
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
    else
    {
        if ( g_rs.rs_usb_connected )
        {
            DEBUG("USB connection lost!\n" );
            g_rs.rs_usb_connected = 0;
        }

        if ( kNow - s_last_check_time > 1000 )
        {
            sprintf( dbg_buffer, "%ld: USB NOT FOUND\n", kNow );
            DEBUG( dbg_buffer );
        }
    }

    s_last_check_time = kNow;
}

void MnUpdateLCD()
{
    char clockbuf[ 32 ];
    char const *clock_src[] = { "Int", "Ext", "MIDI" };

    sprintf( clockbuf, "Clock: %s", clock_src[ g_rs.rs_temp_clock_source ] );

    lcd.clear();

    if ( g_rs.rs_main_menu == MENU_SPLASH )
    {
        lcd.print( "Modular Overkill 0.01" );
    }
    else if ( g_rs.rs_main_menu == MENU_CLOCK_SOURCE )
    {
        lcd.print( clockbuf );
    }
    else if ( g_rs.rs_main_menu == MENU_TRACK_CONFIG )
    {
        if ( g_rs.rs_temp_track_select == -1 )
        {
            lcd.print( "Track Config" );
        }
        else
        {
            char trackbuf[ 32 ];

            sprintf( trackbuf, "Track %d: A01", ( int ) g_rs.rs_temp_track_select );

            lcd.print( trackbuf );
        }
    }

    // Update LCD cursor
    if ( g_rs.rs_is_editing_config )
    {
        lcd.blink();
        lcd.cursor();

        if ( g_rs.rs_main_menu == MENU_CLOCK_SOURCE )
        {
            lcd.setCursor( 7, 0 );
        }
    }
    else 
    {
        lcd.noBlink(); 
        lcd.noCursor();
    }
}

void MnDoBackButton()
{
    g_rs.rs_is_editing_config = false;
    g_rs.rs_temp_track_select = -1;
    g_rs.rs_main_menu = MENU_SPLASH;

    // Update our state
    MnUpdateLCD();
}

void MnDoEncoderPush()
{
    if ( g_rs.rs_main_menu == MENU_CLOCK_SOURCE )
    {
        // Switch in/out of edit mode
        g_rs.rs_is_editing_config = !g_rs.rs_is_editing_config;

        // Entering edit mode, initialize our encoder
        if ( g_rs.rs_is_editing_config )
        {
            g_encoder = g_state.mns_clock_source;
            g_last_encoder = g_encoder;
        }
        // Exiting edit mode, store value
        else
        {
            g_state.mns_clock_source = ( ClockSource_t ) g_rs.rs_temp_clock_source;
        }
    }
    else if ( g_rs.rs_main_menu == MENU_TRACK_CONFIG )
    {
        // If we're not editing, then start editing
        if ( g_rs.rs_temp_track_select == -1 )
        {
            g_rs.rs_temp_track_select = 0;
            g_encoder = 0;
            g_last_encoder = g_encoder;
        }
        // Start editing track parameter
        else
        {
        }
    }

    // Update our state
    MnUpdateLCD();
}

void MnDoEncoder( uint8_t v )
{
    if ( ( g_rs.rs_is_editing_config ) && 
         ( g_rs.rs_main_menu == MENU_CLOCK_SOURCE ) )
    {
        g_rs.rs_temp_clock_source = v % CLOCK_MENU_LAST;
    }
    else if ( !g_rs.rs_is_editing_config )
    {
        g_rs.rs_main_menu = ( MainMenu_t ) ( (v>>2) % MENU_LAST );
    }

    MnUpdateLCD();
}

void MnCheckLocalControls()
{
    long const kNow = millis();

    // Encoder push button
    {
        bool was_down = g_rs.rs_encoder_down;

        g_rs.rs_encoder_down = !digitalRead( PIN_ENCODER_BUTTON );

        if ( !g_rs.rs_encoder_down && was_down )
        {
            MnDoEncoderPush();
        }
    }

    // Back button
    {
        bool was_down = g_rs.rs_back_down;

        g_rs.rs_back_down = !digitalRead( PIN_BACK_BUTTON );

        if ( !g_rs.rs_back_down && was_down )
        {
            MnDoBackButton();
        }
    }

    // Rotation
    if ( g_encoder != g_last_encoder )
    {
        MnDoEncoder( g_encoder );
        g_last_encoder = g_encoder;
        sprintf( dbg_buffer, "enc %d\n", g_encoder );
        DEBUG( dbg_buffer );
    }

    // Time to restore LCD?
    if ( g_rs.rs_lcd_restore_time && g_rs.rs_lcd_restore_time < kNow )
    {
        MnUpdateLCD();
        g_rs.rs_lcd_restore_time = 0;
    }
}

int getFreeRAM() 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void encoderISR()
{
    static uint8_t last;

    uint8_t now = ( digitalRead( PIN_ROTARY_PIN0 ) << 1 ) | digitalRead( PIN_ROTARY_PIN1 );

    //  CW:  0 2 3 1
    //  CCW: 0 1 3 2
    if ( ( last == 0 && now == 2 ) ||
         ( last == 2 && now == 3 ) ||
         ( last == 3 && now == 1 ) ||
         ( last == 1 && now == 0 ) )
    {
        g_encoder++;
    }
    else if ( last != now )
    {
        g_encoder--;
    }
    last = now;
}

// the setup routine runs once when you press reset:
void setup() 
{
    MnUpdateLCD();

    MnLoadState();

    // initialize the LED pin
    pinMode( PIN_LED, OUTPUT);
    digitalWrite( PIN_LED,HIGH);

    // Set up the rotary encoder
    pinMode( PIN_ENCODER_BUTTON, INPUT );
    digitalWrite( PIN_ENCODER_BUTTON, HIGH );

    pinMode( PIN_ROTARY_PIN0, INPUT );
    digitalWrite( PIN_ROTARY_PIN0, HIGH );
    pinMode( PIN_ROTARY_PIN1, INPUT );
    digitalWrite( PIN_ROTARY_PIN1, HIGH );

    pinMode( PIN_BACK_BUTTON, INPUT );
    digitalWrite( PIN_BACK_BUTTON, HIGH );

    // Interrupt #, not pin, so don't use pin constant!
    attachInterrupt( 2, encoderISR, CHANGE );
    attachInterrupt( 3, encoderISR, CHANGE );
  
    for ( unsigned char i = 0; i < NUM_TRIGGERS; i++ )
    {
        pinMode( PIN_TRIGGER0 + i, OUTPUT );
        digitalWrite( PIN_TRIGGER0 + i, LOW );
    }

    // This is debug monitor
    Serial1.begin(57600);

    sprintf( dbg_buffer, "sizeof( g_state ) == %d\n", sizeof( g_state ) );
    DEBUG( dbg_buffer );
    sprintf( dbg_buffer, "free mem == %d\n", getFreeRAM() );
    DEBUG( dbg_buffer );

    // This is the MIDI OUT port
    DEBUG( "Initializing MIDI OUT\n" );
    Serial2.begin( 31250 );

    // Config SD library
    DEBUG("Initialized SD..." );
    if ( !SD.begin( PIN_SPI_MICROSD ) )
    {
       DEBUG( "failed\n" );
    }
    else
    {
        DEBUG( "success\n" );
    }
}

void loop() 
{
    unsigned long const kBeatDurationMicros    = 600000000 / g_state.mns_bpm_times_10; // 1/4  note
    unsigned long const kMicrosPerThirtySecond = kBeatDurationMicros >> 3;   // 1/32 note
    unsigned long const kNow = millis();

    MnCheckSequence( kMicrosPerThirtySecond );

    // Go through all of our state and make sure the LEDs
    // reflect what's going on
    MnUpdateState( kNow );

    MnCheckLocalControls();

    MnCheckUSB();

    g_device.swapBuffers();
}
