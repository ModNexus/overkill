// Schematics:
//
// TODO: SD shield
// TODO: analog clock input/output/reset
// TODO: Num steps per track should be configurable to 32
// TODO: pattern copy

//#define USE_SD 1
#ifdef USE_SD
#include <SD.h>
#endif

#include <avr/pgmspace.h>
#include <Usb.h>
#include <usbh_midi.h>
#include "livid.h"
#include "overkill.h"

#define CLAMP(a,b,c) ((a)=min(max(a,b),c))

static char const *OVERKILL_FNAME = "overkill.bin";

char  dbg_buffer[ 1024 ];
USB   Usb;
MIDI  Midi(&Usb);

volatile unsigned int g_clock;

void MIDI_noteOn( uint8_t n, uint8_t vel, uint8_t port, uint8_t channel )
{
    uint8_t  buffer[3] =
    {
        MIDI_NOTE_ON|channel, n, vel
    };

    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
//        sprintf( dbg_buffer, "[%08lu] Note on  0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
//        DEBUG( dbg_buffer );
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
        sprintf( dbg_buffer, "[%08lu] Note on  0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
        DEBUG( dbg_buffer );
    }
}

void MIDI_cc( uint8_t n, uint8_t val, uint8_t port, uint8_t channel )
{
    uint8_t buffer[3] =
    {
        MIDI_CC|channel, n, val
    };

    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
//        delay(1);
//        sprintf( dbg_buffer, "[%08lu] CC 0x%x 0x%x\n", millis(), (int)n,(int)val );
//        DEBUG( dbg_buffer );
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
}
void MIDI_noteOff( uint8_t n, uint8_t vel, uint8_t port, uint8_t channel )
{
    uint8_t buffer[3] =
    {
        MIDI_NOTE_OFF|channel, n, vel
    };
    if ( port == MIDI_USB )
    {
        SENDMIDI_USB(buffer);
//        delay(1);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
        sprintf( dbg_buffer, "[%08lu] Note off 0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
        DEBUG( dbg_buffer );
    }
}

struct LividBASE
{
    int8_t lb_led_button_backbuffer[ 76 ];
    int8_t lb_led_button_frontbuffer[ 76 ];

    int8_t ccToFaderIndex( int8_t const kCC )
    {
        switch ( kCC )
        {
        case LIVID_FADER0:
        case LIVID_FADER1:
        case LIVID_FADER2:
        case LIVID_FADER3:
        case LIVID_FADER4:
        case LIVID_FADER5:
        case LIVID_FADER6:
        case LIVID_FADER7:
        case LIVID_FADER8:
            return kCC - LIVID_FADER0;
        }
        return 0;
    }

    void resetBuffers( void )
    {
        // Set buffers opposite so we force a clear
        memset( lb_led_button_backbuffer, LIVID_COLOR_WHITE, sizeof( lb_led_button_backbuffer ) );
        memset( lb_led_button_frontbuffer, 0xFF, sizeof( lb_led_button_frontbuffer ) );

        const char c[] =
        {
            LIVID_COLOR_OFF,
            LIVID_COLOR_WHITE,
            LIVID_COLOR_RED,
            LIVID_COLOR_BLUE,
            LIVID_COLOR_GREEN,
            LIVID_COLOR_CYAN,
            LIVID_COLOR_MAGENTA,
            LIVID_COLOR_YELLOW
        };
        for ( int i = 1; i < 76; i++ )
        {
            MIDI_noteOn( i, c[i&7], MIDI_USB, 0 );
        }

    }
    int8_t stepToPad( int8_t const kStep )
    {
        if ( kStep < 8 )
        {
            return LIVID_PAD00 + kStep;
        }
        else if ( kStep < 16 )
        {
            return LIVID_PAD10 + kStep - 8;
        }
        else if ( kStep < 24 )
        {
            return LIVID_PAD20 + kStep - 16;
        }
        return LIVID_PAD30 + kStep - 24;
    }
    void clearScreen( void )
    {
        memset( lb_led_button_backbuffer, 0, sizeof( lb_led_button_backbuffer ) );
    }
    void swapBuffers( void )
    {
        for ( int i = 1; i < 76; i++ )
        {
            if ( lb_led_button_backbuffer[ i ] != lb_led_button_frontbuffer[ i ] )
            {
                MIDI_noteOn( i, lb_led_button_backbuffer[ i ], MIDI_USB, 0 );
            }
            lb_led_button_frontbuffer[ i ] = lb_led_button_backbuffer[ i ];
        }
    }
    void setLED( uint8_t const kWhich, int8_t const kCC )
    {
        lb_led_button_backbuffer[ kWhich ] = kCC;
    }
};

LividBASE g_device;

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
    uint8_t         mnt_current_note;

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
    uint8_t    rs_usb_connected;
    uint16_t   rs_beat;
    uint32_t   rs_last_beat_time;
    bool       rs_running;
    uint8_t    getStep( void ) { return rs_beat & ( NUM_STEPS - 1 ); }

    //
    // fader stuff
    //
    int8_t rs_fader_state[ 9 ];

    int8_t getFaderState( int8_t const kFaderIndex )
    {
        return rs_fader_state[ kFaderIndex ];
    }
    void setFaderState( int8_t const kFaderIndex, int8_t const kValue )
    {
        rs_fader_state[ kFaderIndex ] = kValue;
    }

    //
    // button stuff
    //
    uint8_t rs_button_state[ 10 ]; // Assumes up to 80, BASE has 76
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
    }
};

MnRunningState g_rs;

struct MnState
{
    uint32_t mns_cookie;
    uint16_t mns_version;

    ClockSource_t mns_clock_source;

    //
    // misc state
    //
    uint16_t      mns_bpm_times_10;  // scaled BPM
    uint8_t       mns_active_track;

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

    bool isTrackMuted( uint8_t const kTrack )
    {
        return mns_tracks[ kTrack ].mnt_track_muted;
    }

    // Returns value irrespective of mute
    uint8_t getTrackGate( uint8_t const kTrack, uint8_t const kStep )
    {
        return mns_tracks [ kTrack ].mnt_step_gate[ kStep ];
    }
    uint8_t getTrackNoteOn( uint8_t const kTrack )
    {
        if ( mns_tracks [ kTrack ].mnt_gate_off_time > 0 )
        {
            return mns_tracks[ kTrack ].mnt_current_note;
        }
        return 0;
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

    // Set sane default track values
    MnState() : mns_cookie( OVERKILL_MAGIC_COOKIE ), mns_version( OVERKILL_STATE_VERSION )
    {
        mns_bpm_times_10 = 1200;

        // Set default gate length for drum tracks to 
        for ( uint8_t t = 0; t < NUM_TRIGGERS; t++ )
        {
            for ( uint8_t s = 0; s < NUM_STEPS; s++ )
            {
                setTrackGate( t, s, 1 ); // 1/32nd note
            }
        }

        // Set default gate length for pitch tracks to 1/4 note
        for ( uint8_t t = 8; t < NUM_TRACKS; t++ )
        {
            for ( uint8_t s = 0; s < NUM_STEPS; s++ )
            {
                setTrackGate( t, s, 1 ); // 1/4 note
            }
        }

        // Set default pitch
        for ( uint8_t t = 8; t < NUM_TRACKS; t++ )
        {
            for ( uint8_t s = 0; s < NUM_STEPS; s++ )
            {
                setTrackCV0( t, s, 0x3C ); // 1/4 note
            }
        }

    }
};

MnState g_state;

#if 0
struct MnTimerEvent
{
    uint8_t       te_type;
    unsigned long te_trigger_time_ms;
    uint8_t       te_param0;
    uint8_t       te_param1;
};

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
    DEBUG("Loading state\n" );

#ifdef USE_SD
    g_state = MnState();

    File fp = SD.open( OVERKILL_FNAME, FILE_READ );

    if ( fp )
    {
        DEBUG("Found data file, using\n" );

        MnState old_state;
        memset( &old_state, 0, sizeof( old_state ) );
        fp.read( &old_state, sizeof( old_state ) );
        fp.close();

        if ( old_state.mns_cookie == OVERKILL_MAGIC_COOKIE &&
            old_state.mns_version == OVERKILL_STATE_VERSION )
        {
            DEBUG("Valid data found!\n" );
            g_state = old_state;
        }
    }
#endif
}

void MnSaveState()
{
    DEBUG("Saving state\n" );

#ifdef USE_SD
    // Save patterns to SD
    File fp = SD.open( OVERKILL_FNAME, FILE_WRITE );

    if ( fp )
    {
        fp.write( reinterpret_cast<uint8_t const *>(&g_state), sizeof( g_state ) );
        fp.close();
    }
#endif
}

void MnUpdateState( unsigned long kMillis )
{
    uint8_t const kStep = g_rs.getStep();
    bool const kShiftSystem = g_rs.getButtonState( OVERKILL_SHIFT_SYSTEM_BUTTON );

    g_device.clearScreen();

    //
    // Draw start button
    //
    g_device.setLED( OVERKILL_START_BUTTON, LIVID_COLOR_GREEN );
    if ( g_rs.rs_running )
    {
        if  ( ( g_rs.getStep() & 3 ) == 0 )
        {
            g_device.setLED( LIVID_F1_LED, LIVID_COLOR_GREEN );
        }
    }
    //
    // Draw BPM button
    //
    g_device.setLED( OVERKILL_BPM_BUTTON, LIVID_COLOR_BLUE );
    if ( g_rs.rs_running )
    {
        if  ( ( g_rs.getStep() & 3 ) == 0 )
        {
            g_device.setLED( LIVID_F3_LED, LIVID_COLOR_BLUE );
        }
    }

    //
    // Draw buttons
    //
    g_device.setLED( OVERKILL_TRACK_MUTE_BUTTON, LIVID_COLOR_RED );
    g_device.setLED( OVERKILL_SHIFT_SYSTEM_BUTTON, LIVID_COLOR_YELLOW );

    //
    // Draw pads
    //
    for ( int i = 0; i < NUM_STEPS; i++ )
    {
        if ( g_state.isTrackStepEnabled( g_state.mns_active_track, i ) )
        {
            g_device.setLED( g_device.stepToPad( i ), LIVID_COLOR_GREEN );
        }
    }

    //
    // Draw track buttons
    //
    for ( uint8_t i = 0; i < NUM_TRACKS; i++ )
    {
        int8_t color = LIVID_COLOR_OFF;

        if ( g_state.mns_active_track == i )
        {
            color = LIVID_COLOR_GREEN;
        }
        else if ( g_state.isTrackStepEnabled( i, g_rs.getStep() ) )
        {
//            sprintf(dbg_buffer,"track %d has gate @ step %d\n", (int)i, (int)g_rs.getStep()) ;
//            DEBUG(dbg_buffer);
            color = LIVID_COLOR_CYAN;
        }
        else if ( g_state.trackHasAnyData( i ) )
        {
            color = LIVID_COLOR_BLUE;
        }

        g_device.setLED( LIVID_TRACK_BUTTON0 + i, color );
        if ( g_state.isTrackMuted( i ) )
        {
            g_device.setLED( LIVID_TRACK_BUTTON0_LED + i, LIVID_COLOR_RED );
        }
        else
        {
            g_device.setLED( LIVID_TRACK_BUTTON0_LED + i, LIVID_COLOR_YELLOW );
        }
    }

    //
    // Update sequencer
    //
    for ( uint8_t step = 0; step < NUM_STEPS; step++ )
    {
        // Advance our controller's beat
        if ( step == kStep )
        {
            g_device.setLED( g_device.stepToPad(step), LIVID_COLOR_WHITE );
        }
    }

    //
    // Draw system buttons
    //
    if ( kShiftSystem )
    {
        g_device.setLED( OVERKILL_SYSTEM_SAVE_BUTTON, LIVID_COLOR_GREEN ); // save
        g_device.setLED( OVERKILL_SYSTEM_LOAD_BUTTON, LIVID_COLOR_RED );   // load
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
                sprintf( dbg_buffer, "Pin %d high\n", (int)track );
                DEBUG( dbg_buffer );
                digitalWrite( PIN_TRIGGER0 + track, HIGH );
                kTrack.mnt_gate_on_time = kNowMS;
                kTrack.mnt_gate_off_time = kNowMS + kGateDurationMS;
            }
            // Time to gate off?
            else if ( kTrack.mnt_gate_off_time > 0 && kTrack.mnt_gate_off_time < kNowMS )
            {
                sprintf( dbg_buffer, "Pin %d low\n", (int)track );
                DEBUG( dbg_buffer );
                digitalWrite( PIN_TRIGGER0 + track, LOW );
                kTrack.mnt_gate_off_time = kTrack.mnt_gate_on_time = 0;
            }
        }
        // Tracks 8+ are MIDI or CV
#if 0
        else
        {
            // See if we have to trigger gate on
            if ( kGate && kIsNewBeat && kTrack.isStepEnabled( kStep ) )
            {
                uint8_t curnote = g_state.getTrackNoteOn( track );

                // Already a note on?
                if ( curnote )
                {
                    MIDI_noteOn( curnote, 0x0, MIDI_EXT, 0  ); //TODO: Add channel here
                }

                MIDI_noteOn( kCV0, 0x40, MIDI_EXT, 0 ); //TODO: Add channel here
                MIDI_cc( 14, kCV1, MIDI_EXT, 0 ); //TODO: Add channel here
                MIDI_cc( 15, kCV2, MIDI_EXT, 0 ); //TODO: Add channel here
                kTrack.mnt_gate_on_time = kNowMS;
                kTrack.mnt_gate_off_time = kNowMS + kGateDurationMS;
                kTrack.mnt_current_note = kCV0;
            }
            // Time to gate off?
            else if ( kTrack.mnt_gate_off_time > 0 && kTrack.mnt_gate_off_time < kNowMS )
            {
                MIDI_noteOff( kTrack.mnt_current_note, 0, MIDI_EXT, 0 ); // TODO: Add channel here
                kTrack.mnt_gate_off_time = kTrack.mnt_gate_on_time = 0;
                kTrack.mnt_current_note = 0;
            }
        }
#endif
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
    }

    // Sysex?
    if ( msg[ 0 ] == MIDI_SYSEX )
    {
    }

    //
    // BPM control
    // 
    if ( g_rs.getButtonState( OVERKILL_BPM_BUTTON ) )
    {
        uint16_t bpm = g_state.mns_bpm_times_10;

        g_state.mns_bpm_times_10 = g_rs.getFaderState( 8 ) * 20 + 300;

        if ( g_state.mns_bpm_times_10 < 300 )
            g_state.mns_bpm_times_10 = 300;
        else if ( g_state.mns_bpm_times_10 > 2500 )
            g_state.mns_bpm_times_10 = 2500;

        if ( bpm != g_state.mns_bpm_times_10 )
        {
            sprintf( dbg_buffer, "New bpm %d\n", g_state.mns_bpm_times_10/10);
            DEBUG( dbg_buffer );
        }
    }

    if ( msg[ 0 ] == MIDI_CC )
    {
        int8_t const kFaderIndex = g_device.ccToFaderIndex( msg[ 1 ] );

        if ( kFaderIndex >= 0 && kFaderIndex < 9 )
        {
            g_rs.setFaderState( kFaderIndex, msg[ 2 ] );
        }
    }
    else if ( msg[ 0 ] == MIDI_NOTE_ON )
    {
        bool const kShiftMute = g_rs.getButtonState( OVERKILL_TRACK_MUTE_BUTTON ) != 0;
        bool const kShiftSystem = g_rs.getButtonState( OVERKILL_SHIFT_SYSTEM_BUTTON ) != 0;
        if ( msg[ 1 ] == OVERKILL_START_BUTTON )
        {
            if ( !g_rs.rs_running )
                MnStartSequencer();
            else
                g_rs.rs_running = false;
        }
        else if ( msg[ 1 ] == OVERKILL_RESET_BUTTON )
        {
            MnResetSequencer();
        }
        else if ( msg[ 1 ] >= LIVID_TRACK_BUTTON0 && msg[ 1 ] <= LIVID_TRACK_BUTTON7 )
        {
            int const kTrack = msg[ 1 ] - LIVID_TRACK_BUTTON0;

            if ( kTrack >= 0 )
            {
                if ( kShiftMute )
                {
                    g_state.mns_tracks[ kTrack ].mnt_track_muted = !g_state.mns_tracks[ kTrack ].mnt_track_muted;
                    sprintf( dbg_buffer, "track %d mute %d\n", kTrack, ( int ) g_state.mns_tracks[ kTrack ].mnt_track_muted );
                    DEBUG( dbg_buffer );
                }
                else
                {
                    g_state.mns_active_track = kTrack;
                }
            }
        }
        if ( msg[ 1 ] >= LIVID_PAD00 && msg[ 1 ] <= LIVID_PAD07 )
        {
            g_state.toggleBeat( g_state.mns_active_track, msg[ 1 ] - LIVID_PAD00 );
        }
        else if ( msg[ 1 ] >= LIVID_PAD10 && msg[ 1 ] <= LIVID_PAD17 )
        {
            g_state.toggleBeat( g_state.mns_active_track, msg[ 1 ] + 8 - LIVID_PAD10 );
        }
        else if ( kShiftSystem && msg[ 1 ] == OVERKILL_SYSTEM_SAVE_BUTTON )
        {
            MnSaveState();
        }
        else if ( kShiftSystem && msg[ 1 ] == OVERKILL_SYSTEM_LOAD_BUTTON )
        {
            MnLoadState();
        }
    }
}

void MnInitBASE()
{
    DEBUG( "Initializing BASE\n" );

    g_rs.setFaderState( 8, ( g_state.mns_bpm_times_10 - 300 ) / 20 );

    // Disable local control
//    MIDI_cc( 122, 65, MIDI_USB, 15 );
}

#ifdef REMOVE_THIS
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
#endif

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

    MnInitBASE();

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
        if ( ( kNow & 1023 ) == 0 )
        {
            DEBUG( "USB running\n" );
        }

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

int getFreeRAM() 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void clockInISR()
{
   g_clock++;
}

// the setup routine runs once when you press reset:
void setup() 
{
    // initialize the LED pin
    pinMode( PIN_LED, OUTPUT);
    digitalWrite( PIN_LED,HIGH);

    // Set up the rotary encoder
    pinMode( PIN_CLKIN, INPUT );
    digitalWrite( PIN_CLKIN, HIGH );

    // Interrupt #, not pin, so don't use pin constant!
//    attachInterrupt( 2, encoderISR, CHANGE );
    attachInterrupt( 3, clockInISR, CHANGE );
  
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
#if 0
    DEBUG( "Initializing MIDI OUT\n" );
    Serial2.begin( 31250 );
#endif

    // Config SD library
#ifdef USE_SD
    DEBUG("Initializing SD..." );
    if ( !SD.begin( PIN_SPI_MICROSD ) )
    {
       DEBUG( "failed\n" );
    }
    else
    {
        DEBUG( "success\n" );
    }
#endif

    MnLoadState();
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

    MnCheckUSB();

    g_device.swapBuffers();

    if ( ( kNow & 1023 ) == 0 )
    {
        DEBUG("Swap!\n");
    }
}
