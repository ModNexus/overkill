// Schematics:
//
// 
// TODO: gate length control
// TODO: CV outputs (manual)
// TODO: pattern select
// TODO: analog clock input
// TODO: MIDI IN (for clocking)?
// TODO: CV outputs (sequenced)
// TODO: SD card
// TODO: LCD support
#include <avr/pgmspace.h>
#include <Usb.h>
#include <usbh_midi.h>
#include "livid.h"
#include "overkill.h"

char  dbg_buffer[ 128 ];
bool  g_sysex_received;

USB   Usb;
MIDI  Midi(&Usb);

void MIDI_noteOn( uint8_t n, uint8_t vel, uint8_t port )
{
    uint8_t  buffer[3] =
    {
        MIDI_NOTE_ON, n, vel
    };

    if ( port == MIDI_USB )
    {
        sprintf( dbg_buffer, "[%08lud] Note on  0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
        DEBUG( dbg_buffer );
        SENDMIDI_USB(buffer);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
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
        sprintf( dbg_buffer, "[%08lud] CC 0x%x 0x%x\n", millis(), (int)n,(int)val );
        DEBUG( dbg_buffer );
        SENDMIDI_USB(buffer);
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
    sprintf( dbg_buffer, "[%08lud] Note off 0x%02x,0x%02x\n", millis(), (int)n,(int)vel );
    DEBUG( dbg_buffer );
        SENDMIDI_USB(buffer);
    }
    else
    {
        SENDMIDI_MIDI(buffer,sizeof(buffer));
    }
}


struct LividCNTRLR
{
    uint8_t lc_led_button_backbuffer[ 60 ];   // These are what we update
    uint8_t lc_led_button_frontbuffer[ 60 ];  // These are what we're currently showing
    uint8_t lc_led_encoder_backbuffer[ LIVID_NUM_ENCODERS ];  // These are what we update
    uint8_t lc_led_encoder_frontbuffer[ LIVID_NUM_ENCODERS ]; // These are what we're currently showing

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

struct MnState
{
    uint8_t mns_usb_connected;

    //
    // Timer stuff
    //
    MnTimerEvent  mns_timer_events[ MAX_TIMER_EVENTS ];
    void addTimerEvent( uint8_t const kType, uint8_t const kParam0, uint8_t const kParam1, uint32_t const kWhen )
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

    MnTrack       mns_tracks[ NUM_TRACKS ];

    uint8_t       getStep( void ) { return mns_beat & ( NUM_STEPS - 1 ); }

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

    // Toggles the beat
    void toggleBeat( uint8_t const kTrack, uint8_t const kStep )
    {
        bool const kEnabled = mns_tracks[ kTrack ].isStepEnabled( kStep );

        mns_tracks[ kTrack ].enableStep( kStep, !kEnabled );
    }

    // Set sane default track values
    MnState()
    {
        mns_edit_step = -1;
        mns_running = false;
    }
};

MnState g_state;

/*
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
*/


void FE_SetActiveStep( uint8_t const kStep )
{
    g_device.setButtonLED( kStep + LIVID_SEQ_ROW0, 1 );
    g_device.setEncoderLED( LIVID_ENCODER00, (kStep+1)*7 );
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

    g_device.clearScreen();

    // Advance our controller's beat
    FE_SetActiveStep( kStep );

    // Make sure our mode is up to date
    g_device.setButtonLED( LIVID_PAD30, LIVID_COLOR_YELLOW );
    g_device.setButtonLED( LIVID_PAD31, LIVID_COLOR_YELLOW );
    g_device.setButtonLED( LIVID_PAD32, LIVID_COLOR_YELLOW );
    g_device.setButtonLED( LIVID_PAD33, LIVID_COLOR_YELLOW );

    switch ( g_state.mns_mode )
    {
    case MODE_CONTROL:
        g_device.setButtonLED( LIVID_PAD30, LIVID_COLOR_BLUE );
        break;
    case MODE_TRACK:
        g_device.setButtonLED( LIVID_PAD31, LIVID_COLOR_BLUE );
        break;
    case MODE_MUTE:
        g_device.setButtonLED( LIVID_PAD32, LIVID_COLOR_BLUE );
        break;
    case MODE_PITCH:
        g_device.setButtonLED( LIVID_PAD33, LIVID_COLOR_BLUE );
        break;
    }

    // Update encoder
    if ( g_state.mns_edit_step >= 0 )
    {
        g_device.setEncoderLED( LIVID_ENCODER00, g_state.getTrackNote( g_state.mns_active_track, g_state.mns_edit_step ) );
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
            g_device.setButtonLED( LIVID_PAD00, LIVID_COLOR_OFF );  // start
        }
        else
        {
            g_device.setButtonLED( LIVID_PAD00, LIVID_COLOR_GREEN );  // start
        }
        g_device.setButtonLED( LIVID_PAD01, LIVID_COLOR_RED );    // reset
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

            g_device.setButtonLED( g_device.trackToPad( i ), color );
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

        if ( g_state.isTrackStepEnabled( g_state.mns_active_track, step ) )
        {
            g_device.setButtonLED( LIVID_SEQ_ROW1 + step, color );
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
                g_state.addTimerEvent( TET_ANALOG_LOW, PIN_TRIGGER0 + track, 0, kNowMS + 20 );
            }
            else
            {
                MIDI_noteOn( kNote, 0x40, MIDI_EXT );
                g_state.addTimerEvent( TET_MIDI_NOTE_OFF, kNote, 0, kNowMS + 5 );
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

    // Sysex?
    if ( msg[ 0 ] == MIDI_SYSEX )
    {
        g_sysex_received = 1;
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
        int8_t const kFaderIndex = g_device.faderIndexFromCC( msg[ 1 ] );

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

void WaitForAck()
{
    g_sysex_received = 0;
    DEBUG("Waiting for ack");

    for ( int i = 0; i < 100; i++ )
    {
        Usb.Task();
        delay(10);
        DEBUG(".");
    }
    DEBUG("\n");
}

void MnInitCNTRLR()
{
    DEBUG( "Sending CNTRL:R sysex\n" );

#if 1
    // Factory reset
    DEBUG( "Sending factory reset\n" );
    {
        uint8_t sysex_factory_reset[] = { 
            LIVID_SYSEX, 0x06, MIDI_EOX
        };
        SENDMIDI_USB(sysex_factory_reset);

        WaitForAck();
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

        WaitForAck();
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
        delay(100);

        WaitForAck();
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

//        WaitForAck();
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

        WaitForAck();
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

        WaitForAck();
    }
#endif

}

void MnReconnectUSB()
{
    if ( Usb.getUsbTaskState() == USB_STATE_RUNNING )
        return;

    DEBUG("Reconnecting USB...\n" );
    Serial.begin(31500);

    //workaround for non UHS2.0 Shield
    pinMode(7,OUTPUT);
    digitalWrite(7,HIGH);

    DEBUG("USB.Init()..." );
    if (Usb.Init() == -1) 
    {
        DEBUG("failed\n" );
        return;
    }
    DEBUG("success\n" );

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

    DEBUG("success!\n" );

    delay(10);

    g_state.mns_usb_connected = 1;
    MnInitCNTRLR();

    // Clear back and front buffers to alternate values which will force an update
    // of all backbuffer values
    memset( &g_device.lc_led_button_backbuffer, 0, sizeof( g_device.lc_led_button_backbuffer ) );
    memset( &g_device.lc_led_encoder_backbuffer, 0, sizeof( g_device.lc_led_encoder_backbuffer ) );
    memset( &g_device.lc_led_button_frontbuffer, 0x1, sizeof( g_device.lc_led_button_frontbuffer ) );
    memset( &g_device.lc_led_encoder_frontbuffer, 0x1, sizeof( g_device.lc_led_encoder_frontbuffer ) );

    DEBUG("USB reconnection finished\n" );
}

void MnCheckUSB()
{
    static long s_last_check_time;
    long const kNow = millis();

    if ( !g_state.mns_usb_connected )
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

//        while ( Midi.RcvData(&bytes_received,msgs) == 0 )
        if ( ( bytes_received = Midi.RcvData(msgs) ) != 0 )
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
        if ( g_state.mns_usb_connected )
        {
            DEBUG("USB connection lost!\n" );
            g_state.mns_usb_connected = 0;
        }

        if ( kNow - s_last_check_time > 1000 )
        {
            sprintf( dbg_buffer, "%ld: USB NOT FOUND\n", kNow );
            DEBUG( dbg_buffer );
        }
    }

    s_last_check_time = kNow;
}

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

    MnCheckLocalControls();

    MnCheckUSB();

    g_device.swapBuffers();
}
