#define LIVID_SYSEX 0xF0, 0x00, 0x01, 0x61, 0x08

// All the following are CC message values
#define LIVID_ROTARY00    1
#define LIVID_ROTARY01    5
#define LIVID_ROTARY02    9
#define LIVID_ROTARY03    13
#define LIVID_ROTARY10    2
#define LIVID_ROTARY11    6
#define LIVID_ROTARY12    10
#define LIVID_ROTARY13    14
#define LIVID_ROTARY20    3
#define LIVID_ROTARY21    7
#define LIVID_ROTARY22    11
#define LIVID_ROTARY23    15

#define LIVID_ROTARY04    17
#define LIVID_ROTARY05    21
#define LIVID_ROTARY06    25
#define LIVID_ROTARY07    29
#define LIVID_ROTARY14    18
#define LIVID_ROTARY15    22
#define LIVID_ROTARY16    26
#define LIVID_ROTARY17    30
#define LIVID_ROTARY24    19
#define LIVID_ROTARY25    23
#define LIVID_ROTARY26    28
#define LIVID_ROTARY27    31

#define LIVID_FADER0 4
#define LIVID_FADER1 8
#define LIVID_FADER2 12
#define LIVID_FADER3 16
#define LIVID_FADER4 20
#define LIVID_FADER5 24
#define LIVID_FADER6 28
#define LIVID_FADER7 32

// All the following are Note message values
#define LIVID_PAD00 0
#define LIVID_PAD01 4
#define LIVID_PAD02 8
#define LIVID_PAD03 12
#define LIVID_PAD10 1
#define LIVID_PAD11 5
#define LIVID_PAD12 9
#define LIVID_PAD13 13
#define LIVID_PAD20 2
#define LIVID_PAD21 6
#define LIVID_PAD22 10
#define LIVID_PAD23 14
#define LIVID_PAD30 3
#define LIVID_PAD31 7
#define LIVID_PAD32 11
#define LIVID_PAD33 15

#define LIVID_SEQ_ROW0 16
#define LIVID_SEQ_ROW1 32

// NOTE: These double as both CC and note on parameters
#define LIVID_ENCODER00 48
#define LIVID_ENCODER01 51
#define LIVID_ENCODER02 54
#define LIVID_ENCODER03 57
#define LIVID_ENCODER10 49
#define LIVID_ENCODER11 52
#define LIVID_ENCODER12 55
#define LIVID_ENCODER13 58
#define LIVID_ENCODER20 50
#define LIVID_ENCODER21 53
#define LIVID_ENCODER22 56
#define LIVID_ENCODER23 59

#define LIVID_NUM_ENCODERS       12

// Color values
#define LIVID_COLOR_OFF 0
#define LIVID_COLOR_1   1
#define LIVID_COLOR_2   4
#define LIVID_COLOR_3   8
#define LIVID_COLOR_4   16
#define LIVID_COLOR_5   32
#define LIVID_COLOR_6   64
#define LIVID_COLOR_7   127

#define LIVID_COLOR_OFF     0x00
#define LIVID_COLOR_WHITE   0x01
#define LIVID_COLOR_CYAN    0x04
#define LIVID_COLOR_MAGENTA 0x08
#define LIVID_COLOR_RED     0x10
#define LIVID_COLOR_BLUE    0x20
#define LIVID_COLOR_YELLOW  0x40
#define LIVID_COLOR_GREEN   0x7F
