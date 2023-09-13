#define FUDGEFACTORNOTE 1
#define C4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/440.00))
#define BFLAT4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/466.16))
#define B4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/659.25))
#define EFLAT5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/622.26))
#define F5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/698.46))
#define F5SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/739.99))
#define G5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/880.00))
#define BFLAT5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/932.33))
#define B5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/987.77))
#define C6NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1046.5))
#define C6SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1108.73))
#define D6NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1174.66))
#define E6NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1318.51))
#define F6NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1396.91))
#define F6SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1479.98))
#define G6NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/1567.98))
#define F4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/369.99))
#define G4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))
#define A4FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))
#define C5SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/554.37))
#define A5FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/830.61))


#define OFFNOTE 1

#define SONG_LENGTH 164
uint16_t songarray[SONG_LENGTH] = {
A5NOTE,
B5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
G5NOTE,
F5SHARPNOTE,
F5SHARPNOTE,
OFFNOTE,
OFFNOTE,
F5SHARPNOTE,
F5SHARPNOTE,
G5NOTE,
G5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
G5NOTE,
OFFNOTE,
OFFNOTE,
F5SHARPNOTE,
F5SHARPNOTE,
OFFNOTE,
OFFNOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
OFFNOTE,
B5NOTE,
B5NOTE,
OFFNOTE,
OFFNOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
F5SHARPNOTE,
G5NOTE,
A5NOTE,
G5NOTE,
A5NOTE,
G5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
F5SHARPNOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
OFFNOTE,
A5NOTE,
B5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
G5NOTE,
F5SHARPNOTE,
F5SHARPNOTE,
OFFNOTE,
OFFNOTE,
F5SHARPNOTE,
F5SHARPNOTE,
G5NOTE,
G5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
G5NOTE,
OFFNOTE,
OFFNOTE,
F5SHARPNOTE,
F5SHARPNOTE,
OFFNOTE,
OFFNOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
OFFNOTE,
B5NOTE,
B5NOTE,
OFFNOTE,
OFFNOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
F5SHARPNOTE,
G5NOTE,
A5NOTE,
G5NOTE,
A5NOTE,
G5NOTE,
A5NOTE,
A5NOTE,
G5NOTE,
F5SHARPNOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
OFFNOTE,
C6NOTE,
D6NOTE,
B5NOTE,
B5NOTE,
B5NOTE,
B5NOTE,
B5NOTE,
B5NOTE,
B5NOTE,
C6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
D6NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
G5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
B5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
A5NOTE,
};

#define SONG2_LENGTH 400
uint16_t song2array[SONG2_LENGTH] = {
F4NOTE, //war
F4NOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
D5NOTE, //ea
D5NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE, //gle
C5NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE, //fly
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
OFFNOTE,
OFFNOTE,
G4NOTE,
OFFNOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
A4NOTE,
OFFNOTE,
A4NOTE,
A4NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
OFFNOTE,
OFFNOTE,
G4NOTE,
OFFNOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
A4NOTE,
OFFNOTE,
A4FLATNOTE,
A4FLATNOTE,
OFFNOTE,
OFFNOTE,
A4NOTE,
OFFNOTE,
BFLAT4NOTE,
OFFNOTE,
A4NOTE,
OFFNOTE,
G4NOTE,
G4NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
EFLAT5NOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
G5NOTE,
OFFNOTE,
G5NOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
G5NOTE,
OFFNOTE,
G5NOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
EFLAT5NOTE,
OFFNOTE,
EFLAT5NOTE,
EFLAT5NOTE,
OFFNOTE,
OFFNOTE,
D5NOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
F5NOTE,
OFFNOTE,
OFFNOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
G5NOTE,
G5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
OFFNOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
OFFNOTE,
F5NOTE,
OFFNOTE,
D5NOTE,
D5NOTE,
OFFNOTE,
OFFNOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
OFFNOTE,
BFLAT4NOTE,
BFLAT4NOTE,
BFLAT4NOTE,
BFLAT4NOTE,
BFLAT4NOTE,
BFLAT4NOTE,
};
