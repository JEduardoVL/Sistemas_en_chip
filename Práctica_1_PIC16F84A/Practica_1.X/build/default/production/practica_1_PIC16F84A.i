# 1 "practica_1_PIC16F84A.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "practica_1_PIC16F84A.c" 2






# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\musl_xc8.h" 1 3
# 5 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 2 3





# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\features.h" 1 3
# 11 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 2 3
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 12 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 128 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 143 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef short ssize_t;
# 174 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 210 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 253 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long off_t;
# 409 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 2 3
# 52 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);





int ungetc(int, FILE *);
int getch(void);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);





void putch(char);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

__attribute__((__format__(__printf__, 1, 2)))
int printf(const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int fprintf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int sprintf(char *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 3, 4)))
int snprintf(char *restrict, size_t, const char *restrict, ...);

__attribute__((__format__(__printf__, 1, 0)))
int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 2, 0)))
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 3, 0)))
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

__attribute__((__format__(__scanf__, 1, 2)))
int scanf(const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int fscanf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int sscanf(const char *restrict, const char *restrict, ...);

__attribute__((__format__(__scanf__, 1, 0)))
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__scanf__, 2, 0)))
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 7 "practica_1_PIC16F84A.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdlib.h" 1 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdlib.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdlib.h" 2 3

int atoi (const char *);
long atol (const char *);



double atof (const char *);


float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);





unsigned long __strtoxl(const char * s, char ** endptr, int base, char is_signed);
# 55 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdlib.h" 3
int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);




typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;




div_t div (int, int);
ldiv_t ldiv (long, long);




typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 8 "practica_1_PIC16F84A.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdint.h" 1 3
# 26 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 133 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned short uintptr_t;
# 148 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef short intptr_t;
# 164 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 179 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;
# 192 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef int32_t intmax_t;







typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 215 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;
# 233 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/alltypes.h" 3
typedef uint32_t uintmax_t;
# 27 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdint.h" 2 3

typedef int8_t int_fast8_t;




typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;




typedef uint8_t uint_fast8_t;




typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;
# 148 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 149 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\stdint.h" 2 3
# 9 "practica_1_PIC16F84A.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\xc8debug.h" 1 3
# 12 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\c99\\xc8debug.h" 3
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 23 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 2 3

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\builtins.h" 1 3






#pragma intrinsic(__nop)
extern void __nop(void);
# 19 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\builtins.h" 3
#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(uint32_t);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(uint32_t);
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 2 3




# 1 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 1 3




# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\htc.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\htc.h" 2 3
# 6 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 2 3







# 1 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic_chip_select.h" 1 3
# 543 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic_chip_select.h" 3
# 1 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 1 3
# 44 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\__at.h" 1 3
# 45 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 2 3







extern volatile unsigned char INDF __attribute__((address(0x000)));

__asm("INDF equ 00h");




extern volatile unsigned char TMR0 __attribute__((address(0x001)));

__asm("TMR0 equ 01h");




extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");




extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
        unsigned RP :2;
        unsigned IRP :1;
    };
    struct {
        unsigned :5;
        unsigned RP0 :1;
        unsigned RP1 :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 159 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char FSR __attribute__((address(0x004)));

__asm("FSR equ 04h");




extern volatile unsigned char PORTA __attribute__((address(0x005)));

__asm("PORTA equ 05h");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x005)));
# 210 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char PORTB __attribute__((address(0x006)));

__asm("PORTB equ 06h");


typedef union {
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0x006)));
# 272 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char EEDATA __attribute__((address(0x008)));

__asm("EEDATA equ 08h");




extern volatile unsigned char EEADR __attribute__((address(0x009)));

__asm("EEADR equ 09h");




extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");


typedef union {
    struct {
        unsigned PCLATH :5;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0x00A)));
# 306 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned RBIF :1;
        unsigned INTF :1;
        unsigned T0IF :1;
        unsigned RBIE :1;
        unsigned INTE :1;
        unsigned T0IE :1;
        unsigned EEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :2;
        unsigned TMR0IF :1;
        unsigned :2;
        unsigned TMR0IE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 384 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char OPTION_REG __attribute__((address(0x081)));

__asm("OPTION_REG equ 081h");


typedef union {
    struct {
        unsigned PS :3;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned INTEDG :1;
        unsigned nRBPU :1;
    };
    struct {
        unsigned PS0 :1;
        unsigned PS1 :1;
        unsigned PS2 :1;
    };
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __attribute__((address(0x081)));
# 454 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x085)));

__asm("TRISA equ 085h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x085)));
# 498 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char TRISB __attribute__((address(0x086)));

__asm("TRISB equ 086h");


typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0x086)));
# 560 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0x088)));

__asm("EECON1 equ 088h");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned EEIF :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0x088)));
# 604 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0x089)));

__asm("EECON2 equ 089h");
# 621 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\proc\\pic16f84a.h" 3
extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit EEIE __attribute__((address(0x5E)));


extern volatile __bit EEIF __attribute__((address(0x444)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit INTE __attribute__((address(0x5C)));


extern volatile __bit INTEDG __attribute__((address(0x40E)));


extern volatile __bit INTF __attribute__((address(0x59)));


extern volatile __bit IRP __attribute__((address(0x1F)));


extern volatile __bit PS0 __attribute__((address(0x408)));


extern volatile __bit PS1 __attribute__((address(0x409)));


extern volatile __bit PS2 __attribute__((address(0x40A)));


extern volatile __bit PSA __attribute__((address(0x40B)));


extern volatile __bit RA0 __attribute__((address(0x28)));


extern volatile __bit RA1 __attribute__((address(0x29)));


extern volatile __bit RA2 __attribute__((address(0x2A)));


extern volatile __bit RA3 __attribute__((address(0x2B)));


extern volatile __bit RA4 __attribute__((address(0x2C)));


extern volatile __bit RB0 __attribute__((address(0x30)));


extern volatile __bit RB1 __attribute__((address(0x31)));


extern volatile __bit RB2 __attribute__((address(0x32)));


extern volatile __bit RB3 __attribute__((address(0x33)));


extern volatile __bit RB4 __attribute__((address(0x34)));


extern volatile __bit RB5 __attribute__((address(0x35)));


extern volatile __bit RB6 __attribute__((address(0x36)));


extern volatile __bit RB7 __attribute__((address(0x37)));


extern volatile __bit RBIE __attribute__((address(0x5B)));


extern volatile __bit RBIF __attribute__((address(0x58)));


extern volatile __bit RD __attribute__((address(0x440)));


extern volatile __bit RP0 __attribute__((address(0x1D)));


extern volatile __bit RP1 __attribute__((address(0x1E)));


extern volatile __bit T0CS __attribute__((address(0x40D)));


extern volatile __bit T0IE __attribute__((address(0x5D)));


extern volatile __bit T0IF __attribute__((address(0x5A)));


extern volatile __bit T0SE __attribute__((address(0x40C)));


extern volatile __bit TMR0IE __attribute__((address(0x5D)));


extern volatile __bit TMR0IF __attribute__((address(0x5A)));


extern volatile __bit TRISA0 __attribute__((address(0x428)));


extern volatile __bit TRISA1 __attribute__((address(0x429)));


extern volatile __bit TRISA2 __attribute__((address(0x42A)));


extern volatile __bit TRISA3 __attribute__((address(0x42B)));


extern volatile __bit TRISA4 __attribute__((address(0x42C)));


extern volatile __bit TRISB0 __attribute__((address(0x430)));


extern volatile __bit TRISB1 __attribute__((address(0x431)));


extern volatile __bit TRISB2 __attribute__((address(0x432)));


extern volatile __bit TRISB3 __attribute__((address(0x433)));


extern volatile __bit TRISB4 __attribute__((address(0x434)));


extern volatile __bit TRISB5 __attribute__((address(0x435)));


extern volatile __bit TRISB6 __attribute__((address(0x436)));


extern volatile __bit TRISB7 __attribute__((address(0x437)));


extern volatile __bit WR __attribute__((address(0x441)));


extern volatile __bit WREN __attribute__((address(0x442)));


extern volatile __bit WRERR __attribute__((address(0x443)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nRBPU __attribute__((address(0x40F)));


extern volatile __bit nTO __attribute__((address(0x1C)));
# 544 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic_chip_select.h" 2 3
# 14 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 2 3
# 76 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\eeprom_routines.h" 1 3
# 114 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 84 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 2 3
# 118 "C:/Program Files/Microchip/MPLABX/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8\\pic\\include\\pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 28 "C:\\Program Files\\Microchip\\xc8\\v2.46\\pic\\include\\xc.h" 2 3
# 10 "practica_1_PIC16F84A.c" 2



#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config CP = OFF
# 39 "practica_1_PIC16F84A.c"
const unsigned char teclas[4][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', 'X'},
    {'1', '2', '3', '-'},
    {'C', '0', '=', '+'}
};

unsigned char cursorPosition = 0;


unsigned char asciiMode = 0;
char asciiValue[4] = "";
unsigned char asciiDigits = 0;


void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_clear(void);
void LCD_setCursor(unsigned char row, unsigned char column);
void LCD_print(const char *string);


void Keyboard_init(void);
char Read_Keypad(void);


void Prepare_LCD_Pins(void);
void Prepare_Keypad_Pins(void);


void main() {
    LCD_init();
    Keyboard_init();
    char displayString[15 + 2] = "";
    unsigned char charCount = 0;

    while(1) {
        Prepare_Keypad_Pins();
        char key = Read_Keypad();

        if (asciiMode) {
            if (key == '=') {

                int asciiCode = atoi(asciiValue);
                if (asciiCode >= 33 && asciiCode <= 254) {
                    if (charCount < 15) {
                        for (unsigned char i = charCount; i > cursorPosition; i--) {
                            displayString[i] = displayString[i - 1];
                        }
                        displayString[cursorPosition] = (char)asciiCode;
                        if (cursorPosition < 15) cursorPosition++;
                        if (charCount < 15) charCount++;
                    }
                }
                asciiMode = 0;
                asciiDigits = 0;
                asciiValue[0] = '\0';
            } else if (key >= '0' && key <= '9' && asciiDigits < 3) {

                asciiValue[asciiDigits++] = key;
                asciiValue[asciiDigits] = '\0';
            }
        } else {

            if (key == '+') {
                asciiMode = 1;
                asciiDigits = 0;
                asciiValue[0] = '\0';
            }

            else if (key == '/') {
                if (cursorPosition > 0) cursorPosition--;
            } else if (key == 'X') {
                if (cursorPosition < charCount) cursorPosition++;
            } else if (key == 'C') {
            for (unsigned char i = cursorPosition; i < charCount; i++) {
                displayString[i] = displayString[i + 1];
            }
            if (charCount > 0) charCount--;

            if (cursorPosition >= charCount && charCount > 0) cursorPosition = charCount - 1;
        } else if (key != 0 && key != '=') {

                if (charCount < 15) {
                    for (unsigned char i = charCount; i > cursorPosition; i--) {
                        displayString[i] = displayString[i - 1];
                    }
                    displayString[cursorPosition] = key;
                    if (cursorPosition < 15) cursorPosition++;
                    if (charCount < 15) charCount++;
                }
            }
        }

        Prepare_LCD_Pins();
        LCD_clear();
        LCD_setCursor(1, 0);
        LCD_print(displayString);
        LCD_setCursor(1, cursorPosition);

        _delay((unsigned long)((100)*(4000000/4000.0)));
    }
}



void LCD_command(unsigned char command) {
    RA0 = 0;
    RA1 = 0;
    PORTB = command;
    RA2 = 1;
    _delay((unsigned long)((2)*(4000000/4000.0)));
    RA2 = 0;
}

void LCD_data(unsigned char data) {
    RA0 = 1;
    RA1 = 0;
    PORTB = data;
    RA2 = 1;
    _delay((unsigned long)((2)*(4000000/4000.0)));
    RA2 = 0;
}

void LCD_init() {
    TRISA = 0x00;
    TRISB = 0x00;
    _delay((unsigned long)((20)*(4000000/4000.0)));
    LCD_command(0x38);
    _delay((unsigned long)((5)*(4000000/4000.0)));
    LCD_command(0x0F);
    LCD_command(0x01);
    _delay((unsigned long)((2)*(4000000/4000.0)));
}


void LCD_clear() {
    LCD_command(0x01);
    _delay((unsigned long)((2)*(4000000/4000.0)));
}

void LCD_setCursor(unsigned char row, unsigned char column) {
    unsigned char position = (row == 1) ? 0x80 : 0xC0;
    position += column;
    LCD_command(position);
}

void LCD_print(const char *string) {
    while(*string) {
        LCD_data(*string++);
    }
}



void Keyboard_init() {
    TRISAbits.TRISA3 = 0;
    TRISA = 0x00;
    TRISB = 0x0F;
    PORTB = 0xF0;
}


char Read_Keypad() {
    for(int row = 0; row < 4; row++) {

        PORTB = (unsigned char)~(0x10 << row);


        TRISB = 0x0F;

        _delay((unsigned long)((20)*(4000000/4000.0)));

        for(int col = 0; col < 4; col++) {

            if(!(PORTB & (0x01 << col))) {
                while(!(PORTB & (0x01 << col)));
                PORTAbits.RA3 = 0;
                _delay((unsigned long)((100)*(4000000/4000.0)));
                PORTAbits.RA3 = 1;

                TRISB = 0xF0;

                return teclas[row][col];
            }
        }


        TRISB = 0xF0;
    }

    return 0;
}



void Prepare_LCD_Pins() {

    TRISB = 0x00;
}

void Prepare_Keypad_Pins() {

    TRISB = 0xF0;
    PORTB = 0x0F;
}
