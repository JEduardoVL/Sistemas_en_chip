// Configuraci?n  para PIC16F873A
#pragma config FOSC = XT      // Oscilador externo
#pragma config WDTE = OFF     // Watchdog Timer deshabilitado
#pragma config PWRTE = OFF    // Power-up Timer deshabilitado
#pragma config CP = OFF       // Code Protection deshabilitado
#pragma config BOREN = ON     // Brown-out Reset habilitado
#pragma config LVP = OFF      // Low Voltage Programming deshabilitado
#pragma config CPD = OFF      // Data Code Protection deshabilitado
#pragma config WRT = OFF      // Flash Program Memory Write Enable
#pragma config DEBUG = OFF    // Debug deshabilitado

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include <string.h>
#include <stdbool.h>


#define _XTAL_FREQ 4000000 // Frecuencia del oscilador en Hz

// Definiciones de pines para el LCD
#define RS RA0
#define RW RA1
#define E RA2
#define DATA_PORT PORTB
#define BUZZER RA3

// Definiciones para la comunicacion
#define SCL RC3
#define SDA RC4

// Definiciones de comandos del LCD
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define DISPLAY_ON_CURSOR_BLINK 0x0F
#define FUNCTION_SET_8BIT 0x38
#define SET_CURSOR_ROW1 0x80
#define SET_CURSOR_ROW2 0xC0

#define MAX_CHARS 16 // M?ximo n?mero de caracteres a mostrar
#define MAX_RECEIVE_LENGTH 15

#define MASTER_I2C_ADDRESS 0x20
#define SLAVE_I2C_ADDRESS 0x10

// A?ade esta definici?n para alternar roles
#define SLAVE_MODE 1
#define MASTER_MODE 0

volatile unsigned char i2c_role = MASTER_MODE;

void I2C_Slave_Init(unsigned char address);

// Declaraci?n de variables globales para el teclado
const unsigned char teclas[4][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', 'X'},
    {'1', '2', '3', '-'},
    {'C', '0', '=', '+'}
};

unsigned char cursorPosition = 0;
char displayString[MAX_CHARS + 1] = "";  // Aumenta el tama?o para manejar el '\0'
unsigned char charCount = 0;
unsigned char asciiMode = 0;
char asciiValue[4] = "";
unsigned char asciiDigits = 0;

// Declaraciones de funciones para el LCD
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_clear(void);
void LCD_setCursor(unsigned char row, unsigned char column);
void LCD_print(const char *string);

// Declaraciones de funciones para el teclado matricial
void Keyboard_init(void);
char Read_Keypad(void);

// Funciones auxiliares para manejar los pines compartidos
void Prepare_LCD_Pins(void);
void Prepare_Keypad_Pins(void);

// Declaraciones para I2C
void I2C_Master_Init(const unsigned long c);
void I2C_Master_Wait(void);
void I2C_Master_Start(void);
void I2C_Master_Stop(void);
void I2C_Master_Write(unsigned d);
unsigned char I2C_Master_Read(unsigned char ack);
void I2C_Read_Message(void);

void main() {
    LCD_init();
    Keyboard_init();
    I2C_Master_Init(100000); // Inicializa I2C a 100kHz
    
    
    while(1) {
        Prepare_Keypad_Pins();
        char key = Read_Keypad();  // Lee una tecla presionada
       
        if (key == '-') {
            // Env?o del mensaje si se presiona '-'
            if (charCount > 0) {
                I2C_Master_Start();
                I2C_Master_Write(0x10);  // Direcci?n del esclavo con el bit de escritura
                for (int i = 0; i < charCount; i++) {
                    I2C_Master_Write(displayString[i]);
                }
                I2C_Master_Write('\0');  // Env?a el car?cter nulo explicitamente
                I2C_Master_Stop();
Prepare_LCD_Pins();
                LCD_clear();
                LCD_setCursor(1, 0);
                Prepare_LCD_Pins();
                LCD_print("Mensaje enviado");
                LCD_setCursor(2,0);
                LCD_print("al esclavo . . .");
                __delay_ms(2000);
                LCD_clear();
                
                // Limpieza del buffer y reseteo de variables
                memset(displayString, 0, sizeof(displayString));  // Limpia el buffer
                charCount = 0;
                cursorPosition = 0;
                
                // Cambia a modo esclavo
                i2c_role = SLAVE_MODE;
                I2C_Slave_Init(MASTER_I2C_ADDRESS);
                LCD_setCursor(1,0);
                LCD_print("Esperando res-");
                LCD_setCursor(2,0);
                LCD_print("del esclavo ...");
                
               
            }
        }
         else if (key != 0 && key != '-') {
            // Manejo de teclas normales y modo ASCII
            if (asciiMode && key == '=') {
                // Convierte asciiValue a un n?mero y lo imprime como car?cter
                int asciiCode = atoi(asciiValue);
                if (asciiCode >= 33 && asciiCode <= 254 && charCount < MAX_CHARS) {
                    for (int i = charCount; i > cursorPosition; i--) {
                        displayString[i] = displayString[i - 1];
                    }
                    displayString[cursorPosition] = (char)asciiCode;
                    charCount++;
                    if (cursorPosition < MAX_CHARS - 1) {
                        cursorPosition++;
                    }
                    displayString[charCount] = '\0';  // Asegura el terminador nulo
                }
                asciiMode = 0;
                asciiDigits = 0;
                asciiValue[0] = '\0';
            } else if (asciiMode && key >= '0' && key <= '9' && asciiDigits < 3) {
                asciiValue[asciiDigits++] = key;
                asciiValue[asciiDigits] = '\0';
            } else if (!asciiMode) {
                switch (key) {
                    case '+':
                        asciiMode = 1; // Activa el modo ASCII
                asciiDigits = 0;
                asciiValue[0] = '\0'; // Limpia el buffer para nuevos d?gitos
                        break;
                    case '/':
                        if (cursorPosition > 0) cursorPosition--;
                        break;
                    case 'X':
                        if (cursorPosition < charCount) cursorPosition++;
                        break;
                    case 'C':
                        if (cursorPosition < charCount) {
                            for (int i = cursorPosition; i < charCount; i++) {
                                displayString[i] = displayString[i + 1];
                            }
                            charCount--;
                        }
                        break;
                    default:
                       if (charCount < MAX_CHARS - 1) {
                    for (int i = charCount; i > cursorPosition; i--) {
                        displayString[i] = displayString[i - 1];
                    }
                    displayString[cursorPosition] = key;
                    if (cursorPosition < MAX_CHARS - 1) cursorPosition++;
                    if (charCount < MAX_CHARS) charCount++;
                }

                // Actualizaci?n del LCD despu?s de la inserci?n
                Prepare_LCD_Pins();
                LCD_clear();
                LCD_setCursor(1, 0);
                LCD_print(displayString);
                LCD_setCursor(1, cursorPosition);  // Asegura que el cursor est? en la posici?n correcta despu?s de la inserci?n
                break;
                }
            }

            LCD_clear();
            LCD_setCursor(1, 0);
            LCD_print(displayString);
            LCD_setCursor(1, cursorPosition);  // Actualiza la posici?n del cursor en el LCD
        }

        __delay_ms(100); // Debounce delay para evitar lecturas m?ltiples
    }
}

// Funciones para el LCD a 8 bits

void LCD_command(unsigned char command) {
    RS = 0;
    RW = 0;
    DATA_PORT = command;
    E = 1;
    __delay_ms(2); // Aumenta este tiempo si es necesario
    E = 0;
}

void LCD_data(unsigned char data) {
    RS = 1;
    RW = 0;
    DATA_PORT = data;
    E = 1;
    __delay_ms(2); // Aumenta este tiempo si es necesario
    E = 0;
}

void LCD_init() {
    // Configura todos los pines de PORTA como digitales si es necesario
    ADCON1 = 0x07;  // Desactiva las entradas anal?gicas
    TRISA = 0x00;   // Configura los pines de control como salidas
    TRISB = 0x00;   // Configura los pines de datos como salidas

    // Espera inicial antes de configurar el LCD
    __delay_ms(20); // Espera para que el LCD se estabilice despu?s de encender

    // Inicializaci?n del LCD en modo de 8 bits
    LCD_command(FUNCTION_SET_8BIT);  // Funci?n Set: modo de 8 bits
    __delay_ms(5);                   // Espera para que el comando se procese

    LCD_command(DISPLAY_ON_CURSOR_BLINK); // Display ON, cursor y parpadeo ON
    __delay_ms(5);                   // Espera para que el comando se procese

    LCD_command(CLEAR_DISPLAY);     // Limpia el display
    __delay_ms(2);                  // Espera para que el display se limpie

    LCD_command(RETURN_HOME);       // Retorna el cursor a casa
    __delay_ms(2);                  // Espera para que el cursor se posicione en casa
}

void LCD_clear() {
    LCD_command(CLEAR_DISPLAY);
    __delay_ms(2);
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


// Implementaci?n de funciones para el teclado matricial
void Keyboard_init() {
    TRISAbits.TRISA3 = 0;
    TRISA = 0x00; // Asegura que los pines de control del LCD sean salidas
    TRISB = 0x0F; // RB4-RB7 como salidas (filas) y RB0-RB3 como entradas (columnas)
    PORTB = 0xF0; // Asegura que las filas est?n en alto
}

char Read_Keypad() {
    for(int row = 0; row < 4; row++) {
        // Pone una fila a nivel bajo a la vez
        PORTB = (unsigned char)~(0x10 << row);
        
        // Configura RB0 a RB3 como entradas para leer las columnas
        TRISB = 0x0F;
        
        __delay_ms(20); // Debounce time
        
        for(int col = 0; col < 4; col++) {
            // Si detectamos una entrada en bajo en una de las columnas, se presion? una tecla
            if(!(PORTB & (0x01 << col))) {
                while(!(PORTB & (0x01 << col))); // Espera a que se suelte la tecla
                PORTAbits.RA3 = 0;
                __delay_ms(100); // Duraci?n del sonido del buzzer
                PORTAbits.RA3 = 1; // Apaga el buzzer
                // Configura RB4 a RB7 como salidas antes de continuar
                TRISB = 0xF0; // Esto es necesario para preparar el teclado para la pr?xima lectura
                
                return teclas[row][col]; // Retorna el car?cter correspondiente
            }
        }
        
        // Configura RB4 a RB7 como salidas antes de continuar
        TRISB = 0xF0; // Esto es necesario para preparar el teclado para la pr?xima lectura
    }
    
    return 0; // No se presion? ninguna tecla
}

// Funciones axiliares 

void Prepare_LCD_Pins() {
    // Configura los pines para el uso del LCD antes de usarlo para enviar comandos o datos
    TRISB = 0x00; // Configura PORTB como salida
}

void Prepare_Keypad_Pins() {
    // Configura los pines para el uso del teclado antes de leerlo
    TRISB = 0xF0; // RB0-RB3 como salida (para filas), RB4-RB7 como entrada (para columnas)
    PORTB = 0x0F; // Baja las filas para activar el teclado, activa resistencias de pull-up para columnas
}

// Comunicacion I2C
void I2C_Master_Init(const unsigned long c) {
    SSPCON = 0b00101000;  // SSPEN=1, I2C Master mode, clock = FOSC/(4 * (SSPADD+1))
   SSPADD = (unsigned char)((_XTAL_FREQ / (4 * c)) - 1);  // Aseg?rate de que el resultado sea v?lido para unsigned char

    SSPSTAT = 0x00;
    TRISC3 = 1;  // Configura pines SCL y SDA como entradas
    TRISC4 = 1;
}

void I2C_Master_Wait() {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));  // Espera que el bus est? libre
}

void I2C_Master_Start() {
    I2C_Master_Wait();
    SEN = 1;  // Inicia la condici?n de START
}

void I2C_Master_Stop() {
    I2C_Master_Wait();
    PEN = 1;  // Inicia la condici?n de STOP
}

void I2C_Master_Write(unsigned d) {
    I2C_Master_Wait();
    SSPBUF = (unsigned char)d;  // Escribe el dato en el buffer
}

// Inicializaci?n de I2C como esclavo
void I2C_Slave_Init(unsigned char address) {
    SSPADD = address;  // Establece la direcci?n del esclavo I2C
    SSPCON = 0x36;     // Configura el registro para modo esclavo
    SSPIE = 1;         // Habilita la interrupci?n de SSP (I2C)
    PEIE = 1;          // Habilita interrupciones perif?ricas
    GIE = 1;           // Habilita interrupciones globales
    SSPSTAT = 0x00;
    TRISC3 = 1;        // SCL como entrada
    TRISC4 = 1;        // SDA como entrada
}

void __interrupt() ISR(void) {
    static char receiveBuffer[MAX_RECEIVE_LENGTH + 1];
    static int index = 0;

    if (SSPIF) {
        char temp = SSPBUF;  // Lectura del buffer para limpiar BF
        SSPCONbits.CKP = 1;  // Libera el reloj SCL

        if (!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            index = 0;  // Prepararse para recibir datos
        } else if (SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            if (index < MAX_RECEIVE_LENGTH) {
                receiveBuffer[index++] = temp;  // Almacenar datos en el buffer
                if (temp == '\0' || index == MAX_RECEIVE_LENGTH) {  // Si se recibe el car?cter nulo o se alcanza el tama?o m?ximo
                    receiveBuffer[index] = '\0';  // Termina la cadena
                    Prepare_LCD_Pins();
                    LCD_clear();
                    LCD_setCursor(1, 0);
                    LCD_print(receiveBuffer);
                    __delay_ms(2000);
                    LCD_clear();
                    LCD_setCursor(1,0);
                    LCD_print("Escriba res-");
                    LCD_setCursor(2,0);
                    LCD_print("para esclavo...");
                    index = 0;
                    __delay_ms(2000);
                    // Cambia a modo maestro
                    //LCD_init();
                    LCD_clear();
                    LCD_setCursor(1, 0);
                    
                    Keyboard_init();
                    
                i2c_role = MASTER_MODE;
                I2C_Master_Init(100000);
                }
            }
        } 
        SSPIF = 0;  // Limpia la bandera de interrupci?n
    }
}