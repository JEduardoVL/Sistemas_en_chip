/*
Programa elaborado por:
    Valerio López José Eduardo
*/

// Declaración de librerias
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>

// Configuraciones para el PIC16F84A
#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config CP = OFF

#define _XTAL_FREQ 4000000 // Frecuencia del oscilador en Hz

// Definiciones de pines para el LCD
#define RS RA0
#define RW RA1
#define E RA2
#define DATA_PORT PORTB

#define BUZZER RA3

// Definiciones de comandos del LCD
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define DISPLAY_ON_CURSOR_BLINK 0x0F
#define FUNCTION_SET_8BIT 0x38
#define SET_CURSOR_ROW1 0x80
#define SET_CURSOR_ROW2 0xC0

#define MAX_CHARS 15 // M?ximo n?mero de caracteres a mostrar

// Declaraci?n de variables globales para el teclado
const unsigned char teclas[4][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', 'X'},
    {'1', '2', '3', '-'},
    {'C', '0', '=', '+'}
};

unsigned char cursorPosition = 0; // Agrega un cursorPosition para manejar la posici?n actual del cursor

// Modo ASCII
unsigned char asciiMode = 0; // 0: Modo normal, 1: Modo ASCII
char asciiValue[4] = ""; // Almacenar? los d?gitos del valor ASCII
unsigned char asciiDigits = 0; // N?mero de d?gitos ingresados en modo ASCII

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

// Función principal
void main() {
    LCD_init();
    Keyboard_init();
    char displayString[MAX_CHARS + 2] = "";
    unsigned char charCount = 0;

    while(1) {
        Prepare_Keypad_Pins();
        char key = Read_Keypad();

        if (asciiMode) {
            if (key == '=') {
                // Convierte asciiValue a un n?mero y lo imprime como car?cter
                int asciiCode = atoi(asciiValue);
                if (asciiCode >= 33 && asciiCode <= 254) { // Asegura que est? en el rango permitido
                    if (charCount < MAX_CHARS) {
                        for (unsigned char i = charCount; i > cursorPosition; i--) {
                            displayString[i] = displayString[i - 1];
                        }
                        displayString[cursorPosition] = (char)asciiCode;
                        if (cursorPosition < MAX_CHARS) cursorPosition++;
                        if (charCount < MAX_CHARS) charCount++;
                    }
                }
                asciiMode = 0; // Salir del modo ASCII
                asciiDigits = 0; // Resetea el contador de d?gitos
                asciiValue[0] = '\0'; // Limpia el buffer
            } else if (key >= '0' && key <= '9' && asciiDigits < 3) {
                // Acumula d?gitos para el c?digo ASCII
                asciiValue[asciiDigits++] = key;
                asciiValue[asciiDigits] = '\0'; // Asegura que la cadena est? correctamente terminada
            }
        } else {
            // El resto del manejo de teclas se mantiene igual, excepto por el chequeo adicional para '+' para activar el modo ASCII
            if (key == '+') {
                asciiMode = 1; // Activa el modo ASCII
                asciiDigits = 0;
                asciiValue[0] = '\0'; // Limpia el buffer para nuevos d?gitos
            }
            // Manejo de desplazamiento y edici?n en modo normal
            else if (key == '/') { // Mover cursor a la izquierda
                if (cursorPosition > 0) cursorPosition--;
            } else if (key == 'X') { // Mover cursor a la derecha
                if (cursorPosition < charCount) cursorPosition++;
            } else if (key == 'C') { // Borrar car?cter en el cursor
            for (unsigned char i = cursorPosition; i < charCount; i++) {
                displayString[i] = displayString[i + 1];
            }
            if (charCount > 0) charCount--; // Decrementa el contador de caracteres
            // Ajusta la posici?n del cursor si es necesario
            if (cursorPosition >= charCount && charCount > 0) cursorPosition = charCount - 1;
        } else if (key != 0 && key != '=') {
                // Inserta o sobrescribe un car?cter en la posici?n del cursor en modo normal
                if (charCount < MAX_CHARS) {
                    for (unsigned char i = charCount; i > cursorPosition; i--) {
                        displayString[i] = displayString[i - 1];
                    }
                    displayString[cursorPosition] = key;
                    if (cursorPosition < MAX_CHARS) cursorPosition++; // Mueve el cursor a la derecha
                    if (charCount < MAX_CHARS) charCount++; // Incrementa el contador si no se ha alcanzado el m?ximo
                }
            }
        }
        
        Prepare_LCD_Pins();
        LCD_clear();
        LCD_setCursor(1, 0);
        LCD_print(displayString);
        LCD_setCursor(1, cursorPosition); // Actualiza la posici?n del cursor en el LCD
        
        __delay_ms(100);
    }
}

// Implementaci?n de funciones para el LCD

void LCD_command(unsigned char command) {
    RS = 0;
    RW = 0;
    DATA_PORT = command;
    E = 1;
    __delay_ms(2); 
    E = 0;
}

void LCD_data(unsigned char data) {
    RS = 1;
    RW = 0;
    DATA_PORT = data;
    E = 1;
    __delay_ms(2); 
    E = 0;
}

void LCD_init() {
    TRISA = 0x00; // Configura los pines de control como salidas
    TRISB = 0x00; // Configura los pines de datos como salidas
    __delay_ms(20); // Espera para que el LCD est? listo
    LCD_command(FUNCTION_SET_8BIT);
    __delay_ms(5);
    LCD_command(DISPLAY_ON_CURSOR_BLINK);
    LCD_command(CLEAR_DISPLAY);
    __delay_ms(2);
}

// Función para limpiar el LCD
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



void Prepare_LCD_Pins() {
    // Configura los pines para el uso del LCD antes de usarlo para enviar comandos o datos
    TRISB = 0x00; // Configura PORTB como salida
}

void Prepare_Keypad_Pins() {
    // Configura los pines para el uso del teclado antes de leerlo
    TRISB = 0xF0; // RB0-RB3 como salida (para filas), RB4-RB7 como entrada (para columnas)
    PORTB = 0x0F; // Baja las filas para activar el teclado, activa resistencias de pull-up para columnas
}
