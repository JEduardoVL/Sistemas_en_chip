#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// Configuraciones del PIC16F873A
#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config DEBUG = OFF

#define _XTAL_FREQ 4000000  // Frecuencia del oscilador en Hz

// Definiciones de pines para el LCD y I2C
#define RS RA0
#define RW RA1
#define E RA2
#define DATA_PORT PORTB
#define SCL RC3
#define SDA RC4

// Definiciones de comandos del LCD
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define DISPLAY_ON_CURSOR_BLINK 0x0F
#define FUNCTION_SET_8BIT 0x38
#define SET_CURSOR_ROW1 0x80
#define SET_CURSOR_ROW2 0xC0
// Direcci?n I2C del esclavo
#define SLAVE_I2C_ADDRESS 0x10
#define MAX_RECEIVE_LENGTH 15

#define MASTER_I2C_ADDRESS 0x20 
#define SLAVE_I2C_ADDRESS 0x10

// A?ade esta definici?n para alternar roles
#define SLAVE_MODE 1
#define MASTER_MODE 0

#define MAX_CHARS 16  // Define el m?ximo de caracteres que el LCD puede mostrar por l?nea

// Agregar esta definici?n al inicio
volatile unsigned char i2c_role = SLAVE_MODE;

void I2C_Master_Init(const unsigned long clock);

//int cursorPosition = 0;
unsigned char asciiMode = 0;
char asciiValue[4] = "";
unsigned char asciiDigits = 0;
unsigned char cursorPosition = 0;

// Declaraci?n de variables globales para el teclado
const unsigned char teclas[4][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', 'X'},
    {'1', '2', '3', '-'},
    {'C', '0', '=', '+'}
};

// Prototipos de funciones
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_clear(void);
void LCD_setCursor(unsigned char row, unsigned char column);
void LCD_print(const char *string);
void I2C_Slave_Init(unsigned char address);
void __interrupt() ISR(void);
char Read_Keypad(void);
void Handle_Keypress(char key);
void Send_Message_To_Master(char *message);
void Prepare_Keypad_Pins(void);
void Prepare_LCD_Pins(void);

// Funciones de I2C maestro necesarias
void I2C_Master_Init(const unsigned long clock);
void I2C_Master_Start(void);
void I2C_Master_Stop(void);
void I2C_Master_Write(unsigned char data);
void I2C_Master_Wait(void);


char displayString[MAX_RECEIVE_LENGTH + 1] = "";  // Buffer para almacenar el mensaje a enviar
int charCount = 0;  // Contador de caracteres en el mensaje actual
volatile bool allow_input = false;


void main() {
    LCD_init();
    I2C_Slave_Init(SLAVE_I2C_ADDRESS);

    LCD_clear();
    LCD_setCursor(1, 0);
    LCD_print("Espera de men-");
    LCD_setCursor(2,0);
    LCD_print("del maestro...");

    while (1) {
        if (allow_input) {
            Prepare_Keypad_Pins();
            char key = Read_Keypad();
            if (key !=0) {
                Handle_Keypress(key);
                
    }
            }
        }
    }

void Handle_Keypress(char key) {
    LCD_init();

    // Manejo del modo ASCII
    if (asciiMode) {
        if (key == '=') {
            // Convierte asciiValue a un número y lo imprime como carácter
            int asciiCode = atoi(asciiValue);
            if (asciiCode >= 33 && asciiCode <= 254) { // Asegura que está en el rango permitido
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
            asciiDigits = 0; // Resetea el contador de dígitos
            asciiValue[0] = '\0'; // Limpia el buffer
        } else if (key >= '0' && key <= '9' && asciiDigits < 3) {
            // Acumula dígitos para el código ASCII
            asciiValue[asciiDigits++] = key;
            asciiValue[asciiDigits] = '\0'; // Asegura que la cadena esté correctamente terminada
        }
    } else {
        // Activar el modo ASCII
        if (key == '+') {
            asciiMode = 1; // Activa el modo ASCII
            asciiDigits = 0;
            asciiValue[0] = '\0'; // Limpia el buffer para nuevos dígitos
        }
        // Manejo de desplazamiento y edición en modo normal
        else if (key == '/') { // Mover cursor a la izquierda
            if (cursorPosition > 0) cursorPosition--;
        } else if (key == 'X') { // Mover cursor a la derecha
            if (cursorPosition < charCount) cursorPosition++;
        } else if (key == 'C') { // Borrar carácter en el cursor
            for (unsigned char i = cursorPosition; i < charCount; i++) {
                displayString[i] = displayString[i + 1];
            }
            if (charCount > 0) charCount--; // Decrementa el contador de caracteres
            // Ajusta la posición del cursor si es necesario
            if (cursorPosition >= charCount && charCount > 0) cursorPosition = charCount - 1;
        } else if (key == '-') {
            if (charCount > 0) {
                displayString[cursorPosition] = '\0'; // Asegura el terminador nulo antes de enviar
                Send_Message_To_Master(displayString);
                LCD_clear();
                LCD_setCursor(1, 0);
                LCD_print("Mensaje enviado");
                LCD_setCursor(2,0);
                LCD_print("al maestro . . .");
                
                __delay_ms(2000); // Muestra el mensaje por dos segundos
                LCD_clear();
                LCD_setCursor(1, 0);
                LCD_print("Esperando res-");
                LCD_setCursor(2,0);
                LCD_print("del maestro . . .");
                allow_input = false;  // Deshabilita la entrada mientras procesa
                memset(displayString, 0, sizeof(displayString));  // Limpia el buffer
                charCount = 0;
                cursorPosition = 0;
            }
        } else if (key != 0 && key != '=' && charCount < MAX_CHARS - 1) {
            // Agrega o modifica el carácter al buffer
            for (int i = charCount; i > cursorPosition; i--) {
                displayString[i] = displayString[i - 1];
            }
            displayString[cursorPosition] = key;
            cursorPosition++;
            charCount++;
            displayString[charCount] = '\0'; // Asegura el terminador nulo para el buffer

            Prepare_LCD_Pins();
            LCD_clear();
            LCD_setCursor(1, 0);
            LCD_print(displayString);
            LCD_setCursor(1, cursorPosition); // Actualiza la posición del cursor en el LCD
        }
    }
}



void I2C_Slave_Init(unsigned char address) {
    SSPADD = address;  // Establece la direcci?n del esclavo I2C
    SSPCON = 0x36;  // 0b00110110 -> SSPEN = 1, SSPOV = 0, SSPM3:SSPM0 = 0110 (I2C Slave mode, 7-bit address)
    SSPSTAT = 0x00;
    TRISC3 = 1;  // Configura los pines SCL y SDA como entradas
    TRISC4 = 1;
    PEIE = 1;  // Habilita interrupciones perif?ricas
    SSPIE = 1;  // Habilita interrupci?n de SSP (I2C)
    GIE = 1;  // Habilita interrupciones globales
}

void I2C_Slave_Write(unsigned char data) {
    SSPBUF = data;
    while(SSPIF == 0);  // Espera hasta que la transmisi?n se complete
    SSPIF = 0;
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
                    LCD_clear();
                    LCD_setCursor(1, 0);
                    LCD_print(receiveBuffer);
                    index = 0;
                    allow_input = true;  // Permite la entrada despu?s de 3 segundos
                __delay_ms(2000);
                LCD_clear();
                LCD_setCursor(1,0);
                LCD_print("Escriba res-");
                LCD_setCursor(2,0);
                LCD_print("para maestro...");
                __delay_ms(2000);
                LCD_clear();
                LCD_setCursor(1,0);
                
                // Cambia a modo maestro para enviar respuesta
                    i2c_role = MASTER_MODE;
                    I2C_Master_Init(100000);
                }
                
            }
        } 
        SSPIF = 0;  // Limpia la bandera de interrupci?n
    }
}


// Implementaci?n de funciones para el LCD

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


void Prepare_LCD_Pins() {
    // Configura los pines para el uso del LCD antes de usarlo para enviar comandos o datos
    TRISB = 0x00; // Configura PORTB como salida
}
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


void Prepare_Keypad_Pins() {
    // Configura los pines para el uso del teclado antes de leerlo
    TRISB = 0xF0; // RB0-RB3 como salida (para filas), RB4-RB7 como entrada (para columnas)
    PORTB = 0x0F; // Baja las filas para activar el teclado, activa resistencias de pull-up para columnas
}


// Asegurar que el esclavo, al convertirse en maestro, env?e mensajes correctamente
void Send_Message_To_Master(char *message) {
    // Cambia a modo maestro
    i2c_role = MASTER_MODE;
    I2C_Master_Init(100000);  // Inicializa I2C con la configuraci?n de maestro

    I2C_Master_Start();
    I2C_Master_Write(0x20);  // Env?a la direcci?n del maestro con el bit de escritura
    while (*message) {
        I2C_Master_Write(*message++);  // Env?a cada car?cter del mensaje
    }
    I2C_Master_Write('\0');  // Env?a el car?cter de terminaci?n
    I2C_Master_Stop();

    // Cambiar de vuelta a esclavo despu?s de enviar el mensaje
    i2c_role = SLAVE_MODE;
    I2C_Slave_Init(SLAVE_I2C_ADDRESS);
}


void I2C_Master_Init(const unsigned long c) {
    SSPCON = 0b00101000;  // SSPEN=1, I2C Master mode, clock = FOSC/(4 * (SSPADD+1))
   SSPADD = (unsigned char)((_XTAL_FREQ / (4 * c)) - 1);  // Aseg?rate de que el resultado sea v?lido para unsigned char

    SSPSTAT = 0x00;
    TRISC3 = 1;  // Configura pines SCL y SDA como entradas
    TRISC4 = 1;
}


void I2C_Master_Start(void) {
    I2C_Master_Wait();
    SEN = 1;  // Inicia la condici?n de START
}

void I2C_Master_Stop(void) {
    I2C_Master_Wait();
    PEN = 1;  // Inicia la condici?n de STOP
}

void I2C_Master_Write(unsigned char data) {
    I2C_Master_Wait();
    SSPBUF = data;  // Coloca el dato en el buffer
}

void I2C_Master_Wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));  // Espera a que se complete la operaci?n actual
}