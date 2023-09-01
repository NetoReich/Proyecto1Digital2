/*
 * File:   main_master.c
 * Author: Johann Haeussler y Marcela Padilla
 *
 * Created on , 13 de agosto 2023
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "I2C.h" //Librería de I2C

#define _XTAL_FREQ 8000000 //Frecuencia 8MHz
#define TRIGGER_PIN PORTDbits.RD3 //Pin de TRIGGER
#define ECHO_PIN PORTDbits.RD2 //Pin de ECHO
//SLAVE2 SLAVE2 SLAVE2
uint8_t z; //Variable para guardar datos temporales de I2C
uint8_t recibido, enviado, dis; //Variables de la comunicacion I2C

void setup(void); //Función del setup
void delay(unsigned int micro); //Función de delay variable
float ultrasonic_measure_distance(void); //Función para obtener distancia

void main(void){
    setup(); //Llmar al setup
    while(1){ //LOOP
    }
}

void __interrupt() isr(void){ //Interrupciones
    if(PIR1bits.SSPIF == 1){ //Verificar si es interrupción del I2C

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            //__delay_us(7);
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            //__delay_us(2);
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            recibido = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF; //Guardar temporalmente
            BF = 0;
            SSPBUF = ultrasonic_measure_distance(); //Enviar distancia
            SSPCONbits.CKP = 1; //Habilitar SCL
            __delay_us(250); //delay de 250 us
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0; //Limpiar bandera
    }
    if (INTCONbits.T0IF == 1){ //chequear interrupción del Timer0
        if (ultrasonic_measure_distance() < 8.0){
            PORTAbits.RA0 = 1; //encender puerto
            delay(25); // delay (tiempo en alto del pulso)
            PORTAbits.RA0 = 0; //apagar
        }
        else {
            PORTAbits.RA0 = 1; //encender puerto
            delay(10); // delay (tiempo en alto del pulso)
            PORTAbits.RA0 = 0; //apagar
        }
        
        INTCONbits.T0IF = 0; // limpiar bandera
        TMR0 = 100; //asignar valor al timer0
    }
}

void setup(void){
    ANSEL = 0; //Puertos digitales
    ANSELH = 0; //Puertos digitales
    
    TRISA = 0; //Puerto a como salida
    TRISDbits.TRISD3 = 0; //D3 como salida
    TRISDbits.TRISD2 = 1; //D2 como entrada
            
    PORTA = 0; //Limpiar Puerto A
    PORTB = 0; //Limpiar Puerto B
    PORTC = 0; //Limpiar Puerto C
    PORTD = 0; //Limpiar Puerto D
    
    OSCCONbits.IRCF2 = 1; //Frecuencia de 8MHz
    OSCCONbits.IRCF1 = 1; 
    OSCCONbits.IRCF0 = 1;
    
    OSCCONbits.SCS = 1; //Utilizar reloj interno
    
    INTCONbits.T0IE = 1; //Activar interrupciones del timer0
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del Timer0
    
    OPTION_REGbits.T0CS = 0; //Usar Timer0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Prescaler con el Timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    TMR0 = 100;
    
    T1CON = 0b00000000;

    I2C_Slave_Init(0xB0); //Inicar esclavo y definir su direccións
}

//FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){ //mientras el delay no sea 0
        __delay_us(50); //delay de 250us
        micro--; //decrementar variable
    }
}

float ultrasonic_measure_distance(void){
    uint16_t pulse_duration; //Variable para guardar la duración del pulso
    float distance; //Variable para guardar distancia

    // Send a 10us pulse to trigger pin
    TRIGGER_PIN = 1; //Poner el pin en 1
    __delay_us(10); //Enviar 1 por 10 us
    TRIGGER_PIN = 0; //Apagar Pin

    while (ECHO_PIN == 0); //Mientras sea 0 no hacer nada

    TMR1L = 0x00; //Timer1 en 0
    TMR1H = 0X00;
    T1CONbits.TMR1ON = 1; //Iniciar Timer1
    while ((ECHO_PIN == 1)); //Mientras sea 1 guardar Timer 1 se incrementa
    pulse_duration = (TMR1H << 8 ) + TMR1L; //Guardar variable de Timer1
    T1CONbits.TMR1ON = 0; //Apagar timer1
    distance = (pulse_duration*0.5*1)/58; //Calcular distancia

    return distance; //Retornar distancia
}