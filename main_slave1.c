/*
 * File:   main_master.c
 * Author: Johann Haeussler y Marcela Padilla
 *
 * Created on , 13 de agosto 2023
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF              // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF              // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                 // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF              // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "I2C.h" //incluir librería de I2C
#define _XTAL_FREQ 8000000 //Frecuencia 8MHz
//SLAVE SLAVE SLAVE

uint8_t z; //Variable para guardar dato de la comunicación I2C
uint8_t bandera = 0; //Variable para indicar el delay de 3s de detención del motor
uint8_t time = 0; //Varaible para obtener delay de 3s
uint8_t cont, dc; //Variables del regristo, variable motor
uint8_t antireb = 0; // Variable Antirebote
void setup(void); //Definir Función del setup

void main(void){
    setup(); //Llamar a setup
    cont = 0; //Numero de personas que entran a la bodega (contador)
    dc = 1; //Se inicia con motor dc activo
    PORTAbits.RA1 = 1; //Encender pin para activar dc
    __delay_ms(2000); //delay de 2s
    while(1){
        while (dc == 1){
            if (PORTBbits.RB7 == 1){ //Si se detecta un movimiento en el sensor
                PORTAbits.RA1 = 0; // encender motor
                time = 0; //Iniciar delay de 3s
                INTCONbits.T0IF = 0; //Limpiar bandera de interrupción de Timer0
                INTCONbits1.T0IE = 1; //Iniciar timer
                TMR0 = 100; //Cargar valor para delay de 20ms
                bandera = 1; //Encender bandera
            }
            if (bandera == 0){ // Se apaga la  bandera
                INTCONbits.T0IF = 0; //Limpiar bandera del timer0
                INTCONbits.T0IE = 0; //Apagar timer0
                PORTAbits.RA1 = 1; // Apagar motor
            }
            if (PORTBbits.RB7 == 1){ //Detectó movimiento el sensor infrarrojo
                PORTCbits.RC1 = 0; //Encender led
            }
            else { //No se detectó movimiento
                PORTCbits.RC1 = 1; //Apagar led
                
            }
        }
    }
}
void __interrupt() isr(void){ //Interrupciones
    if(PIR1bits.SSPIF == 1){ //Interrupcón del módul MSSP

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){ //Condición para escibirle al maestro
            z = SSPBUF;
            BF = 0;
            SSPBUF = cont;              // Se escribe en el registro el valor contenido en la variable
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0; //Limpiar bandera    
    }
    if (INTCONbits.RBIF == 1){ //Verificar si es interrupción del puerto B
        if (PORTBbits.RB7 == 1){ //Ver si se detectó movimiento
            if(!antireb){
            antireb = 1;            //Antirrebote
            __delay_ms(500);
            }
        }else
        {
             cont++; //Incrementar numero de personas
             antireb = 0;
            __delay_ms(500);
            }
            INTCONbits.RBIF = 0; //Limpiar bandera
    }
    if (INTCONbits.T0IF == 1){ //Ver si es interrupción del Timer0
        time++; //Incrementar variable
        if (time == 150){ //Ver si llego a 3s
            bandera = 0; //Limpiar bandera para activar motor dc
            time = 0; //Resetear cuenta
        }
        INTCONbits.T0IF = 0; //Limpiar bandera
        TMR0 = 100; //Cargar valor para delay de 3s
    }
}

void setup(void){
    ANSEL = 0; //Puertos como digitales
    ANSELH = 0; //Puertos como digitales
    
    TRISAbits.TRISA0 = 0; //Pin A0 como salida
    TRISAbits.TRISA1 = 0; //Pin A1 como salida
    TRISCbits.TRISC2 = 0; //Pin C2 como salida
    TRISCbits.TRISC1 = 0; //Pin C1 como salida
    TRISBbits.TRISB7 = 1; //Pin RB7 como entrada
    
    PORTA = 0; //Limpiar puerto A
    PORTB = 0; //Limpiar puerto B
    PORTC = 0; //Limpiar puerto C
    PORTD = 0; //Limpiar puerto D
    
    
    INTCONbits.RBIF = 0; //Limpiar bandera de interrupción del timer0
    INTCONbits.RBIE = 1; //Habilitar interrupción del puerto B
    
    OPTION_REGbits.T0CS = 0; //Utilizar timer 0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Utilizar prescaler con el timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    
    IOCBbits.IOCB7 = 1; //Interrupt on-change del pin RB7
    
    OSCCONbits.IRCF2 = 1; //Frecuencia de 8MHz
    OSCCONbits.IRCF1 = 1; 
    OSCCONbits.IRCF0 = 1;
    
    OSCCONbits.SCS = 1; //Utilizar reloj interno
    I2C_Slave_Init(0xA0);  //Inicar esclavo y definir su dirección
    
    TMR0 = 100; //Valor a cargar para delay de 20ms
}