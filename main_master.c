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
//#include <pic16f887.h>
#include "LCD.h" //librería LCD
#include "I2C.h"
#include "float_str.h" //Funcion para convertir cadena a texto

#define _XTAL_FREQ 8000000 //Frecuencia 8MHz

uint8_t pantalla = 0; //Variable para determinar que se muestra en la pantalla
float distancia; //Variable para guardar la distancia del sensor ultrasónico
uint8_t cont = 0; //Variable para guardar cantidad de personas
uint8_t contador = 0; //Variable para guardar cantidad de personas
uint8_t bandera = 0; //Variable para antirrebotes
uint8_t det = 0; //Variable para determinar si ya paso el delay de 3 segundos del motor
uint8_t time = 0; //Variable del timer0 para realizar delay de 3s
float temperatura; //Variable para guardar valor de temperatura
uint8_t segundos, minutos, horas; //Variable para gaurdar minutos, segundos y horas
char buffer[30]; //Buffer para convertir de numeros a cadena
char buffer2[48]; //Buffer para convertir de numeros a cadena
char buffer3[48]; //Buffer para convertir de numeros a cadena

void setup(void);
void Set_sec(uint8_t sec); //Función para setear segundos
void Set_min(uint8_t min); //Función para setear minutos
void Set_hour(uint8_t hour); //Función para setear horas
void Read_Time(uint8_t *s, uint8_t *m, uint8_t *h); //Función para obtener valores de tiempo
uint8_t Dec_to_Bcd(uint8_t dec_number); //Función para pasar de decimal a bcd
uint8_t Bcd_to_Dec(uint8_t bcd); //Función para pasar de bcd a decimal
void AHT10_Init(void); //Inicializar el sensor AHT10
void AHT10_Soft_Reset(void); //Función para resetear el sensor de temperatura
void AHT10_Read(void); //Función para leer y desplegar temperatura
void Slave1_Total(void); //Función para leer la cantidad de personas que han pasado
void Slave2(void); //Función para leer distancia del sensor ultrasónico
void ESP32_Write(void); //Función para enviar datos al ESP32
void floatToString(float value, char* buffer, int decimalPlaces); //Función para convertir de float a cadena de texto

void main(void){
    setup(); //Llamar al setup
    Lcd_Init(); //Función para inicializar LCD
    AHT10_Soft_Reset(); //Resetear el sensor de temperatura
    AHT10_Init(); //Inicar el sensor de temperatura
    Set_sec(0); //Setear segundos en 0
    Set_min(0); //Setear minutos en 0 
    Set_hour(0); //Setear horas en 0
    __delay_ms(1000); //delay de 1s
    while(1){
        contador++;
        Slave1_Total(); //Llamamos para leer la cantidad de personas
        Read_Time(&segundos, &minutos, &horas); //Tomar el tiempo
        AHT10_Read(); //Tomar la temperatura
        Slave2(); //Leer distancia del sensor ultrasónico
        __delay_ms(10); //delay de 10ms
        ESP32_Write(); //Función para enviar datos al ESP32
           
        if (pantalla == 0){ //Determinar que valores mostrar en la lcd
            Lcd_Set_Cursor(1,10); //Setear cursor en 1,10
            Lcd_Write_String("T:"); //Escribir una T para temperatura en la lcd
            
            Lcd_Set_Cursor(1,12); //Setear cursor en 1,12
            Lcd_Write_String(buffer2); //Mostrar temperatura
            
            Lcd_Set_Cursor(1,1); //Setear cursor en 1,1
            sprintf(buffer, "%02u:%02u:%02u", horas, minutos, segundos);//Convertir segundos, minutos y horas a cadena
            Lcd_Write_String(buffer); //Mostrar en Lcd
            
            Lcd_Set_Cursor(2,1); //Setear cursor en 2,1
            sprintf(buffer, "%02u", cont); //Convertir numero de personas cadena
            Lcd_Write_String(buffer); //Mostrar en Lcd
        }
        
        else if (pantalla == 1){ //Si se presiona el boton mostrar estos botones
            Lcd_Set_Cursor(1,10); //Setear cursor en 1,10
            Lcd_Write_String("T:"); //Escribir T
            
            floattostr(distancia, buffer3, 2); //Convertir distancia a cadena
            Lcd_Set_Cursor(1,1); //Setear cursor en 1,1
            Lcd_Write_String(buffer3); //Mostrar en Lcd
        }
    }
}

void __interrupt() isr(void){ //Interrupciones
    if (INTCONbits.RBIF == 1){ //Verificar si es interrupción del puerto B
        INTCONbits.RBIF = 0; //Limpiar bandera de interrupción
        if (PORTBbits.RB3 == 0){ //Ver si se presionó botón en RB3
            __delay_ms(20); //delay de 20 ms
            bandera = 1; //Encender bandera antirrebote
        }
        else if ((PORTBbits.RB3 == 1) && (bandera == 1)){ // Verificar si se dejo de presinar el botón 
            
            if (pantalla == 0){ //si pantalla estaba en modo 0
                pantalla = 1; //Cambiar a modo 1
                Lcd_Clear(); //Limpiar LCD
                Lcd_Init(); //Inicializar de nuevo
                __delay_ms(400); //delay de 400ms
            } 
            else if (pantalla == 1){ //si pantalla estaba en modo 1             
                pantalla = 0; //Cambiar a modo 0
                Lcd_Clear(); //Limpiar LCD
                Lcd_Init(); //Inicializar de nuevo
                __delay_ms(400); //delay de 400ms
            }
            bandera = 0; //Limpair bandera de botón
        }
    }
    if (INTCONbits.T0IF == 1){ //Verificar si es interrupción del Timer0
        time++; //Incrementar variable para hacer delay de 3s
        if (time == 175){ //Verificar si variable llego a 175
            det = 0; //Variable para que el motor continue
            time = 0; //Resetear variable
            INTCONbits.T0IE = 0; //Detener Timer 0
        }
        INTCONbits.T0IF = 0; //Limpiar variable del Timer0
        TMR0 = 100; //Cargar valor para delay de 20ms
    }
}

void setup(void){
    ANSEL = 0; //Puertos como I/O digitales
    ANSELH = 0; //Puertos como I/O digitales
    
    TRISB = 0b00001000; //Puerto B como salida excepto RB3
    TRISD = 0; //Puerto D como salida
    TRISE = 0;
     
    PORTA = 0; //Limpiar puerto A
    PORTB = 0; //Limpiar puerto B
    PORTC = 0; //Limpiar puerto C 
    PORTD = 0; //Limpiar puerto D
    PORTE = 0; //Limpiar puerto D

    
    OSCCONbits.IRCF2 = 1; //Frecuencia en 8MHz
    OSCCONbits.IRCF1 = 1; 
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1; //Usar oscilador interno
    
    OPTION_REGbits.T0CS = 0; //Utilzar el timer 0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Utilizar prescaler con timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    INTCONbits.GIE = 1; //Activar interrupciones globales
    INTCONbits.RBIF = 0; //Limpiar bandera de interrupcion del puerto B
    INTCONbits.RBIE = 1; //Activar interrupciones del puerto B
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del puerto B
    
    OPTION_REGbits.nRBPU = 0; //Activar pull-ups del puerto B
    
    IOCBbits.IOCB3 = 1; //Activar interrupt on-change del pin RB3
    WPUBbits.WPUB3 = 1; //Activar pull-up del pin RB3

    I2C_Master_Init(100000); //Setear I2C a 100kHz
}

uint8_t Read(uint8_t address){ //Función para obtener datos
    uint8_t dato = 0; //Variable temporal
    I2C_Master_Start(); //Inicializar comunicación I2C //Iniciar i2c
    I2C_Master_Write(0xD0); //Introducir dirección del esclavo
    I2C_Master_Write(address); //Introducir dirección 
    I2C_Master_RepeatedStart(); //Restart i2c
    I2C_Master_Write(0xD1); //Introducir dirección del esclavo más bit de escritura
    dato = I2C_Master_Read(0); //Almacenar dato en variable temporal
    I2C_Nack(); //Enviar Nack //Encender bit de not aknowledge e iniciar secuencia de reconocimiento  y transimitir el bit de reconocimiento
    I2C_Master_Stop(); //detener comunicacion //Stop i2c
    __delay_us(10); //delay de 10 us
    return dato; //Retornar dato
}

void Read_Time(uint8_t *s, uint8_t *m, uint8_t *h){ //Función de obtener valores del tiempo
    *s = Bcd_to_Dec(Read(0x00)); //Obtener segundos
    *m = Bcd_to_Dec(Read(0x01)); //Obtener minutos
    *h = Bcd_to_Dec(Read(0x02)); //Obtener horas
}

void Set_sec(uint8_t sec){ //Función para setear segundo
    I2C_Master_Start(); //Inicializar comunicación I2C //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x00); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(sec)); //Mandar dato en BCD
    I2C_Master_Stop(); //detener comunicacion //Terminar i2c
}

void Set_min(uint8_t min){ //Función para setear minutos
    I2C_Master_Start(); //Inicializar comunicación I2C //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x01); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(min)); //Mandar dato en BCD
    I2C_Master_Stop(); //detener comunicacion //Terminar i2c
}

void Set_hour(uint8_t hour){ //Función para setear horas
    I2C_Master_Start(); //Inicializar comunicación I2C //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x02); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(hour)); //Mandar dato en BCD
    I2C_Master_Stop(); //detener comunicacion //Terminar i2c
}

uint8_t Dec_to_Bcd(uint8_t dec_number){ //Función para pasar de numero decimal a bcd
    uint8_t bcd_number; //Variable para almacenar dato bcd
    bcd_number = 0; //Limpiar numero
    while(1){ //Loop
        if (dec_number >= 10){ //Convertir numero y repetir ciclo hasta que el numero sea menor que 10
            dec_number = dec_number - 10; //Restar 10
            bcd_number = bcd_number + 0b00010000; //Ir sumando diez en bcd
        }
        else { //Suma de números
            bcd_number = bcd_number + dec_number; //Suma
            break; //Salirse del loop
        }
    }
    return bcd_number; //Retornar valor BCD
}

uint8_t Bcd_to_Dec(uint8_t bcd){ //Función para pasar números de bcd a decimal
    uint8_t dec; //Variable para guardar valor
    dec = ((bcd>>4)*10)+(bcd & 0b00001111); // Hacer un corrimiento de bits y sumar con la unidad
    return dec; //Retornar valor
}

void AHT10_Init(void){ //Función para inicializar sensor de temperatura
    __delay_ms(40); //delay de 40ms
    uint8_t status; //variable para status del sensor
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0x71); //Enviar para obtener el status del sensor
    I2C_Master_RepeatedStart(); //Repeated Start
    I2C_Master_Write(0x71); //Leer del sensor de temperatura
    status = I2C_Master_Read(0); //Guardar status
    I2C_Nack(); //Enviar Nack
    I2C_Master_Stop(); //detener comunicacion
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xE1); //Enviar secuencia de inicialización
    I2C_Master_Write(0x08);
    I2C_Master_Write(0x00);
    I2C_Master_Stop(); //detener comunicacion

    __delay_ms(10); //delay de 10ms

}

void AHT10_Read(void){ //Función para leer
    uint8_t data[7]; //arreglo para guardar los datos recibidos del sensor de temperatura
    uint8_t r; //Variable para determinar si el sensor está listo para volver a realizar una medición
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xAC); //Enviar secuencia de medición
    I2C_Master_Write(0x33);
    I2C_Master_Write(0x00);
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(80); //delay de 80ms
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0x71); //Secuencia para obtener status
    I2C_Master_RepeatedStart(); //Repeated start
    I2C_Master_Write(0x71); //Dirección mas bit de escritura
    r = I2C_Master_Read(0); //Guardar status
    I2C_Nack(); //Enviar Nack
    I2C_Master_Stop(); //detener comunicacion
    
    r = r & 0b00000000; //convertir todas las variables en 0
    while (r != 0b00000000); //Mientras sean 0 no hacer nada
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x71); //Enviar dirección mas bit de escritura
    data[0] = I2C_Master_Read(0); //Guardar
    I2C_Ack(); //Enviar acknowledge bit
    data[1] = I2C_Master_Read(0); //Guardar status
    I2C_Ack(); //Enviar acknowledge bit
    data[2] = I2C_Master_Read(0); //Guardar valor de humedad 1
    I2C_Ack(); //Enviar acknowledge bit
    data[3] = I2C_Master_Read(0); //Guardar valor de humedad 2
    I2C_Ack(); //Enviar acknowledge bit
    data[4] = I2C_Master_Read(0); //Guardar valor de humedad 3 y temperatura 1
    I2C_Ack(); //Enviar acknowledge bit
    data[5] = I2C_Master_Read(0); //Guardar valor de temperatura 2
    I2C_Ack(); //Enviar acknowledge bit
    data[6] = I2C_Master_Read(0); //Guardar valor de temperatura 3
    I2C_Nack(); //Enviar Nack
    I2C_Master_Stop(); //detener comunicacion
    
	temperatura = (((uint32_t)data[3] & 0x0F) << 16) + ((uint16_t)data[4] << 8) + data[5]; //Unir datos de temperatura en uno solo
    temperatura = ((temperatura/1048576)*200-50); //Realizar conversión indicada por el fabricante
    floattostr(temperatura, buffer2, 2); //Convertir el dato a cadena
    Lcd_Set_Cursor(1,12); //Setear cursor en 1,12
    Lcd_Write_String(buffer2); //Mostrar en LCD
}

void AHT10_Soft_Reset(void){ //Función de reset 
    __delay_ms(40); //delay de 40ms
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xBA);//Enviar secuencia de reset
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(25); //delay de 25ms
}

void Slave1_Total(void){ //Función para obtener numero de personas
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0xA0); //Enviar dirección de esclavo
    I2C_Master_Write(0x03); //Enviar 3 para indicar que dato recibir
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(20);
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0xA1); //Enviar dirección mas bit de escritura
    cont = I2C_Master_Read(0); //Leer cantidad de personas
    I2C_Master_Stop(); //detener comunicacion
    __delay_us(10); //delay de 10us
}
void Slave1_Return(void){
    if (det == 0 && distancia < 15.0){
        I2C_Master_Start(); //Inicializar comunicación I2C
        I2C_Master_Write(0xA0); //Enviar dirección de esclavo
        //I2C_Master_Write(0x06); //Enviar 6 para detener motor dc
        I2C_Master_Stop(); //detener comunicacion
        __delay_ms(20); //delay de 20ms
        det = 1; //Variable para indicar que han pasado 3s
        time = 0; //Iniciar conteo de 3s
        INTCONbits.T0IE = 1; //Iniciar timer
        TMR0 = 100; //Cargar valor para delay de 100s
    }
}
void Slave2(void){ //Función para recibir distancia del sensor ultrasónico
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0xB1); //Dirección más bit de escritura
    distancia = I2C_Master_Read(0); //Guardar distancia
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(20); //delay de 20ms
}


void ESP32_Write(void){ //Función para enviar datos al microcontrolador ESP32
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x60); //Dirección del esclavo
    I2C_Master_Write(distancia); //Enviar distancia
    I2C_Master_Write(cont); //Enviar numero de persosnas
    I2C_Master_Write(temperatura); //Enviar temperatura
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(10); //delay de 10ms
}