/* #include "float_str.h"
*
* Creada por: Ing. Abiezer Hernandez O.
* Fecha de creacion: 25/02/2020
* Electronica y Circuitos
*
*/

void floattostr(float numero_, unsigned char *cadena_,char decimales_);

void floattostr(float numero_, unsigned char *cadena_,char decimales_)
{
    int largo_entera,largo_n,cont_for,tempo_int;
    double tempo_float;
    largo_n = decimales_+1;
    largo_entera = 0;

    if(numero_ < 0)
    {
        *cadena_++ = '-';
        numero_ = -numero_;
    }
    if(numero_ > 0.0) while (numero_ < 1.0)
    {
        numero_ =numero_* 10.0;
        largo_entera--;
    }
    while(numero_ >= 10.0)
    {
        numero_ = numero_/10.0;
        largo_entera++;
    }
    largo_n = largo_n+largo_entera;
    
    for(tempo_float = cont_for = 1; cont_for < largo_n; cont_for++)
        tempo_float = tempo_float/10.0;
    numero_ += tempo_float/2.0;
    if(numero_ >= 10.0)
    {
        numero_ = 1.0; largo_entera++;
    }
    if(largo_entera<0)
    {
        *cadena_++ = '0'; *cadena_++ = '.';
        if(largo_n < 0) largo_entera = largo_entera-largo_n;
        for(cont_for = -1; cont_for > largo_entera; cont_for--) *cadena_++ = '0';
    }
    for(cont_for=0; cont_for < largo_n; cont_for++)
    {
        tempo_int = numero_;
        *cadena_++ = tempo_int + 48;
        if (cont_for ==  largo_entera ) *cadena_++ = '.';
        numero_ -= (tempo_float=tempo_int);
        numero_ = numero_*10.0;
    }
    *cadena_ = 0;
}