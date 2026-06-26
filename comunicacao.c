#include <stdio.h>
#include "comunicacao.h"
#include "driver/uart.h"

#define PORTA UART_NUM_0

// Garante que a estrutura terá exatamente o tamanho dos dados
typedef struct __attribute__((packed)) 
{
    char header[2]; // Sempre conterá os caracteres 'S' e 'G' (Sinal Gráfico)
    float tensao;
    float corrente;
    char blackout;
    char surtos;
} PacoteDados;


void comunicacao_serial(float* v, float* i, char* blackout, char* surtos)
{
    PacoteDados pacote;
    pacote.header[0] = 'S';
    pacote.header[1] = 'G';
    pacote.tensao = *v;
    pacote.corrente = *i;
    pacote.blackout = *blackout;
    pacote.surtos = *surtos;
    // Envio direto da memória para a porta serial (padrão: UART_NUM_0)
    uart_write_bytes(PORTA, &pacote, sizeof(PacoteDados));

}
