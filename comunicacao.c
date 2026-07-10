#include <stdio.h>
#include "comunicacao.h"
#include "driver/uart.h"

#define PORTA UART_NUM_0

// Garante que a estrutura terá exatamente o tamanho dos dados
typedef struct __attribute__((packed)) 
{
    char header[2]; // Sempre conterá os caracteres 'S' e 'G' (Sinal Gráfico)
    float tensao;
    char blackout;
    char surtos;
} PacoteDados;

void uart_init()
{
    // 1. Configuração dos parâmetros de comunicação
    uart_config_t uart_config = {
        .baud_rate = 921600,                  // Alterado de 115200 para 921600
        .data_bits = UART_DATA_8_BITS,        
        .parity = UART_PARITY_DISABLE,        
        .stop_bits = UART_STOP_BITS_1,        
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
    };

    // Aplica as configurações na porta
    ESP_ERROR_CHECK(uart_param_config(PORTA, &uart_config));
    
    // 2. Configuração dos Pinos
    // UART_PIN_NO_CHANGE para manter os pinos padrão da UART0 (geralmente TX = GPIO1, RX = GPIO3)
    ESP_ERROR_CHECK(uart_set_pin(PORTA, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 3. Instalação do Driver
    // Buffer de TX de 256 bytes é insuficiente para vários pacotes de 12 bytes.
    const int uart_buffer_size = 2048;
    ESP_ERROR_CHECK(uart_driver_install(PORTA, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
}

void comunicacao_serial(float* v, char* blackout, char* surtos)
{
    /*
    Função responsável por enviar os dados de tensão, blackout e surtos para o Python via UART.
    */
    PacoteDados pacote;
    pacote.header[0] = 'S';
    pacote.header[1] = 'G';
    pacote.tensao = *v;
    pacote.blackout = *blackout;
    pacote.surtos = *surtos;
    
    // Envio direto da memória para a porta serial (UART_NUM_0)
    uart_write_bytes(PORTA, &pacote, sizeof(PacoteDados));
}
