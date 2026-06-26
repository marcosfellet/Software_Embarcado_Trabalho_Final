#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "freertos/stream_buffer.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ADC_UNIT                ADC_UNIT_1 
#define EXAMPLE_ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1 
#define EXAMPLE_ADC_ATTEN               ADC_ATTEN_DB_12 
#define EXAMPLE_ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH 
#define EXAMPLE_READ_LEN                256 

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6}; 
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

// static const char *TAG = "CONT_ADC";
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

StreamBufferHandle_t buffer;

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096, 
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t pattern[1];
    pattern[0].atten = EXAMPLE_ADC_ATTEN;
    pattern[0].channel = channel[0];
    pattern[0].unit = EXAMPLE_ADC_UNIT;
    pattern[0].bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000, // 20kHz continuo
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .pattern_num = 1,
        .adc_pattern = pattern
    };
    
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;
}


void vtaskADC(void *pvparameters)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0}; 

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &adc_handle); 
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while(1)
    {
        // Concentra-se puramente em colher os dados brutos. Sem travas de tempo.
        ret = adc_continuous_read(adc_handle, result, EXAMPLE_READ_LEN, &ret_num, portMAX_DELAY);

        if (ret == ESP_OK && ret_num > 0) {
            // Envia o bloco bruto. Se o buffer encher, espera até 10ms (evita travar a CPU para sempre)
            xStreamBufferSend(buffer, result, ret_num, pdMS_TO_TICKS(10));
        }
    }
}

void vtaskProcessamento(void *pv)
{
    uint8_t raw_frame[EXAMPLE_READ_LEN]; 
    static adc_continuous_data_t parsed_data[EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES];
    
    uint32_t contador_print = 0;
    int soma_tensao = 0;
    int amostras_acumuladas = 0;

    while (1)
    {
        // CORREÇÃO DO WATCHDOG: A task fica bloqueada aqui (portMAX_DELAY) se não houver dados.
        // Isso dá tempo para o IDLE do FreeRTOS rodar e alimentar o Watchdog.
        size_t bytes_recebidos = xStreamBufferReceive(buffer, raw_frame, EXAMPLE_READ_LEN, portMAX_DELAY);

        if (bytes_recebidos > 0) {
            uint32_t num_parsed_samples = 0;
            esp_err_t parse_ret = adc_continuous_parse_data(adc_handle, raw_frame, bytes_recebidos, parsed_data, &num_parsed_samples);

            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (parsed_data[i].valid) {
                        int tensao_mv = 0;
                        adc_cali_raw_to_voltage(cali_handle, parsed_data[i].raw_data, &tensao_mv);
                        
                        // Acumula para tirar uma média em vez de dar printf em todas as 20.000 amostras
                        soma_tensao += tensao_mv;
                        amostras_acumuladas++;
                    }
                }
            }
            
            // A cada ~2000 amostras (aprox. 100ms), printamos a média calculada
            contador_print += bytes_recebidos;
            if (contador_print >= 4000) { 
                if (amostras_acumuladas > 0) {
                    int media = soma_tensao / amostras_acumuladas;
                    printf("Média Tensão: %d mV | Amostras: %d\n", media, amostras_acumuladas);
                }
                contador_print = 0;
                soma_tensao = 0;
                amostras_acumuladas = 0;
            }
        }
    }
}

void app_main(void)
{
    // Calibração iniciada na Main
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    // Buffer de 4KB para transição suave de dados contínuos
    buffer = xStreamBufferCreate(4096, EXAMPLE_READ_LEN); 
    
    // Na leitura contínua, deixamos as duas tasks com a mesma prioridade (5)
    // para que o FreeRTOS faça o escalonamento por fatias de tempo (Round-Robin) automaticamente
    xTaskCreate(vtaskADC, "Task ADC", 4096, NULL, 5, NULL);
    xTaskCreate(vtaskProcessamento, "PROC", 4096, NULL, 5, NULL);
}
