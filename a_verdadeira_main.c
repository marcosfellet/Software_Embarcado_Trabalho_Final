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
#include <math.h>

// Bibliotecas para o Display Oled
#include "ssd1306.h"
#include "driver/i2c.h"

// Bibliotecas para o Display Oled
#include "ssd1306.h"
#include "driver/i2c.h"

// Variável do display
SSD1306_t device;

// Variável Stream Buffer do oled
StreamBufferHandle_t buffer_oled;

// Inicializa o display OLED via I2C
void display_init(void) 
{
    i2c_master_init(&device, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

	ssd1306_init(&device, 128, 64);

    ssd1306_clear_screen(&device, false);
	ssd1306_contrast(&device, 0xff);
}


#define EXAMPLE_ADC_UNIT                ADC_UNIT_1 
#define EXAMPLE_ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1 
#define EXAMPLE_ADC_ATTEN               ADC_ATTEN_DB_12 
#define EXAMPLE_ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH 
#define EXAMPLE_READ_LEN                256 

// MATEMÁTICA DO SINAL: 20000Hz / 60Hz = 333.33 amostras por período
#define SAMPLES_PER_PERIOD              333 

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6}; 
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

static const char *TAG = "SENOIDE_60HZ";
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
        .sample_freq_hz = 20000, 
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .pattern_num = 1,
        .adc_pattern = pattern
    };
    
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;
}

// TASK 1: Produtora (idêntica à anterior, focada em velocidade)
void vtaskADC(void *pvparameters)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0}; 

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &adc_handle); 
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while(1)
    {
        ret = adc_continuous_read(adc_handle, result, EXAMPLE_READ_LEN, &ret_num, portMAX_DELAY);
        if (ret == ESP_OK && ret_num > 0) {
            xStreamBufferSend(buffer, result, ret_num, pdMS_TO_TICKS(10));
        }
    }
}

// TASK 2: Consumidora com Janelamento de Período de 60Hz
void vtaskProcessamento(void *pv)
{
    uint8_t raw_frame[EXAMPLE_READ_LEN]; 
    static adc_continuous_data_t parsed_data[EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES];
    
    // EVOLUÇÃO: Variáveis de controle do período da senoide
    int periodo_senoide[SAMPLES_PER_PERIOD] = {0};
    float valor_real[SAMPLES_PER_PERIOD] = {0};
    int sample_idx = 0;
    uint32_t total_periodos_capturados = 0;

    // Variáveis do calculo RMS
    float vrms = 0;
    float aux = 0;
    int n =0;

    while (1)
    {
        // Bloqueia eficientemente até receber dados do buffer
        size_t bytes_recebidos = xStreamBufferReceive(buffer, raw_frame, EXAMPLE_READ_LEN, portMAX_DELAY);

        if (bytes_recebidos > 0) {
            uint32_t num_parsed_samples = 0;
            esp_err_t parse_ret = adc_continuous_parse_data(adc_handle, raw_frame, bytes_recebidos, parsed_data, &num_parsed_samples);

            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (parsed_data[i].valid) {
                        int tensao_mv = 0;
                        adc_cali_raw_to_voltage(cali_handle, parsed_data[i].raw_data, &tensao_mv);
                        
                        // Armazena a amostra atual calculada dentro do vetor do período
                        periodo_senoide[sample_idx] = tensao_mv;
                        sample_idx++;

                        // CONDIÇÃO DE DISPARO: Capturou exatamente 1 período completo (16.67ms)
                        if (sample_idx >= SAMPLES_PER_PERIOD) {
                            total_periodos_capturados++;
                            
                            //-------------------------------------------------------------
                            // Vrms
                            n = sizeof(periodo_senoide)/sizeof(periodo_senoide[0]);
                            aux = 0;
                            //float valor_real[SAMPLES_PER_PERIOD] = {0};
                            for(int i=0; i < n; i++)
                            {
                                valor_real[i] = ((periodo_senoide[i]/1000)-1.497)/0.007639;
                                aux = aux + pow(valor_real[i],2);
                            }
                            
                            vrms = sqrt(aux/n);

                            xStreamBufferSend(buffer_oled, &vrms, sizeof(vrms), 0);

                            // -------------------------------------------------------------
                            // ÁREA DE PROCESSAMENTO DO PERÍODO
                            // O seu vetor "periodo_senoide" está 100% preenchido aqui.
                            // Você pode calcular o RMS, achar o pico, etc.
                            // Exemplo: Detectar o Valor de Pico a Pico (Vpp) deste período:
                            int v_max = periodo_senoide[0];
                            int v_min = periodo_senoide[0];
                            for (int j = 0; j < SAMPLES_PER_PERIOD; j++) {
                                if (periodo_senoide[j] > v_max) v_max = periodo_senoide[j];
                                if (periodo_senoide[j] < v_min) v_min = periodo_senoide[j];
                            }
                            
                            // Logamos apenas uma vez a cada ciclo completo (Evita estourar o Watchdog)
                            ESP_LOGI(TAG, "[Ciclo %"PRIu32"] Período de 16.6ms Fechado! Pico-a-Pico: %d mV", 
                                     total_periodos_capturados, (v_max - v_min));
                            // -------------------------------------------------------------

                            // Zera o índice para sobrescrever e atualizar o vetor com o próximo período contíguo
                            sample_idx = 0;
                        }
                    }
                }
            }
        }
    }
}

// Task para exibir informações no display
void display(void *pvParameters)
{
    // Display max 17 char, sem char especial
    float dados;
    char str[7];
    char str2[7];
    int blackouts = 0;
    ESP_LOGI("Oled","Init...");
    while(1)
    {
        char form[20] = "TensaoRMS:"; // Precisa ser reatualuizado a cada volta do loop
        // Recebe os dados do buffer
        xStreamBufferReceive(buffer_oled, &dados, sizeof(dados), portMAX_DELAY);
        //ESP_LOGI("Oled", "printing");

        // Limpa o display
        //ssd1306_clear_screen(&device, false);
        // Converte o valor de 'dados' para uma string 'str'
        snprintf(str, sizeof(str), "%.2f", dados);
        // Concatena 'str' com uma string de formatação 'form'
        strcat(form, str);      
        // printa o valor de 'form' + 'dados'  
        //ssd1306_display_text(&device, 0, form, sizeof(form), false);

        // Verifica a ocorrência de blackouts em 16.67ms
        if(dados <= 66)
        {
            blackouts++;            
        }

        char form2[20] = "Quedas:";
         snprintf(str2, sizeof(str2), "%d", blackouts);
         strcat(form2, str2);      
        // printa o valor de 'form2' + 'blackouts'  
        ssd1306_display_text(&device, 1, form2, sizeof(form2), false);

         printf("Val RMS: %s, quedas: %s \n", form, form2);
    }
    
}

void app_main(void)
{
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    // Buffer dimensionado para segurar confortavelmente múltiplos frames brutos
    buffer = xStreamBufferCreate(4096, EXAMPLE_READ_LEN);
    buffer_oled = xStreamBufferCreate(4,1); 
    
    xTaskCreate(vtaskADC, "Task ADC", 4096, NULL, 5, NULL);
    xTaskCreate(vtaskProcessamento, "PROC", 2*4096, NULL, 5, NULL);
    xTaskCreate(display, "oled", 4096, NULL, 2, NULL);
    // Inicialização do display
    //display_init();
}
