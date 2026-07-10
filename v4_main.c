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

// Biblioteca da comunicação
#include "comunicacao.h"

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

// 333.33 amostras por período
#define SAMPLES_PER_PERIOD              333 

#define LED1 GPIO_NUM_23

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6}; 
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

//static const char *TAG = "SENOIDE_60HZ";
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

void vtaskProcessamento(void *pv)
{
    uint8_t raw_frame[EXAMPLE_READ_LEN]; 
    static adc_continuous_data_t parsed_data[EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES];
    char blackout = 0;
    char surto = 0;
    int periodo_senoide[SAMPLES_PER_PERIOD] = {0};
    float valor_real[SAMPLES_PER_PERIOD] = {0};
    int sample_idx = 0;

    float vrms = 0;
    float aux = 0;
    int n = 0;

    static float ultima_amostra = 0.0f;
    static uint32_t contador_amostras = 0;
    static float frequencia_estimada = 0.0f;

    while (1)
    {
        size_t bytes_recebidos = xStreamBufferReceive(buffer, raw_frame, EXAMPLE_READ_LEN, portMAX_DELAY);
        if (bytes_recebidos > 0) {
            uint32_t num_parsed_samples = 0;
            esp_err_t parse_ret = adc_continuous_parse_data(adc_handle, raw_frame, bytes_recebidos, parsed_data, &num_parsed_samples);

            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (parsed_data[i].valid) {
                        int tensao_mv = 0;
                        adc_cali_raw_to_voltage(cali_handle, parsed_data[i].raw_data, &tensao_mv);

                        float amostra_atual = ((tensao_mv / 1000.0f) - 1.497f) / 0.007639f;
                        contador_amostras++;

                        if (ultima_amostra <= 0.0f && amostra_atual > 0.0f && contador_amostras > 250) {
                            frequencia_estimada = 20000.0f / (float)contador_amostras;
                            // SINALIZADOR APAGADO: printf removido para não sujar o buffer binário
                            contador_amostras = 0;
                        }

                        ultima_amostra = amostra_atual;
                        periodo_senoide[sample_idx] = tensao_mv;
                        sample_idx++;

                        // Capturou 1 período completo (16.67ms)
                        if (sample_idx >= SAMPLES_PER_PERIOD) {
                            n = sizeof(periodo_senoide)/sizeof(periodo_senoide[0]);
                            aux = 0;
                            
                            // 1. Calcula os valores reais e acumula para o RMS
                            for(int j=0; j < n; j++)
                            {
                                valor_real[j] = ((periodo_senoide[j]/1000.0f) - 1.497f) / 0.007639f;
                                aux = aux + pow(valor_real[j], 2);
                            }
                            
                            // 2. Calcula o RMS apenas para atualizar os flags de alerta do pacote
                            vrms = sqrt(aux / n);
                            blackout = (vrms < 50.0f) ? 1 : 0;
                            surto    = (vrms > 245.0f) ? 1 : 0;
                            
                            // Envia o RMS para a fila do display local (Oled)
                            xStreamBufferSend(buffer_oled, &vrms, sizeof(vrms), 0);

                            // 3. ENVIA OS VALORES INSTANTÂNEOS SUBAMOSTRADOS (5000 Hz) PARA O PYTHON
                            // Enviamos a cada 4 amostras para não estourar a banda da serial e manter a fluidez
                            for(int j=0; j < n; j += 4)
                            {
                                comunicacao_serial(&valor_real[j], &blackout, &surto);
                            }

                            sample_idx = 0;
                        }
                    }
                }
            }
        }
    }
}


//Task para exibir informações no display
void display(void *pvParameters)
{
    // Display max 17 char, sem char especial
    float dados;
    char str[7];
    char str2[7];
    bool blackout = 0;
    //ESP_LOGI("Oled","Init...");
    while(1)
    {
        char form[20] = "TensaoRMS:"; // Precisa ser reatualuizado a cada volta do loop
        // Recebe os dados do buffer
        xStreamBufferReceive(buffer_oled, &dados, sizeof(dados), portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
        //ESP_LOGI("Oled", "printing");

        // Limpa o display
        ssd1306_clear_screen(&device, false);
        // Converte o valor de 'dados' para uma string 'str'
        snprintf(str, sizeof(str), "%.2f", dados);
        // Concatena 'str' com uma string de formatação 'form'
        strcat(form, str);      
        // printa o valor de 'form' + 'dados'  
        ssd1306_display_text(&device, 0, form, sizeof(form), false);

        // Verifica a ocorrência de blackouts em 16.67ms
        if(dados <= 66)
        {
            blackout = 1;
            gpio_set_level(LED1,0);
            
        }
        else
        {
            blackout = 0;
            gpio_set_level(LED1,1);
        }
        
        char form2[20] = "Quedas:";
        snprintf(str2, sizeof(str2), "%d", blackout);
        strcat(form2, str2);      
        // printa o valor de 'form2' + 'blackouts'
        ssd1306_display_text(&device, 1, form2, sizeof(form2), false);

        printf("%s, %s \n", form, form2);
    }
    
}

void app_main(void)
{

    gpio_reset_pin(LED1);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);


    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    esp_log_level_set("*", ESP_LOG_NONE);
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
    // Iniciando a comunicação UART
    uart_init();
    // Buffer dimensionado para segurar confortavelmente múltiplos frames brutos
    buffer = xStreamBufferCreate(4096, EXAMPLE_READ_LEN);
    buffer_oled = xStreamBufferCreate(4,1); 
    
    xTaskCreate(vtaskADC, "Task ADC", 4096, NULL, 5, NULL);
    xTaskCreate(vtaskProcessamento, "PROC", 2*4096, NULL, 5, NULL);
    xTaskCreate(display, "oled", 4096, NULL, 2, NULL);
    // Inicialização do display
    display_init();
}
