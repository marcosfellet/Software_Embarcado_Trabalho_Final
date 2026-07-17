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

#include "comunicacao.h"
#include "ssd1306.h"
#include "driver/i2c.h"

SSD1306_t device;

// 1. Criamos uma estrutura para enviar os dados juntos para o display
typedef struct {
    float vrms;
    float frequencia;
} dados_display_t;

StreamBufferHandle_t buffer;
StreamBufferHandle_t buffer_display; // Um único buffer para o display

void display_init(void) {
    i2c_master_init(&device, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&device, 128, 64);
    ssd1306_clear_screen(&device, false);
    ssd1306_contrast(&device, 0xff);
}

#define EXAMPLE_ADC_UNIT                 ADC_UNIT_1 
#define EXAMPLE_ADC_CONV_MODE            ADC_CONV_SINGLE_UNIT_1 
#define EXAMPLE_ADC_ATTEN                ADC_ATTEN_DB_12 
#define EXAMPLE_ADC_BIT_WIDTH            SOC_ADC_DIGI_MAX_BITWIDTH 
#define EXAMPLE_READ_LEN                 256 
#define SAMPLES_PER_PERIOD               333 
#define LED1                             GPIO_NUM_23

static adc_channel_t channel[1] = {ADC_CHANNEL_6}; 
static const char *TAG = "SENOIDE";
static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {
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

void vtaskADC(void *pvparameters) {
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0}; 

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &adc_handle); 
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while(1) {
        ret = adc_continuous_read(adc_handle, result, EXAMPLE_READ_LEN, &ret_num, portMAX_DELAY);
        if (ret == ESP_OK && ret_num > 0) {
            xStreamBufferSend(buffer, result, ret_num, pdMS_TO_TICKS(10));
        }
    }
}

void vtaskProcessamento(void *pv) {
    uint8_t raw_frame[EXAMPLE_READ_LEN]; 
    static adc_continuous_data_t parsed_data[EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES];
    char blackout = 0;
    char surto = 0;
    int periodo_senoide[SAMPLES_PER_PERIOD] = {0};
    float valor_real[SAMPLES_PER_PERIOD] = {0};
    int sample_idx = 0;

    float vrms = 0;
    float aux = 0;
    
    static float ultima_amostra = 0.0f;
    static uint32_t contador_amostras = 0;
    static float frequencia_estimada = 60.0f; // Valor padrão inicial

    while (1) {
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
                        
                        // Zero-Crossing
                        if (ultima_amostra <= 0.0f && amostra_atual > 0.0f && contador_amostras > 250) {
                            frequencia_estimada = 20000.0f / (float)contador_amostras;
                            contador_amostras = 0;
                        }
                        
                        ultima_amostra = amostra_atual;
                        periodo_senoide[sample_idx] = tensao_mv;
                        sample_idx++;

                        if (sample_idx >= SAMPLES_PER_PERIOD) {
                            aux = 0;
                            
                            // Correção: Multiplicação direta em vez de usar pow()
                            for(int j=0; j < SAMPLES_PER_PERIOD; j++) {
                                valor_real[j] = ((periodo_senoide[j]/1000.0f) - 1.497f) / 0.007639f;
                                aux += (valor_real[j] * valor_real[j]); 
                            }
                            
                            vrms = sqrt(aux / SAMPLES_PER_PERIOD);
                            
                            // Envia o pacote completo de dados estruturados para o Display
                            dados_display_t dados_envio = { .vrms = vrms, .frequencia = frequencia_estimada };
                            xStreamBufferSend(buffer_display, &dados_envio, sizeof(dados_display_t), 0);

                            // Envia para o Python
                            for(int j=0; j < SAMPLES_PER_PERIOD; j += 4) {
                                blackout = (vrms <= 66) ? 1 : 0;
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

void display(void *pvParameters) {
    dados_display_t dados_recebidos;
    char str[7];
    char str2[7];
    
    while(1) {
        char form[20] = "TensaoRMS:";
        char form2[20] = "Frequencia:";

        // Bloqueia de forma segura em um único buffer combinado
        xStreamBufferReceive(buffer_display, &dados_recebidos, sizeof(dados_display_t), portMAX_DELAY);
        
        // LOG funcional para debug no terminal do VS Code
        //ESP_LOGI(TAG, "RMS: %.2f V | Freq: %.2f Hz", dados_recebidos.vrms, dados_recebidos.frequencia);

        // Atualização da tela física
        ssd1306_clear_screen(&device, false);
        snprintf(str, sizeof(str), "%.2f", dados_recebidos.vrms);
        snprintf(str2, sizeof(str2), "%.2f", dados_recebidos.frequencia);
        strcat(form, str);
        strcat(form2, str2);
        
        ssd1306_display_text(&device, 0, form, sizeof(form), false);
        ssd1306_display_text(&device, 1, form2, sizeof(form2), false);
        
        if(dados_recebidos.vrms <= 66) {
            gpio_set_level(LED1, 0);
        } else {
            gpio_set_level(LED1, 1);
        }
    }
}

void app_main(void) {
    gpio_reset_pin(LED1);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
    
    uart_init();
    
    buffer = xStreamBufferCreate(4096, EXAMPLE_READ_LEN);
    // Espaço para até 5 estruturas completas de dados do display (5 * 8 bytes = 40)
    buffer_display = xStreamBufferCreate(40, sizeof(dados_display_t)); 
    
    xTaskCreate(vtaskADC, "Task ADC", 4096, NULL, 5, NULL);
    xTaskCreate(vtaskProcessamento, "PROC", 2*4096, NULL, 5, NULL);
    xTaskCreate(display, "oled", 4096, NULL, 2, NULL);
    
    display_init();
}
