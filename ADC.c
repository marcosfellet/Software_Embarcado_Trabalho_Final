/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#include "esp_timer.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1 // define a unidade de ADC como EXAMPLE_ADC_UNIT
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1 // será usado somente 1 unidade para realizar a conversão

#if SOC_ADC_ATTEN_NUM <= 1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#else
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_12 // atenuação de 12(~11) dB equivale a um ganho de aprox. 0,282.
#endif
// SOC é uma "biblioteca" que passa as informações de hardware do ESP que estamos usando.
// O SOC_ADC_ATTEN_NUM diz o número de atenuações possíveis daquele esp32. Alguns só podem operar com um fundo de escala de 1,1 V. 
// O modelo DEVKIT suporta até 12 db de atenuação.
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH //especifica a resolução do modelo de esp usado. no nosso caso é 12.
#define EXAMPLE_READ_LEN                    256 // tamanho do frame em bytes.

// o frame de 256 bytes é o bloco que o DMA preenche antes de disparar o callback "on_conv_done". 
// Com amostras de 2 bytes cada, o frame então possui 128 amostras.
// Dos 2 Bytes (16 Bits) 12 bits são para a conversão e 4 bits são o número do canal.

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6}; // no nosso caso como CONFIG_IDF_TARGET_ESP32 é igual a um, teremos um canal e esse canal será o 6.
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";


static TaskHandle_t procTaskHandle = NULL;
#define MAX_SAMPLES 2048
StreamBufferHandle_t buffer;


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
        adc_continuous_handle_t handle = NULL;

        adc_continuous_handle_cfg_t adc_config = {
            .max_store_buf_size = 1024,
            .conv_frame_size = EXAMPLE_READ_LEN,
        };
        ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t pattern[1];

    pattern[0].atten = ADC_ATTEN_DB_12;
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
    //out_handle é handle de fora, neste caso, criada e passa pela main.
    // o valor apontado pelo out_handle vai receber a handle (que é a handle do ADC).
}

void vtaskADC(void *pvparameters)
{
    ESP_LOGI("ADC", "Task Inicializando!");
    adc_cali_handle_t cali_handle = NULL; // criei o espaço onde terá as configurações da calibração
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    esp_err_t  ret_cali = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);

    //if (ret_cali == ESP_OK) {
    //    ESP_LOGI(TAG, "Calibração criada com sucesso");
    //} else {
    //    ESP_LOGW(TAG, "Calibração não disponível: %s", esp_err_to_name(ret_cali));
    //}

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0}; //vetor de zeros com tamanho da quantidade de bytes que temos no frame
    //memset(result, 0xcc, EXAMPLE_READ_LEN); // preenchendo os 256 bytes do result com 1100 1100 (auxilia depuração)

    s_task_handle = xTaskGetCurrentTaskHandle();
    // s_task_handle vai receber o handle da main, visto que usamos xTaskGetCurrentTaskHandle() dentro da tarefa ADC. 
    // Isso é importante visto que para usar o notify vamos ter que passar o handle da task que queremos notificar
    // além disso, s_task_handle é global.

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); //criando e configurando o ADC através da função criada.

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    }; //callback que sinaliza quando o frame é preenchido.

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    while(1)
{
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const int64_t tempo_antes = esp_timer_get_time();
    const int64_t duracao_us = 86000;

    while(esp_timer_get_time() - tempo_antes < duracao_us){

    ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, pdMS_TO_TICKS(5));

    if (ret == ESP_OK)
{
    // ESP_LOGI(TAG, "Recebi %"PRIu32" bytes", ret_num);
    // declara o array com tamanho suficiente para todas as amostras do frame
    static adc_continuous_data_t parsed_data[EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES];
    // ou seja, o parsed_data é um vetor com 126 posições. 126 posições visto que é o número de amostras obtidas.
    uint32_t num_parsed_samples = 0;
    //result contém os dados brutos (256 bytes), o parse contém os dados já tratados.
    // chama a função que faz o parse: transforma result[] em parsed_data[]
    esp_err_t parse_ret = adc_continuous_parse_data(
        handle,           // o handle do driver
        result,           // buffer bruto de entrada
        ret_num,          // quantos bytes vieram
        parsed_data,      // array de saída já estruturado
        &num_parsed_samples // quantas amostras foram extraídas
    );

    // 3. agora você percorre as amostras já legíveis
    if (parse_ret == ESP_OK) {
        for (int i = 0; i < num_parsed_samples; i++) {
            if (parsed_data[i].valid) {
                //int tensao_mv = 0;
                //adc_cali_raw_to_voltage(cali_handle, parsed_data[i].raw_data, &tensao_mv);

                //ESP_LOGI(TAG, "Canal: %d | Raw: %"PRIu32" | Tensão: %d mV",
                //parsed_data[i].channel,
                //parsed_data[i].raw_data,
                //tensao_mv);
                xStreamBufferSend(buffer, &parsed_data[i].raw_data, sizeof(parsed_data[i].raw_data), 0);
            }
            
        }
    }
    else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
}   
}
    
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    xTaskNotifyGive(procTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vtaskProcessamento(void *pvparameters)
{
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };

    esp_err_t  ret_cali = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);

    int tensao_mv = 0;
    while (1)
    {
        uint32_t total = 0;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // drena tudo que está no stream buffer
        while (xStreamBufferReceive(buffer, &tensao_mv, sizeof(tensao_mv), 0) == sizeof(tensao_mv))
        {
            //adc_cali_raw_to_voltage(cali_handle, parsed_data[i].raw_data, &tensao_mv);

            printf("%d\n", tensao_mv);
            total++;
        }

        if(total != 0){ESP_LOGI(TAG, "Total de amostras processadas: %"PRIu32, total);}
    }
}

void app_main(void)
{
    buffer = xStreamBufferCreate(MAX_SAMPLES*sizeof(int), sizeof(int));
    xTaskCreate(vtaskADC, "Task ADC", 8192, NULL, 5, NULL); // por eu não precisar controlar ela eu não preciso passar um handle para ela. ela roda em loop.
    xTaskCreate(vtaskProcessamento,"PROC", 4096, NULL, 4, &procTaskHandle); // vou precisar criar e referenciar com o buffer e o tasknotification, portanto, estou passando o handle da task.
}
