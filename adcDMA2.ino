#include <driver/adc.h>
#include <esp_adc_cal.h>
#define TIMES              200
#define GET_UNIT(x)        ((x>>3) & 0x1)
#define ADC_RESULT_BYTE                 2
#define ADC_CONV_LIMIT_EN               1                       //For ESP32, this should always be set to 1
#define ADC_OUTPUT_TYPE                 ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_CONV_MODE                   ADC_CONV_SINGLE_UNIT_1
#define NUM_SAMPLES 200
static uint16_t adc1_chan_mask = BIT(4) | BIT(5) | BIT(6) | BIT(7);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[4] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7}; // GPIO33
#define DEFAULT_VREF 1100              // Use a tensão de referência padrão em mV
uint8_t result[TIMES] = {0};            // Buffer to store raw readings
uint32_t sum_channel_32, sum_channel_33;
uint32_t max_channel_32, max_channel_33;
uint32_t min_channel_32, min_channel_33;

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num) {
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 200,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    adc_digi_initialize(&adc_dma_config);

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 200,
        .sample_freq_hz = 24 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

   
   adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
   dig_cfg.pattern_num = channel_num;
   for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    adc_digi_controller_configure(&dig_cfg);
}

void read_adc_data() {
    uint32_t Conta=0, ret_num = 0;
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
     
    adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    sum_channel_32 = 0; max_channel_32 = 0; min_channel_32 = 4096 << 6;
    sum_channel_33 = 0; max_channel_33 = 0; min_channel_33 = 4096 << 6;

    for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
        int value = (esp_adc_cal_raw_to_voltage(p->type1.data, adc_chars)) << 6; // Multiplica por 64 do resistor-TC para passar para Ampere
        if (p->type1.channel == ADC_CHANNEL_4) {
            sum_channel_32 += value;
            Conta++;
            if (value > max_channel_32) max_channel_32 = value;
            if (value < min_channel_32) min_channel_32 = value;
        } else if (p->type1.channel == ADC_CHANNEL_5) {
            sum_channel_33 += value;
            if (value > max_channel_33) max_channel_33 = value;
            if (value < min_channel_33) min_channel_33 = value;
        }
    }
   sum_channel_32=sum_channel_32/Conta;
   sum_channel_33=sum_channel_33/Conta; 
   free(adc_chars);
}

void setup() {
    Serial.begin(115200);
    memset(result, 0xcc, TIMES);
    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    adc_digi_start();
}

void loop() {
    delay(1000);
    read_adc_data();
    printf("Channel 32 - m: %d, Max: %d, Min: %d\n", sum_channel_32>>6, max_channel_32>>6, min_channel_32>>6);
    printf("Channel 33 - m: %d, Max: %d, Min: %d\n", sum_channel_33>>6, max_channel_33>>6, min_channel_33>>6);
  //  int botao = analogRead(35);
  //  Serial.print(botao);
  //  adc_digi_init();
}
