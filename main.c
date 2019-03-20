/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "i2s.h"
#include "sysctl.h"
#include "fpioa.h"
#include "uarths.h"
#include "dmac.h"
#include <math.h>
#include "fft.h"
#include "st7789.h"
#include "lcd.h"
#include "board_config.h"

uint32_t rx_buf[1024];
uint32_t g_index;
uint32_t g_tx_len;

#define FRAME_LEN           512
#define FFT_N               512U
#define FFT_FORWARD_SHIFT   0x0U
#define SAMPLE_RATE         16000
#define SPECTROGRAM_LENGTH LCD_X_MAX
#define SPECTROGRAM_HEIGHT FFT_N/2


#define MIN_DB -30
#define MAX_DB  20

/*static uint16_t gray2rgb565[64]={
0x0000, 0x2000, 0x4108, 0x6108, 0x8210, 0xa210, 0xc318, 0xe318, 
0x0421, 0x2421, 0x4529, 0x6529, 0x8631, 0xa631, 0xc739, 0xe739, 
0x0842, 0x2842, 0x494a, 0x694a, 0x8a52, 0xaa52, 0xcb5a, 0xeb5a, 
0x0c63, 0x2c63, 0x4d6b, 0x6d6b, 0x8e73, 0xae73, 0xcf7b, 0xef7b, 
0x1084, 0x3084, 0x518c, 0x718c, 0x9294, 0xb294, 0xd39c, 0xf39c, 
0x14a5, 0x34a5, 0x55ad, 0x75ad, 0x96b5, 0xb6b5, 0xd7bd, 0xf7bd, 
0x18c6, 0x38c6, 0x59ce, 0x79ce, 0x9ad6, 0xbad6, 0xdbde, 0xfbde, 
0x1ce7, 0x3ce7, 0x5def, 0x7def, 0x9ef7, 0xbef7, 0xdfff, 0xffff,
};*/



//uint16_t g_lcd_gram[LCD_Y_MAX*LCD_X_MAX] __attribute__((aligned(64)));
//uint16_t g_lcd_gram_old[LCD_Y_MAX*LCD_X_MAX] __attribute__((aligned(64)));
uint16_t g_lcd_gram[SPECTROGRAM_LENGTH*SPECTROGRAM_HEIGHT] __attribute__((aligned(64)));
uint16_t g_lcd_gram_old[SPECTROGRAM_LENGTH*SPECTROGRAM_HEIGHT] __attribute__((aligned(64)));
//float hard_power_spectrogram[SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH];
//float hard_power_spectrogram_disp[SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH];
int16_t  real[FFT_N];
int16_t  imag[FFT_N];
float    hard_power[FFT_N];
uint64_t fft_out_data[FFT_N / 2];
uint64_t buffer_input[FFT_N];
uint64_t buffer_output[FFT_N];


uint16_t hann[FFT_N];


uint16_t get_bit1_num(uint32_t data)
{
    uint16_t num;
    for (num = 0; data; num++)
        data &= data - 1;
    return num;
}

void io_mux_init()
{

    fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);
    fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(36, FUNC_SPI0_SS3);
    fpioa_set_function(39, FUNC_SPI0_SCLK);

    fpioa_set_function(20, FUNC_I2S0_IN_D0);
    fpioa_set_function(19, FUNC_I2S0_WS);
    fpioa_set_function(18, FUNC_I2S0_SCLK);

    sysctl_set_spi0_dvp_data(1);
}


static void io_set_power(void)
{
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK3, SYSCTL_POWER_V18);
}

uint16_t color_scale(float value)
{
  uint16_t color_code ;
  if(value > 1) value = 1 ;
  if(value < 0) value = 0 ;
  int aR = 0;   int aG = 0; int aB=31;  // RGB for our 1st color (blue in this case).
  int bR = 31; int bG = 0; int bB=0;    // RGB for our 2nd color (red in this case).

  float red   = (float)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
  float green = (float)(bG - aG) * value + aG;      // Evaluates as 0.
  float blue  = (float)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.

  uint16_t rs = ((uint16_t) red)  ; //5bits only
  uint16_t gs = ((uint16_t) green)  ; //6bits
  uint16_t bs = ((uint16_t) blue)  ;
  color_code = (rs << 11) | (gs << 5)| bs ; 
  return color_code ;
}




void hann_init(){
	for (int i = 0; i < FFT_N; i++) {
    		float multiplier = 0.5f * (1.f - cos(2.f*M_PI*i/(FFT_N))) * 65535.f;// FP Q1.16
		hann[i] = floor(multiplier) ;
	}
}

void update_image( float* hard_power,  float  pw_max, float  pw_min, uint16_t* pImage);
uint16_t dbToColor(float db, float max, float min);
int main(void)
{
    int i;
    float pmax = 0 , pmin = 0;

    complex_hard_t data_hard[FFT_N] = {0};
    fft_data_t *output_data;
    fft_data_t *input_data;


    sysctl_pll_set_freq(SYSCTL_PLL0, 480000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL1, 160000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);

    uarths_init();
    io_mux_init();
    io_set_power();

    lcd_init(20000000);
    lcd_clear(WHITE);

    i2s_init(I2S_DEVICE_0, I2S_RECEIVER, 0x3);

    i2s_set_sample_rate(I2S_DEVICE_0, SAMPLE_RATE );

    i2s_rx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
                          RESOLUTION_16_BIT, SCLK_CYCLES_32,
                          TRIGGER_LEVEL_4, STANDARD_MODE);

    hann_init();
    pmax = MAX_DB ;
    pmin = MIN_DB ;
    lcd_draw_string(SPECTROGRAM_HEIGHT + 10 , 120, "STFT", BLACK);
    while (1)
    {

	//Channels layout is stereo 32bit aligned
	memcpy(rx_buf, &rx_buf[FRAME_LEN], FRAME_LEN*sizeof(uint32_t));
        i2s_receive_data_dma(I2S_DEVICE_0, &rx_buf[FRAME_LEN], FRAME_LEN, DMAC_CHANNEL3);//Only getting the overlap of FFT_N/2

        for ( i = 0; i < FFT_N / 2; ++i)
        {
	    input_data = (fft_data_t *)&buffer_input[i];
            uint32_t v = ((uint32_t) rx_buf[2*i]) * ((uint32_t) hann[(i*2)]);
	    v = v >> 16 ;
            input_data->R1 = v ;
	    //input_data->R1 = rx_buf[2*i];   // data_hard[2 * i].real;
            input_data->I1 = 0;             // data_hard[2 * i].imag;
            v = ((uint32_t) rx_buf[(2*i)+1]) * ((uint32_t) hann[(i*2)+1]);
            v = v >> 16 ;
	    input_data->R2 = v;
	    //input_data->R2 = rx_buf[2*i+1]; // data_hard[2 * i + 1].real;
            input_data->I2 = 0;             // data_hard[2 * i + 1].imag;
        }

        fft_complex_uint16_dma(DMAC_CHANNEL0, DMAC_CHANNEL1, FFT_FORWARD_SHIFT, FFT_DIR_FORWARD, buffer_input, FFT_N, buffer_output);

        for ( i = 0; i < SPECTROGRAM_HEIGHT / 2; i++)
        {
            output_data = (fft_data_t*)&buffer_output[i];
            data_hard[2 * i].imag = output_data->I1  ;
            data_hard[2 * i].real = output_data->R1  ;
            data_hard[2 * i + 1].imag = output_data->I2  ;
            data_hard[2 * i + 1].real = output_data->R2  ;
	}


	/*memcpy(&hard_power_spectrogram_disp[SPECTROGRAM_HEIGHT], hard_power_spectrogram, sizeof(float)*(SPECTROGRAM_HEIGHT*(SPECTROGRAM_LENGTH-1)));
        for (i = BIN_START; i < (SPECTROGRAM_HEIGHT+BIN_START); i++)//Only interested in one size of the FFT
        {
            float pow = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);
            pow = 2*pow/FFT_N  ;
	    pow = 20*log(pow) ;
	    if(pow > pmax) pmax = pow ;
	    hard_power_spectrogram_disp[i-BIN_START] = pow ;
        }
	memcpy(hard_power_spectrogram, hard_power_spectrogram_disp, sizeof(float)*(SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH));
	update_image(hard_power_spectrogram_disp, pmax, pmin, g_lcd_gram);*/
	
	//Trying to update buffer instead of doing the full buffer copy
	//memcpy(&g_lcd_gram[LCD_Y_MAX], g_lcd_gram_old, (LCD_Y_MAX*(LCD_X_MAX-1))*sizeof(uint16_t));
	memcpy(&g_lcd_gram[SPECTROGRAM_HEIGHT], g_lcd_gram_old, (SPECTROGRAM_HEIGHT*(SPECTROGRAM_LENGTH-1))*sizeof(uint16_t));
	for (i = 0; i < SPECTROGRAM_HEIGHT; i++)//Only interested in one size of the FFT
        {
            float pow = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);
            pow = 2*pow/FFT_N  ;
            pow = 20*log(pow) ;
            uint16_t c = dbToColor(pow, pmax, pmin);
            g_lcd_gram[i] = c ;
        }
	//memcpy(&g_lcd_gram_old, g_lcd_gram, (LCD_Y_MAX*LCD_X_MAX)*sizeof(uint16_t));
        memcpy(&g_lcd_gram_old, g_lcd_gram, (SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH)*sizeof(uint16_t));
	//lcd_draw_picture(0, 0, LCD_Y_MAX, LCD_X_MAX, (uint32_t*) g_lcd_gram);
	lcd_draw_picture(0, 0, SPECTROGRAM_HEIGHT, SPECTROGRAM_LENGTH, (uint32_t*) g_lcd_gram);
	//lcd_draw_string(SPECTROGRAM_HEIGHT + 10 , 120, "STFT", BLACK);
    }

    return 0;
}


uint16_t dbToColor(float db, float max, float min){
	db = (db - min)/(max - min);
	return color_scale(db);
}

void update_image( float* hard_power, float pw_max, float pw_min, uint16_t* pImage)
{

    int i, j;
    for(j = 0 ; j < SPECTROGRAM_LENGTH ; j ++){
      for(i = 0 ; i < SPECTROGRAM_HEIGHT ; i ++){
	float v = (hard_power[i+(j*SPECTROGRAM_HEIGHT)] - pw_min)/(pw_max - pw_min);
	pImage[i+(j*LCD_Y_MAX)] = color_scale(v);
      }
    }
}

