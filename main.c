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

int32_t rx_buf[1024];
//int16_t rx_buf[1024];
uint32_t g_index;
uint32_t g_tx_len;

#define FFT_N               512U
#define FRAME_LEN           FFT_N
#define FFT_FORWARD_SHIFT   0x0U
#define SAMPLE_RATE         23040
#define SPECTROGRAM_LENGTH (LCD_X_MAX-20)
#define SPECTROGRAM_HEIGHT FFT_N/2//LCD_Y_MAX//FFT_N //(FFT_N/2 + 50)


#define MIN_DB -50
#define MAX_DB  50

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


#define COLOR_LUT_SIZE 1024
static const uint16_t jetMap[COLOR_LUT_SIZE]={
0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011,
0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013,
0x0013, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015,
0x0015, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017,
0x0017, 0x0017, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019,
0x0019, 0x0019, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B,
0x001B, 0x001B, 0x001B, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001D, 0x001D, 0x001D, 0x001D, 0x001D,
0x001D, 0x001D, 0x001D, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
0x001F, 0x001F, 0x003F, 0x003F, 0x003F, 0x003F, 0x005F, 0x005F, 0x005F, 0x005F, 0x007F, 0x007F, 0x007F, 0x007F, 0x009F, 0x009F,
0x009F, 0x009F, 0x00BF, 0x00BF, 0x00BF, 0x00BF, 0x00DF, 0x00DF, 0x00DF, 0x00DF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x011F, 0x011F,
0x011F, 0x011F, 0x013F, 0x013F, 0x013F, 0x013F, 0x015F, 0x015F, 0x015F, 0x015F, 0x017F, 0x017F, 0x017F, 0x017F, 0x019F, 0x019F,
0x019F, 0x019F, 0x01BF, 0x01BF, 0x01BF, 0x01BF, 0x01DF, 0x01DF, 0x01DF, 0x01DF, 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x021F, 0x021F,
0x021F, 0x021F, 0x021F, 0x023F, 0x023F, 0x023F, 0x023F, 0x025F, 0x025F, 0x025F, 0x025F, 0x027F, 0x027F, 0x027F, 0x027F, 0x029F,
0x029F, 0x029F, 0x029F, 0x02BF, 0x02BF, 0x02BF, 0x02BF, 0x02DF, 0x02DF, 0x02DF, 0x02DF, 0x02FF, 0x02FF, 0x02FF, 0x02FF, 0x031F,
0x031F, 0x031F, 0x031F, 0x033F, 0x033F, 0x033F, 0x033F, 0x035F, 0x035F, 0x035F, 0x035F, 0x037F, 0x037F, 0x037F, 0x037F, 0x039F,
0x039F, 0x039F, 0x039F, 0x03BF, 0x03BF, 0x03BF, 0x03BF, 0x03DF, 0x03DF, 0x03DF, 0x03DF, 0x03FF, 0x03FF, 0x03FF, 0x03FF, 0x041F,
0x041F, 0x041F, 0x041F, 0x041F, 0x043F, 0x043F, 0x043F, 0x043F, 0x045F, 0x045F, 0x045F, 0x045F, 0x047F, 0x047F, 0x047F, 0x047F,
0x049F, 0x049F, 0x049F, 0x049F, 0x04BF, 0x04BF, 0x04BF, 0x04BF, 0x04DF, 0x04DF, 0x04DF, 0x04DF, 0x04FF, 0x04FF, 0x04FF, 0x04FF,
0x051F, 0x051F, 0x051F, 0x051F, 0x053F, 0x053F, 0x053F, 0x053F, 0x055F, 0x055F, 0x055F, 0x055F, 0x057F, 0x057F, 0x057F, 0x057F,
0x059F, 0x059F, 0x059F, 0x059F, 0x05BF, 0x05BF, 0x05BF, 0x05BF, 0x05DF, 0x05DF, 0x05DF, 0x05DF, 0x05FF, 0x05FF, 0x05FF, 0x05FF,
0x05FF, 0x061F, 0x061F, 0x061F, 0x061F, 0x063F, 0x063F, 0x063F, 0x063F, 0x065F, 0x065F, 0x065F, 0x065F, 0x067F, 0x067F, 0x067F,
0x067F, 0x069F, 0x069F, 0x069F, 0x069F, 0x06BF, 0x06BF, 0x06BF, 0x06BF, 0x06DF, 0x06DF, 0x06DF, 0x06DF, 0x06FF, 0x06FF, 0x06FF,
0x06FF, 0x071F, 0x071F, 0x071F, 0x071F, 0x073F, 0x073F, 0x073F, 0x073F, 0x075F, 0x075F, 0x075F, 0x075F, 0x077F, 0x077F, 0x077F,
0x077F, 0x079F, 0x079F, 0x079F, 0x079F, 0x07BF, 0x07BF, 0x07BF, 0x07BF, 0x07DF, 0x07DF, 0x07DF, 0x07DF, 0x07FF, 0x07FF, 0x07FF,
0x07FF, 0x07FF, 0x07FF, 0x07FF, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x17FD, 0x17FD, 0x17FD, 0x17FD,
0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x27FB, 0x27FB, 0x27FB, 0x27FB,
0x27FB, 0x27FB, 0x27FB, 0x27FB, 0x27FB, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x37F9, 0x37F9, 0x37F9,
0x37F9, 0x37F9, 0x37F9, 0x37F9, 0x37F9, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x47F7, 0x47F7, 0x47F7,
0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x57F5, 0x57F5,
0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x67F3, 0x67F3,
0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x77F1,
0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x87F0,
0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE,
0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC,
0x9FEC, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA,
0xAFEA, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8,
0xBFE8, 0xBFE8, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6,
0xCFE6, 0xCFE6, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4,
0xDFE4, 0xDFE4, 0xDFE4, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xEFE2, 0xEFE2, 0xEFE2, 0xEFE2, 0xEFE2,
0xEFE2, 0xEFE2, 0xEFE2, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0,
0xFFE0, 0xFFE0, 0xFFC0, 0xFFC0, 0xFFC0, 0xFFC0, 0xFFA0, 0xFFA0, 0xFFA0, 0xFFA0, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF60, 0xFF60,
0xFF60, 0xFF60, 0xFF40, 0xFF40, 0xFF40, 0xFF40, 0xFF20, 0xFF20, 0xFF20, 0xFF20, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFEE0, 0xFEE0,
0xFEE0, 0xFEE0, 0xFEC0, 0xFEC0, 0xFEC0, 0xFEC0, 0xFEA0, 0xFEA0, 0xFEA0, 0xFEA0, 0xFE80, 0xFE80, 0xFE80, 0xFE80, 0xFE60, 0xFE60,
0xFE60, 0xFE60, 0xFE40, 0xFE40, 0xFE40, 0xFE40, 0xFE20, 0xFE20, 0xFE20, 0xFE20, 0xFE00, 0xFE00, 0xFE00, 0xFE00, 0xFDE0, 0xFDE0,
0xFDE0, 0xFDE0, 0xFDE0, 0xFDC0, 0xFDC0, 0xFDC0, 0xFDC0, 0xFDA0, 0xFDA0, 0xFDA0, 0xFDA0, 0xFD80, 0xFD80, 0xFD80, 0xFD80, 0xFD60,
0xFD60, 0xFD60, 0xFD60, 0xFD40, 0xFD40, 0xFD40, 0xFD40, 0xFD20, 0xFD20, 0xFD20, 0xFD20, 0xFD00, 0xFD00, 0xFD00, 0xFD00, 0xFCE0,
0xFCE0, 0xFCE0, 0xFCE0, 0xFCC0, 0xFCC0, 0xFCC0, 0xFCC0, 0xFCA0, 0xFCA0, 0xFCA0, 0xFCA0, 0xFC80, 0xFC80, 0xFC80, 0xFC80, 0xFC60,
0xFC60, 0xFC60, 0xFC60, 0xFC40, 0xFC40, 0xFC40, 0xFC40, 0xFC20, 0xFC20, 0xFC20, 0xFC20, 0xFC00, 0xFC00, 0xFC00, 0xFC00, 0xFC00,
0xFBE0, 0xFBE0, 0xFBE0, 0xFBE0, 0xFBC0, 0xFBC0, 0xFBC0, 0xFBC0, 0xFBA0, 0xFBA0, 0xFBA0, 0xFBA0, 0xFB80, 0xFB80, 0xFB80, 0xFB80,
0xFB60, 0xFB60, 0xFB60, 0xFB60, 0xFB40, 0xFB40, 0xFB40, 0xFB40, 0xFB20, 0xFB20, 0xFB20, 0xFB20, 0xFB00, 0xFB00, 0xFB00, 0xFB00,
0xFAE0, 0xFAE0, 0xFAE0, 0xFAE0, 0xFAC0, 0xFAC0, 0xFAC0, 0xFAC0, 0xFAA0, 0xFAA0, 0xFAA0, 0xFAA0, 0xFA80, 0xFA80, 0xFA80, 0xFA80,
0xFA60, 0xFA60, 0xFA60, 0xFA60, 0xFA40, 0xFA40, 0xFA40, 0xFA40, 0xFA20, 0xFA20, 0xFA20, 0xFA20, 0xFA00, 0xFA00, 0xFA00, 0xFA00,
0xFA00, 0xF9E0, 0xF9E0, 0xF9E0, 0xF9E0, 0xF9C0, 0xF9C0, 0xF9C0, 0xF9C0, 0xF9A0, 0xF9A0, 0xF9A0, 0xF9A0, 0xF980, 0xF980, 0xF980,
0xF980, 0xF960, 0xF960, 0xF960, 0xF960, 0xF940, 0xF940, 0xF940, 0xF940, 0xF920, 0xF920, 0xF920, 0xF920, 0xF900, 0xF900, 0xF900,
0xF900, 0xF8E0, 0xF8E0, 0xF8E0, 0xF8E0, 0xF8C0, 0xF8C0, 0xF8C0, 0xF8C0, 0xF8A0, 0xF8A0, 0xF8A0, 0xF8A0, 0xF880, 0xF880, 0xF880,
0xF880, 0xF860, 0xF860, 0xF860, 0xF860, 0xF840, 0xF840, 0xF840, 0xF840, 0xF820, 0xF820, 0xF820, 0xF820, 0xF800, 0xF800, 0xF800,
0xF800, 0xF800, 0xF800, 0xF800, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xE800, 0xE800, 0xE800, 0xE800,
0xE800, 0xE800, 0xE800, 0xE800, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xD800, 0xD800, 0xD800, 0xD800,
0xD800, 0xD800, 0xD800, 0xD800, 0xD800, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xC800, 0xC800, 0xC800,
0xC800, 0xC800, 0xC800, 0xC800, 0xC800, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xB800, 0xB800, 0xB800,
0xB800, 0xB800, 0xB800, 0xB800, 0xB800, 0xB800, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xA800, 0xA800,
0xA800, 0xA800, 0xA800, 0xA800, 0xA800, 0xA800, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0x9800, 0x9800,
0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x8800,
0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000
};


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
complex_hard_t data_hard[FFT_N];

uint16_t hann[FFT_N];
#define FPS_MODULO (SAMPLE_RATE/FFT_N/2/30) 
uint16_t fps_counter;

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

uint16_t color_scale(const float value)
{
  /*uint16_t color_code ;
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
 */
  int index = round(value * (COLOR_LUT_SIZE-1)) ;
  if(index < 0) index = 0;
  if(index > (COLOR_LUT_SIZE-1)) index = (COLOR_LUT_SIZE-1);
  return jetMap[index];
  //return color_code ;
}




void hann_init(){
	for (int i = 0; i < FFT_N; i++) {
    		float multiplier = 0.5f * (1.f - cos(2.f*M_PI*i/(FFT_N-1))) * 65535.f;// FP Q1.16
		hann[i] = floor(multiplier) ;
	}
}

void generate_sinewave_stereo(uint32_t freq, int32_t * buffer, uint32_t nb_samples){
	int i;
	for(i = 0 ; i < nb_samples ; i ++){
		int32_t temp = 2048*cosf(2*M_PI*freq*(i*1./SAMPLE_RATE));
		buffer[i*2] = temp ;
		buffer[i*2+1] = temp;
	}
}

void update_image( float* hard_power,  float  pw_max, float  pw_min, uint16_t* pImage);
uint16_t dbToColor(float db, float max, float min);
int main(void)
{
    int i;
    float pmax = 0 , pmin = 0;
    char title_buffer [8] = {0} ;

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
    fps_counter = 0 ;
    hann_init();
    pmax = MAX_DB ;
    pmin = MIN_DB ;
    lcd_draw_string(SPECTROGRAM_HEIGHT + 10 , 120, "STFT", BLACK);
    sprintf(title_buffer,"%u",0);
    lcd_draw_string(0 , SPECTROGRAM_LENGTH+5, title_buffer, BLACK);
    sprintf(title_buffer,"%u",SAMPLE_RATE/8000);
    lcd_draw_string(SPECTROGRAM_HEIGHT/4-5 , SPECTROGRAM_LENGTH+5, title_buffer, BLACK);
    sprintf(title_buffer,"%u",SAMPLE_RATE/4000);
    lcd_draw_string(SPECTROGRAM_HEIGHT/2-5 , SPECTROGRAM_LENGTH+5, title_buffer, BLACK);
    sprintf(title_buffer,"%u",3*SAMPLE_RATE/8000);
    lcd_draw_string(3*(SPECTROGRAM_HEIGHT/4)-5 , SPECTROGRAM_LENGTH+5, title_buffer, BLACK);
    sprintf(title_buffer,"%u",SAMPLE_RATE/2000);
    lcd_draw_string(SPECTROGRAM_HEIGHT-5 , SPECTROGRAM_LENGTH+5, title_buffer, BLACK);
    while (1)
    {

	//Channels layout is stereo 32bit aligned
	memcpy(rx_buf, &rx_buf[FRAME_LEN], FRAME_LEN*sizeof(int32_t));//To overlap samples ...
        i2s_receive_data_dma(I2S_DEVICE_0, (uint32_t *) &rx_buf[FRAME_LEN], FRAME_LEN, DMAC_CHANNEL3);//Only getting the overlap of FFT_N/2
	//i2s_receive_data_dma(I2S_DEVICE_0, (uint32_t *) rx_buf, FRAME_LEN * 2, DMAC_CHANNEL3);//This receive  interlaced 32-bits samples RLRLRLRLRL
	//i2s_receive_data(I2S_DEVICE_0,I2S_CHANNEL_0, (uint32_t *) rx_buf, FRAME_LEN);
	//generate_sinewave_stereo(2000, (int32_t *) rx_buf, FRAME_LEN);
        for ( i = 0; i < FFT_N / 2; ++i)
        {
	    input_data = (fft_data_t *)&buffer_input[i];
	    int32_t v = ((int32_t) (rx_buf[4*i])) * ((int32_t) hann[(i*2)]);
	    v = v >> 16 ;
            input_data->R1 = v ;
	    //input_data->R1 = rx_buf[2*i];   // data_hard[2 * i].real;
            input_data->I1 = 0;             // data_hard[2 * i].imag;
            v = ((uint32_t) rx_buf[(4*i)+2]) * ((uint32_t) hann[(i*2)+1]);
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

	memcpy(&g_lcd_gram[SPECTROGRAM_HEIGHT], g_lcd_gram_old, (SPECTROGRAM_HEIGHT*(SPECTROGRAM_LENGTH-1))*sizeof(uint16_t));
	for (i = 0; i < SPECTROGRAM_HEIGHT; i++)//Only interested in one size of the FFT
        {
            float pow = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);
            pow = pow/FFT_N  ;
            pow = 20*log(pow) ;
            uint16_t c = dbToColor(pow, pmax, pmin);
            g_lcd_gram[i] = c ;
        }
        memcpy(&g_lcd_gram_old, g_lcd_gram, (SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH)*sizeof(uint16_t));
	fps_counter ++ ;
	if(fps_counter >= FPS_MODULO){
		fps_counter = 0 ;
		lcd_draw_picture(0, 0, SPECTROGRAM_HEIGHT, SPECTROGRAM_LENGTH, (uint32_t*) g_lcd_gram);
	}
    }

    return 0;
}


uint16_t dbToColor(float db, const float max, const float min){
	db = (db - min)/(max - min);
	if(db < 0) db = 0;
	if(db > 1.0) db = 1.0;
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

