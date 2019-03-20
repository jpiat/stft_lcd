# stft_lcd


Largely inspired from https://github.com/MrJBSwe/fft_lcd.git

Prerequisites toolchain
https://github.com/kendryte

Test of HW FFT with Sipeed MAIX GO

build
cmake .. -DPROJ=stft_lcd -DTOOLCHAIN=/opt/kendryte-toolchain/bin && make
 
flash
sudo python3 kflash.py -p /dev/ttyUSB0 stft_lcd.bin

