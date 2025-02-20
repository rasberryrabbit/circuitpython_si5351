Simple clock generator by circuitpython+si5351.  
clock range is 2kHz~ 60MHz.  

Used component.  
1. RP2040 Compatible board. This project use YD-RP2040 board.
2. SSD1306 OLED I2C 128x64.
3. si5351 module.
4. Rotary encoder module.
  
  
Pin connection  
   
rotary1 -> GP4  
rotary2 -> GP3  
rotarysw -> GP5  
  
scl -> GP27  
sda -> GP26  

Serial command syntax:  
clk{0-2}{e|d}f{frequency}p{0-1}  
clk{0-2} : clk selection  
{e|d|i} : enable or disable or invert output  
f{frequency} : frequency   
p{0-1} : Select PLL 0-1  
  
