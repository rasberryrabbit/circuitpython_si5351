from board import *
import neopixel_write
import digitalio
import rotaryio
import asyncio
import keypad
import adafruit_ssd1306
import busio
import silicon5351
import supervisor, re, sys

if board_id=="vcc_gnd_yd_rp2040":
    # disble onboard neopixel
    pin = digitalio.DigitalInOut(NEOPIXEL)
    pin.direction = digitalio.Direction.OUTPUT
    pixel_off = bytearray([0, 0, 0])
    neopixel_write.neopixel_write(pin, pixel_off)

# pin defines
rotary1 = GP4
rotary2 = GP3
rotarysw = GP5

scl=GP27
sda=GP26

# 2000Hz ~ 60MHz
pllmul=20
minfreq=pllmul*100

# init i2c
i2c = busio.I2C(scl=scl,sda=sda)

# list i2c device
while not i2c.try_lock():
    pass
devices = i2c.scan()
i2c.unlock()

if devices:
    for device in devices:
        print("device", device)


# global variables
last_position = None
position_diff = None
iMode = 0
iMenu = 0
iMPos = 0
iOut = 0
ipll = [0,0,0]
ipllchg = [0,0,0]
iFreq = [bytearray(b"000002000"),bytearray(b"000002000"),bytearray(b"000002000")]
iEna = [0,0,0]
ikeydur = 0
datpat=re.compile("clk(\d)(e|d)f([0-9]+)(p[0-9])?")

# init display
display = adafruit_ssd1306.SSD1306_I2C(128,64,i2c)

# init si5351
si5351=silicon5351.SI5351_I2C(i2c, crystal=25e6)
si5351.setup_pll(pll=0, mul=pllmul)
si5351.setup_pll(pll=1, mul=pllmul)
si5351.init_clock(output=0, pll=ipll[0])
si5351.init_clock(output=1, pll=ipll[1])
si5351.init_clock(output=2, pll=ipll[2])

# init rotary encoder
enc = rotaryio.IncrementalEncoder(rotary1, rotary2)

def update_focus(bMark):
    global display, iMPos, iOut

    display.fill_rect(0,(0*2+1)*8,128,8,0)
    display.fill_rect(0,(1*2+1)*8,128,8,0)
    display.fill_rect(0,(2*2+1)*8,128,8,0)
    if bMark:
        if iMPos>0 and iMPos<10:
            display.text('^',30+iMPos*6,(iOut*2+1)*8,1)
        elif iMPos>9:
            display.text('^',96,(iOut*2+1)*8,1)
        else:
            display.text('^',24+iMPos*6,(iOut*2+1)*8,1)
    display.show()

def update_freq():
    global display,iFreq,iOut,iEna,ipll
    # on/off
    display.fill_rect(24,iOut*16,6,8,0)
    if iEna[iOut]==0:
        display.text('x',24,iOut*16,1)
    else:
        display.text('o',24,iOut*16,1)
    # frequency
    display.fill_rect(36,iOut*16,80,8,0)
    display.text('%s' % iFreq[iOut].decode("utf-8"),36,iOut*16,1)
    display.fill_rect(0,48,128,16,0)
    # pll
    display.fill_rect(96,iOut*16,7,8,0)
    display.text('%d' % ipll[iOut],96,iOut*16,1)
    display.show()
    
def update_si5351():
    global si5351,iOut,iEna,iFreq,display,ipll,ipllchg
    
    ifreq=int(iFreq[iOut].decode("utf-8"))
    try:
        if ipllchg[iOut]!=0:
            si5351.init_clock(output=iOut, pll=ipll[iOut])
            ipllchg[iOut]=0
        si5351.set_freq_fixedpll(output=iOut, freq=ifreq)
        if iEna[iOut]==0:
            si5351.disable_output(output=iOut)
        else:
            si5351.enable_output(output=iOut)
    except Exception as err:
        print(err)
        display.fill_rect(0,48,128,8,1)
        display.text(str(err),0,48,0)
        display.show()

async def catch_key(pin):
    global iMode
    global ikeydur
    with keypad.Keys((pin,), value_when_pressed=False, pull=True) as keys:
        while True:
            if event := keys.events.get():
                if event.pressed:
                    ikeydur=event.timestamp
                    iMode+=1
                    if iMode>2:
                        iMode=1
                    update_focus(True)
                if event.released and event.timestamp-ikeydur>1000:
                    iMode=0
                    update_focus(False)
                    update_si5351()

            await asyncio.sleep(0)
        
async def process_rotary():
    global last_position,position_diff
    global iMode,iFreq,iMenu,iMPos,iOut,iEna
    global minfreq, datpat, ipll, ipllchg
    while True:
        position = enc.position
        if last_position == None or position != last_position:
            if last_position == None:
                position_diff=position
            else:
                position_diff=position-last_position
            last_position = position
            if iMode==0:
                iOut+=position_diff
                if iOut>2:
                    iOut=0
                elif iOut<0:
                    iOut=2
                display.fill_rect(0,0,8,48,0)
                display.text('>',0,iOut*2*8,1)
                display.show()
            elif iMode==1:
                iMPos+=position_diff
                if iMPos<0:
                    iMPos=10
                elif iMPos>10:
                    iMPos=0
                update_focus(True)
            else:
                if iMPos>0 and iMPos<10:
                    ch=iFreq[iOut][iMPos-1]
                    val=ch-0x30                        
                    if iMPos>1:
                        ch2=iFreq[iOut][iMPos-2]
                        val2=ch2-0x30
                    val+=position_diff
                    if iMPos==1:
                        if val>1:
                            val=0
                        elif val<0:
                            val=1
                    else:
                        if val>9:
                            val=0
                            if iMPos>1:
                                val2+=1
                                if val2>9:
                                    val2=0
                        elif val<0:
                            val=9
                            if iMPos>1:
                                val2-=1
                                if val2<0:
                                    val2=9
                    iFreq[iOut][iMPos-1]=0x30+val
                    if iMPos>1:
                        iFreq[iOut][iMPos-2]=0x30+val2
                    if int(iFreq[iOut].decode("utf-8"))<minfreq:
                        iFreq[iOut][iMPos-1]=ch
                        if iMPos>1:
                            iFreq[iOut][iMPos-2]=ch2
                elif iMPos==0:
                    iEna[iOut]=1-iEna[iOut]
                else:
                    ipll[iOut]=1-ipll[iOut]
                    ipllchg[iOut]=1
                update_freq()
                update_si5351()
        # serial command, clk{0-1}{e|d}f{frequency}
        n=supervisor.runtime.serial_bytes_available
        if n>0:
            data = sys.stdin.read(n)
            validcmd=True
            if data != "":
                sdata=datpat.match(data)
                if sdata:
                    # clk
                    iclk=int(sdata.group(1))
                    if iclk>=0 and iclk<3:
                        iOut=iclk
                        # on/off
                        if sdata.group(2)=="e":
                            iEna[iclk]=1
                        else:
                            iEna[iclk]=0
                        # set frequency
                        ifre=int(sdata.group(3))
                        if ifre>=minfreq and ifre<200000000:
                            sfre="%09d" % ifre
                            iFreq[iclk]=bytearray(sfre.encode("utf-8"))
                            update_freq()
                            update_si5351()
                        else:
                            validcmd=False
                            print("Frequency out of range minimun: %d" % minfreq)
                        if sdata.group(4):
                            spll=sdata.group(4)[1:]
                            vpll=int(spll)
                            if vpll>=0 and vpll<2:
                                if ipll[iOut]!=vpll:
                                    ipll[iOut]=vpll
                                    ipllchg[iOut]=1
                            else:
                                print("Out of PLL range 0-1")
                    else:
                        validcmd=False
                else:
                    validcmd=False
            if not validcmd:
                print("Syntax: clk{0-2}{e|d}f{frequency}p{0-1}")
        await asyncio.sleep(0)
            
def draw_screen():
    global display
    display.fill(0)
    display.text('  0 x %9s 0' % iFreq[0].decode("utf-8"),0,0,1)
    display.text('  1 x %9s 0' % iFreq[1].decode("utf-8"),0,16,1)
    display.text('  2 x %9s 0' % iFreq[2].decode("utf-8"),0,32,1)
    display.show()

            
async def main():
    draw_screen()
    update_si5351()
    # pin interrupt
    interrupt_task=asyncio.create_task(catch_key(rotarysw))
    rotary_task=asyncio.create_task(process_rotary())
    await asyncio.gather(interrupt_task,rotary_task)
        
asyncio.run(main())
