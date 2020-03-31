from machine import Timer
from machine import I2C
import touchscreen as ts
from Maix import I2S, GPIO, FFT
from machine import UART
import sensor
import image
import time
import lcd
import utime
from fpioa_manager import *
import audio
import KPU as kpu
import axp202
import pcf8563

BACKLIGHT_PIN_NUM = 17
BACKLIGHT_FPIOA = fm.fpioa.GPIO0
BACKLIGHT_GPIO = GPIO.GPIO0

LED_PIN_NUM = 22
LED_FPIOA = fm.fpioa.GPIO1
LED_GPIO = GPIO.GPIO1

BOOT_PIN_NUM = 16
BOOT_FPIOA = fm.fpioa.GPIO2
BOOT_GPIO = GPIO.GPIO2

PIR_PIN_NUM = 23
PIR_FPIOA = fm.fpioa.GPIOHS0
PIR_GPIO = GPIO.GPIOHS0

x = 0
y = 0

# Init lcd 
lcd.init(freq=15000000,color=(0,0,0))
lcd.rotation(1)
lcd.clear((0,0,0))

# Init backlight 
fm.register(BACKLIGHT_PIN_NUM, BACKLIGHT_FPIOA)
bl = GPIO(BACKLIGHT_GPIO, GPIO.OUT)
bl.value(1)

fm.register(LED_PIN_NUM, LED_FPIOA)
led = GPIO(LED_GPIO, GPIO.OUT)

def irqHande(x):
    led.value(not led.value())

# pir
fm.register(PIR_PIN_NUM, PIR_FPIOA)
pir = GPIO(PIR_GPIO, GPIO.IN)
pir.irq(irqHande, GPIO.IRQ_RISING, GPIO.WAKEUP_NOT_SUPPORT, 7)

# button
fm.register(BOOT_PIN_NUM, BOOT_FPIOA)
button = GPIO(BOOT_GPIO, GPIO.IN)

i2c = I2C(I2C.I2C0, freq=400000, scl=30, sda=31)

addr = 0x68
img = image.Image()

def bytes_toint( firstbyte, secondbyte):
    if not firstbyte & 0x80:
        return firstbyte << 8 | secondbyte
    return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

def get_values():
    raw_ints = i2c.readfrom_mem(addr, 0x3B, 14)
    vals = {}
    vals["AcX"] = bytes_toint(raw_ints[0], raw_ints[1])
    vals["AcY"] = bytes_toint(raw_ints[2], raw_ints[3])
    vals["AcZ"] = bytes_toint(raw_ints[4], raw_ints[5])
    vals["Tmp"] = bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
    vals["GyX"] = bytes_toint(raw_ints[8], raw_ints[9])
    vals["GyY"] = bytes_toint(raw_ints[10], raw_ints[11])
    vals["GyZ"] = bytes_toint(raw_ints[12], raw_ints[13])
    return vals  # returned in range of Int16

def checkRTC():
    global img
    rtc = None
    i2cdev = i2c.scan()
    if 81 not in i2cdev:
        showInfo('setupRTC failed',False)
        time.sleep(3)
        return
    else:
        rtc = pcf8563.PCF8563(i2c)
        rtc.set_datetime((2020,3,1,0,0,0,3))

    start = utime.ticks_ms()

    while button.value():
        if utime.ticks_ms() - start > 1000:
            start = utime.ticks_ms()
            img.clear()
            t = rtc.datetime()
            string ='date:'+ str(t[0] + 2000) + '/' + str(t[1]) +'/'+ str(t[2])
            img.draw_string(0, 0, string, lcd.RED, scale=2)
            string = 'time:'+str(t[4]) + ':' + str(t[5]) +':'+ str(t[6])
            img.draw_string(0, 16, string, lcd.RED, scale=2)
            lcd.display(img)


def checkMPU6050():
    global img
    i2cdev = i2c.scan()
    if 104 not in i2cdev:
        showInfo('setupMPU6050 failed',False)
        return
    # img = image.Image()
    i2c.writeto(addr, bytearray([107, 0]))
    while button.value():
        a = get_values()
        if False:
            lcd.clear()
            string = 'AcX:' + str(a['AcX']) + ' GyX:' + str(a['GyX'])
            lcd.draw_string(0, 0, string,lcd.GREEN, lcd.BLACK)

            string = 'AcY:' + str(a['AcY']) + ' GyY:' + str(a['GyY'])
            lcd.draw_string(0, 16, string,lcd.GREEN, lcd.BLACK)

            string = 'AcZ:' + str(a['AcZ']) + ' GyZ:' + str(a['GyZ'])
            lcd.draw_string(0, 32, string,lcd.GREEN, lcd.BLACK)
            string = 'Tmp:' + str(a['Tmp']) + '*C'
            lcd.draw_string(0, 48, string,lcd.GREEN, lcd.BLACK)

            time.sleep(0.2)
        else:
            img.clear()
            string = 'AcX:' + str(a['AcX']) 
            img.draw_string(0, 0, string, lcd.RED, scale=2)
            string = 'AcY:' + str(a['AcY'])
            img.draw_string(0, 16, string, lcd.RED, scale=2)
            string = 'AcZ:' + str(a['AcZ'])
            img.draw_string(0, 32, string, lcd.RED, scale=2)
            string = 'GyX:' + str(a['GyX']) 
            img.draw_string(0, 48, string, lcd.RED, scale=2)
            string = 'GyY:' + str(a['GyY'])
            img.draw_string(0, 64, string, lcd.RED, scale=2)
            string = 'GyZ:' + str(a['GyZ'])
            img.draw_string(0, 80, string, lcd.RED, scale=2)
            string = 'Tmp:' + str(a['Tmp']) + '*C'
            img.draw_string(0, 96, string, lcd.RED, scale=2)
            lcd.display(img)
            time.sleep(0.5)

def showInfo(string,ret):
    global x,y
    if ret:
        lcd.draw_string(x, y, string,
                        lcd.GREEN, lcd.BLACK)
    else:
        lcd.draw_string(
            x, y, string, lcd.RED, lcd.BLACK)
    y = y + 16
    if y > 240:
        y = 0
        lcd.clear(lcd.BLACK)

def checkSDCard():
    dev = os.listdir('/')
    if 'sd' in dev:
        showInfo("find the sdcard", True)
    else:
        showInfo("sdcard not the found", False)

def checkNetwork():
    fm.register(6, fm.fpioa.UART1_TX)
    fm.register(7, fm.fpioa.UART1_RX)
    serial = UART(UART.UART1, 115200, 8, None, 1,timeout=1000, read_buf_len=4096)
    start = utime.ticks_ms()
    serial.write('at\r\n')
    while True:
        data = serial.read()
        if 'OK' in data:
            print(data)
            showInfo("esp32 respone is OK", True)
            break
        if utime.ticks_ms() > 3000:
            showInfo("esp32 no respone", False)
            break
    serial.deinit()
    del serial

def checkMems():
    fm.register(20, fm.fpioa.I2S0_IN_D0)
    fm.register(19, fm.fpioa.I2S0_WS)
    fm.register(18, fm.fpioa.I2S0_SCLK)
    mic = I2S(I2S.DEVICE_0)
    mic.channel_config(mic.CHANNEL_0, mic.RECEIVER,align_mode=I2S.STANDARD_MODE)
    mic.set_sample_rate(38640)
    img = image.Image()
    while button.value():
        audio = mic.record(1024)
        fft_res = FFT.run(audio.to_bytes(), 512)
        fft_amp = FFT.amplitude(fft_res)
        img = img.clear()
        x_shift = 0
        for i in range(50):
            if fft_amp[i] > 240:
                hist_height = 240
            else:
                hist_height = fft_amp[i]
            hist_width = int(240 / 50)
            img = img.draw_rectangle((x_shift, 240-hist_height, hist_width, hist_height), [255, 255, 255], 2, True)
            img.draw_string(0, 0, "Press btn break", lcd.GREEN, scale=2)
            x_shift = x_shift + hist_width
        lcd.display(img)
        fft_amp.clear()

def play(patht):
    global x,y
    x = 0
    y = 0
    showInfo("music playing", True)
    fm.register(34,fm.fpioa.I2S0_OUT_D1)
    fm.register(35,fm.fpioa.I2S0_SCLK)
    fm.register(33,fm.fpioa.I2S0_WS)
    wav_dev = I2S(I2S.DEVICE_0)
    player = audio.Audio(path = patht)
    player.volume(20)
    wav_info = player.play_process(wav_dev)
    print("wav file head information: ", wav_info)
    # wav_dev.channel_config(wav_dev.CHANNEL_1, I2S.TRANSMITTER,resolution = I2S.RESOLUTION_16_BIT ,cycles = I2S.SCLK_CYCLES_32, align_mode = I2S.RIGHT_JUSTIFYING_MODE)
    wav_dev.channel_config(wav_dev.CHANNEL_1, I2S.TRANSMITTER,resolution = I2S.RESOLUTION_16_BIT ,cycles = I2S.SCLK_CYCLES_32, align_mode = I2S.LEFT_JUSTIFYING_MODE)
    wav_dev.set_sample_rate(wav_info[1])
    while button.value():
        ret = player.play()
        if ret == None:
            print("format error")
            break
        elif ret==0:
            print("end")
            break
    player.finish()

def playMusic():
    try:
        filelist = os.listdir('/sd')
        for i in filelist:
            if '.wav' in i:
                path = '/sd/' + i
                play(path)
                return
        showInfo('Not fount *.wav file',False)
    except:
        showInfo('Not fount *.wav file',False)
        time.sleep(3)
    
def checkCamera():
    global x,y
    x = 0
    y = 0
    showInfo("camera starting", True)
    img = image.Image()
    try:
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)
        lcd.rotation(2)
    except:
        showInfo("camera init failed", False)
        return
    else:
        showInfo("camera init done", True)

    try:
        task = kpu.load(0x300000) 
        anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025)
        a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)
        while True:
            img = sensor.snapshot()
            code = kpu.run_yolo2(task, img)
            if code:
                for i in code:
                    a = img.draw_rectangle(i.rect(),color=lcd.BLUE)
            new = img.copy(roi=(0, 0, 239, 239))
            lcd.display(new)
    except :
        print('kup load fialed')
        while True:
            img = sensor.snapshot()
            new = img.copy(roi=(0, 0, 239, 239))
            lcd.display(new)

def checkTouch():
    global x,y
    x = 0
    y = 0
    try:
        ts.init(i2c, ts.FT62XX)
    except:
        print('faile to init touchpad')
        showInfo("touchpad init failed", False)
        return
    else:
        showInfo("touchpad init done", True)
        print('pass to init touchpad')
    time.sleep(2);
    lcd.clear()
    img = image.Image()
    status_last = ts.STATUS_IDLE
    x_last = 0
    y_last = 0
    draw = False
    while button.value():
        (status, x, y) = ts.read()
        if draw:
            img.draw_line((x_last, y_last, x, y))
        if status_last != status:
            if (status == ts.STATUS_PRESS or status == ts.STATUS_MOVE):
                draw = True
            else:
                draw = False
            status_last = status
        img.draw_string(0, 0, "Touch me", lcd.GREEN, scale=2)
        lcd.display(img)
        x_last = x
        y_last = y
    del img
    ts.__del__()

def i2cDevScan():
    i2cdev = i2c.scan()
    if 53 in i2cdev:
        print('find axp202')
        showInfo("find axp202", True)
    else:
        showInfo("axp202  not the found", False)

    if 56 in i2cdev:
        print('find ft6236')
        showInfo("find ft6236", True)
    else:
        showInfo("ft6236 not the found", False)

    if 81 in i2cdev:
        showInfo("find pcf8563", True)
        print('find pcf8563')
    else:
        showInfo("pcf8563 not the found", False)

    if 104 in i2cdev:
        print('find mpu6050')
        showInfo("find mpu6050", True)
    else:
        showInfo("mpu6050 init failed", False)


i2cDevScan()

p = None
try:
    p = axp202.PMU(i2c,0x35)
except:
    p = axp202.PMU(i2c,0x34)

if p is not None:
    p.setLDO2Voltage(1800)
    p.enablePower(axp202.AXP192_LDO2)
    p.enablePower(6)
    # p.setLDO3Voltage(2275)
    p.setLDO3Mode(1)
else:
    showInfo('power start failed')

# sdcard test
checkSDCard()
# esp32 respone test
checkNetwork()
time.sleep(5)
lcd.clear((0,0,0))

checkTouch()
time.sleep(1)
lcd.clear((0,0,0))

checkMPU6050()
time.sleep(1)
lcd.clear((0,0,0))

checkRTC()
time.sleep(1)
lcd.clear((0,0,0))

checkMems()
lcd.clear((0,0,0))
time.sleep(1)

playMusic()
time.sleep(1)
lcd.clear((0,0,0))

checkCamera()
