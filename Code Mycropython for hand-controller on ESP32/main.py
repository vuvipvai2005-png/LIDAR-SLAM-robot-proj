from machine import Pin, I2C
import time
import network
import struct
from sh1106 import SH1106_I2C  
from machine import ADC, PWM
import socket

# =========================================================
# còi và các hàm liên quan
# =========================================================
BUZZER_PIN = 4
buzzer = Pin(BUZZER_PIN, Pin.OUT, value=0)
lastDebounceTime = 0
DEBOUNCE_MS = 50

_beep_active = False
_beep_start = 0
_beep_duration = 50  #ms

def beep():
    global _beep_active, _beep_start
    if not _beep_active:
        buzzer.value(1)
        _beep_active = True
        _beep_start = time.ticks_ms()

def update_buzzer():
    global _beep_active
    if _beep_active:
        if time.ticks_diff(time.ticks_ms(), _beep_start) >= _beep_duration:
            buzzer.value(0)
            _beep_active = False

# =========================================================
# wifi và udp
# =========================================================
ssid = "P203"
password = "203withlove"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

while not wlan.isconnected():
    print("Connecting WiFi...")
    time.sleep(1)

print("Da ket noi:", wlan.ifconfig())

UDP_IP = "192.168.80.38"
UDP_PORT = 12349
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def pack_data(x, y,z,t):
    return struct.pack("<Bhhbb", 0xAA, x, y, mode, chosenItem1)

# =========================================================
# ngoại vi: LED và ADC
# =========================================================
R = PWM(Pin(25), freq=1000)
G = PWM(Pin(26), freq=1000)
B = PWM(Pin(27), freq=1000)

def set_color(r, g, b):
    R.duty(1023 - r)
    G.duty(1023 - g)
    B.duty(1023 - b)

adc1 = ADC(Pin(34))
adc2 = ADC(Pin(35))
adc1.atten(ADC.ATTN_11DB)
adc2.atten(ADC.ATTN_11DB)
def read_avg(adc, n=100):
    s = 0
    for _ in range(n):
        s += adc.read()
        time.sleep_ms(2)
    return s // n

print("De joystick o vi tri HOME")
time.sleep(1)

CENTER_X = read_avg(adc1, 200)
CENTER_Y = read_avg(adc2, 200)

print("Center_X =", CENTER_X)
print("Center_Y =", CENTER_Y)
DEADZONE = 120
MAX_RANGE = 1800

def normalize_axis(raw, center):
    value = raw - center

    if abs(value) < DEADZONE:
        return 0

    if value > 0:
        value = value - DEADZONE
    else:
        value = value + DEADZONE

    value = int(value * 1000 / MAX_RANGE)

    if value > 1000:
        value = 1000
    elif value < -1000:
        value = -1000

    return value
# =========================================================
# khởi tạo oled và các hàm liên quan
# =========================================================
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64
BTN_PIN = 14
BTN2_PIN = 16

# Khởi tạo I2C với tốc độ cao 400kHz
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
oled = SH1106_I2C(SCREEN_WIDTH, SCREEN_HEIGHT, i2c)

btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)
btn2 = Pin(BTN2_PIN, Pin.IN, Pin.PULL_UP)

mode = 0 

menuItems = [
    "      MENU",
    "1. Mo hinh DK",      
    "2. Chinh he so",
    "3. DK thu cong",     
    "4. Reset he thong"
]

mhdkItems = [
    "1. PID",
    "2. PID + FuzzyPP",
    "3. FuzzyPID+Fuzz",
    "4. RBF Neural",
    "5. GA-PID"
]

selectedRow = 1     
chosenItem = -1     
selectedRow1 = 0    
chosenItem1 = -1    
chosenJoy = 0

lastBtnState = 1
btnPressTime = 0
btnHeld = False
shortPressHandled = False
HOLD_TIME = 2000    

lastBlinkTime = 0
blinkState = True
BLINK_INTERVAL = 500

# =========================================================
# các hàm vẽ màn hình
# =========================================================
def drawMenu():
    oled.fill(0)
    for i in range(len(menuItems)):
        yPos = i * 12 
        if i == selectedRow and i != 0:
            if blinkState:
                oled.text(menuItems[i], 0, yPos + 2, 1)
            else:
                oled.fill_rect(0, yPos, SCREEN_WIDTH, 12, 1)
                oled.text(menuItems[i], 0, yPos + 2, 0)
        else:
            oled.text(menuItems[i], 0, yPos + 2, 1)
    oled.show()

def drawSubMenu():
    oled.fill(0)
    for i in range(len(mhdkItems)):
        yPos = i * 12
        if i == selectedRow1:
            if blinkState:
                oled.text(mhdkItems[i], 0, yPos + 2, 1)
            else:
                oled.fill_rect(0, yPos, SCREEN_WIDTH, 12, 1)
                oled.text(mhdkItems[i], 0, yPos + 2, 0)
        else:
            oled.text(mhdkItems[i], 0, yPos + 2, 1)
    oled.show()

def drawConfirmed():
    oled.fill(0)
    oled.text("Confirmed:", 0, 10, 1)
    oled.text(mhdkItems[chosenItem1], 0, 30, 1)
    oled.show()

def drawResult():
    oled.fill(0)
    oled.text("Muc: " + str(chosenItem), 40, 20, 1)
    oled.text("Da chon dong " + str(chosenItem), 10, 40, 1)
    oled.show()

# Thuật toán vẽ vòng tròn
def draw_circle(x0, y0, r, color):
    f = 1 - r
    ddf_x = 1
    ddf_y = -2 * r
    x = 0
    y = r
    oled.pixel(x0, y0 + r, color)
    oled.pixel(x0, y0 - r, color)
    oled.pixel(x0 + r, y0, color)
    oled.pixel(x0 - r, y0, color)
    while x < y:
        if f >= 0:
            y -= 1
            ddf_y += 2
            f += ddf_y
        x += 1
        ddf_x += 2
        f += ddf_x
        oled.pixel(x0 + x, y0 + y, color)
        oled.pixel(x0 - x, y0 + y, color)
        oled.pixel(x0 + x, y0 - y, color)
        oled.pixel(x0 - x, y0 - y, color)
        oled.pixel(x0 + y, y0 + x, color)
        oled.pixel(x0 - y, y0 + x, color)
        oled.pixel(x0 + y, y0 - x, color)
        oled.pixel(x0 - y, y0 - x, color)

def drawManualControl(val1, val2):
    oled.fill(0)

    oled.text("ADC1", 0, 5, 1)
    oled.text(str(val1), 0, 15, 1)
    oled.text("ADC2", 0, 35, 1)
    oled.text(str(val2), 0, 45, 1)

    cx = 85
    cy = 32
    r = 30

    draw_circle(cx, cy, r, 1)

    dx = (val1 / 4095.0) * 2 - 1.0
    dy = (val2 / 4095.0) * 2 - 1.0

    dist = (dx * dx + dy * dy) ** 0.5
    if dist > 1.0:
        dx = dx / dist
        dy = dy / dist

    dot_x = cx + int(dx * r)
    dot_y = cy - int(dy * r)

    oled.fill_rect(dot_x - 6, dot_y - 6, 13, 13, 1)
    oled.show()
# =========================================================
# chạy một lần
# =========================================================
drawMenu()
need_redraw = False  
# =========================================================
# Timer cho ADC / UDP / OLED
# =========================================================
ADC_PERIOD_MS = 5       # đọc ADC 200Hz
UDP_PERIOD_MS = 20      # gửi UDP 50Hz
OLED_PERIOD_MS = 50     # OLED 20 FPS

last_adc_time = 0
last_udp_time = 0
last_oled_time = 0
last_btn2_time = 0

adc1_val = 0
adc2_val = 0

sock.setblocking(False)
while True:
    now = time.ticks_ms()

    btnState = btn.value()
    btn2State = btn2.value()

    # =====================================================
    # đọc ADC
    # =====================================================
    if time.ticks_diff(now, last_adc_time) >= ADC_PERIOD_MS:
        adc1_val = adc1.read()
        adc2_val = adc2.read()
        last_adc_time = now

    # =====================================================
    # xử lý nút "quay về"
    # =====================================================
    if btn2State == 0 and time.ticks_diff(now, last_btn2_time) >= DEBOUNCE_MS:
        mode = 0
        need_redraw = True
        last_btn2_time = now

    # =====================================================
    # xử lý nút "chọn"
    # =====================================================
    if btnState == 0 and lastBtnState == 1:
        beep()
        btnPressTime = now
        btnHeld = False
        shortPressHandled = False

    if btnState == 0 and not btnHeld:
        if time.ticks_diff(now, btnPressTime) >= HOLD_TIME:
            btnHeld = True
            need_redraw = True

            if mode == 0:
                chosenItem = selectedRow
                if chosenItem == 1:
                    mode = 1
                    selectedRow1 = 0
                else:
                    mode = 3
            elif mode == 1:
                chosenItem1 = selectedRow1
                mode = 2

    if btnState == 1 and lastBtnState == 0:
        if not btnHeld and not shortPressHandled:
            need_redraw = True

            if mode == 0:
                selectedRow = (selectedRow + 1) % len(menuItems)
                if selectedRow == 0:
                    selectedRow += 1
            elif mode == 1:
                selectedRow1 = (selectedRow1 + 1) % len(mhdkItems)

            shortPressHandled = True

        if btnHeld:
            btnHeld = False

    lastBtnState = btnState

    # =====================================================
    # chớp nháy menu
    # =====================================================
    if time.ticks_diff(now, lastBlinkTime) >= BLINK_INTERVAL:
        blinkState = not blinkState
        lastBlinkTime = now
        need_redraw = True

    # =====================================================
    # OLED vẽ theo chu kỳ riêng
    # =====================================================
    if mode == 3 and chosenItem == 3:
        if time.ticks_diff(now, last_oled_time) >= OLED_PERIOD_MS:
            drawManualControl(adc1_val, adc2_val)
            last_oled_time = now
    else:
        if need_redraw:
            if mode == 0:
                drawMenu()
            elif mode == 1:
                drawSubMenu()
            elif mode == 2:
                drawConfirmed()
            elif mode == 3:
                drawResult()

            need_redraw = False

    # =====================================================
    # LED
    # =====================================================
    if chosenItem1 == 0:
        set_color(1023, 0, 0)
    elif chosenItem1 == 1:
        set_color(0, 1023, 0)
    elif chosenItem1 == 2:
        set_color(0, 0, 1023)
    elif chosenItem1 == 3:
        set_color(1023, 1023, 0)
    elif chosenItem1 == 4:
        set_color(1023, 1023, 1023)

    # =====================================================
    # gửi udp theo chu kỳ riêng
    # =====================================================
    adc1_send  = normalize_axis(adc1_val, CENTER_X)
    adc2_send = normalize_axis(adc2_val, CENTER_Y)
    if time.ticks_diff(now, last_udp_time) >= UDP_PERIOD_MS:
        try:
            packet = pack_data(adc1_send, adc2_send, mode, chosenItem1)
            sock.sendto(packet, (UDP_IP, UDP_PORT))
        except OSError:
            pass

        last_udp_time = now

    update_buzzer()

    # delay nhẹ
    time.sleep_ms(1)