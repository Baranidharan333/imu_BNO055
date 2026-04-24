from machine import I2C, Pin
import time
import network
import socket

BNO055_ADDR = 0x28

# Registers
OPR_MODE = 0x3D
PWR_MODE = 0x3E

QUAT = 0x20
GYRO = 0x14
ACC  = 0x08   # 🔥 RAW accel (for INS)

CONFIG_MODE = 0x00
NDOF_MODE   = 0x0C

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

def write(reg, val):
    i2c.writeto_mem(BNO055_ADDR, reg, bytes([val]))

def read(reg, n):
    return i2c.readfrom_mem(BNO055_ADDR, reg, n)

# ✅ Correct signed conversion
def read_vec(reg, n, scale):
    data = read(reg, n*2)
    out = []
    for i in range(n):
        raw = data[2*i] | (data[2*i+1] << 8)
        if raw > 32767:
            raw -= 65536
        out.append(raw / scale)
    return out

# INIT
time.sleep(1)
write(OPR_MODE, CONFIG_MODE)
time.sleep(0.05)
write(PWR_MODE, 0x00)
time.sleep(0.05)
write(OPR_MODE, NDOF_MODE)
time.sleep(0.1)

# WIFI
ssid = "PHY_AI"
password = "PHY_AI@IHUB"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

while not wlan.isconnected():
    time.sleep(1)

print("IP:", wlan.ifconfig()[0])

# UDP
UDP_IP = "192.168.200.153"   # your PC IP
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

dt = 0.01  # 100 Hz

while True:
    start = time.ticks_ms()

    q = read_vec(QUAT, 4, 16384.0)
    g = read_vec(GYRO, 3, 16.0)
    a = read_vec(ACC, 3, 100.0)

    data = f"Q:{q},A:{a},G:{g}"
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))

    elapsed = (time.ticks_ms() - start)/1000
    if elapsed < dt:
        time.sleep(dt - elapsed)
