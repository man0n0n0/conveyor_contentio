import network
import socket
import time

# --- CONFIGURE THESE ---
SSID = 'ok'
PASSWORD = '123456789'
FLUIDNC_IP = '192.168.8.222'   # Change to your FluidNC IP
GCODE = 'G0X10\n'        # Example G-code command
# -----------------------

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print('Connecting to Wi-Fi...')
    wlan.connect(SSID, PASSWORD)
    for _ in range(20):  # 10 seconds total
        if wlan.isconnected():
            break
        print('.', end='')
        time.sleep(0.5)

# Connect to FluidNC via Telnet (port 23)
addr = socket.getaddrinfo(FLUIDNC_IP, 23)[0][-1]
s = socket.socket()
s.connect(addr)
print('Connected to FluidNC')

# Wake up FLUIDNC
s.send(b'\r\n') 
time.sleep(2)

s.send(b'$X\n')
time.sleep(0.1)

# Send G-code
s.send(GCODE.encode())
print('Sent:', GCODE.strip())

# Optionally read response
time.sleep(0.1)
data = s.recv(1024)
print('Response:', data.decode())

# Close connection
s.close()
print('Connection closed')
