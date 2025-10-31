import machine
import time

# Initialize UART (for ESP32: UART1 on TX=12, RX=13)
uart = machine.UART(1, baudrate=115200, tx=12, rx=13)

# Reset/initialize GRBL
uart.write(b"\r\n\r\n")
time.sleep(2)   # Wait for GRBL to initialize and flush startup text

# Flush input buffer
uart.read()

# Unlock GRBL
uart.write(b"$X\n")
time.sleep(0.1)
resp = uart.read()
if resp:
    print(resp.decode())

# Send a G-code move command
uart.write(b"$Motor/Enable\n")
time.sleep(0.1)
resp = uart.read()
if resp:
    print(resp.decode())
else:
    print("No response received.")



    