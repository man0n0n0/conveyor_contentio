# minimal_stepper_uart.py -- MicroPython

import machine
import time

# --- Stepper configuration ---
ENABLE_ACTIVE_LOW = True
STEP_PULSE_US = 5
STEPS_PER_MM = {'X':80, 'Y':80, 'Z':400, 'E0':95, 'E1':95}

PIN_NUM = {
    'X_ENABLE': 128, 'X_STEP': 129, 'X_DIR': 130,
    'Y_ENABLE': 131, 'Y_STEP': 132, 'Y_DIR': 133,
    'Z_ENABLE': 134, 'Z_STEP': 135, 'Z_DIR': 136,
    'E0_ENABLE': 137, 'E0_STEP': 138, 'E0_DIR': 139,
    'E1_ENABLE': 140, 'E1_STEP': 141, 'E1_DIR': 142,
}

def mk_pin(name, mode=machine.Pin.OUT):
    return machine.Pin(PIN_NUM[name], mode)

STEPPERS = {
    'X': {'EN': mk_pin('X_ENABLE'), 'STEP': mk_pin('X_STEP'), 'DIR': mk_pin('X_DIR')},
    'Y': {'EN': mk_pin('Y_ENABLE'), 'STEP': mk_pin('Y_STEP'), 'DIR': mk_pin('Y_DIR')},
    'Z': {'EN': mk_pin('Z_ENABLE'), 'STEP': mk_pin('Z_STEP'), 'DIR': mk_pin('Z_DIR')},
    'E0':{'EN': mk_pin('E0_ENABLE'),'STEP': mk_pin('E0_STEP'),'DIR': mk_pin('E0_DIR')},
    'E1':{'EN': mk_pin('E1_ENABLE'),'STEP': mk_pin('E1_STEP'),'DIR': mk_pin('E1_DIR')},
}

# Enable/disable steppers
def set_enable(axis, on):
    en = STEPPERS[axis]['EN']
    if ENABLE_ACTIVE_LOW:
        en.value(0 if on else 1)
    else:
        en.value(1 if on else 0)

for a in STEPPERS:
    set_enable(a, False)

# Step pulse
def step_pulse(step_pin):
    step_pin.value(1)
    time.sleep_us(STEP_PULSE_US)
    step_pin.value(0)

# Linear move (simple DDA-style)
def move_linear(steps):
    axes = [k for k in steps if steps[k]!=0]
    if not axes:
        return

    counts = {a: abs(steps[a]) for a in axes}
    for a in axes:
        STEPPERS[a]['DIR'].value(1 if steps[a]>=0 else 0)
        set_enable(a, True)

    max_steps = max(counts.values())
    acc = {a:0 for a in axes}

    for i in range(max_steps):
        for a in axes:
            acc[a] += counts[a]
            if acc[a] >= max_steps:
                step_pulse(STEPPERS[a]['STEP'])
                acc[a] -= max_steps
        time.sleep_us(2000)  # adjust speed

    for a in axes:
        set_enable(a, False)

# --- UART2 setup ---
uart2 = machine.UART(2, baudrate=115200, tx=machine.Pin(17), rx=machine.Pin(16))
print("Stepper listener on UART2 @115200")

buf = b""

while True:
    if uart2.any():
        buf += uart2.read(uart2.any())
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                line = line.decode().strip()
            except:
                continue
            if not line:
                continue

            # Parse G0/G1
            if line.startswith(('G0','G1')):
                parts = line.split()
                move_args = {}
                for p in parts[1:]:
                    axis = p[0].upper()
                    try:
                        val = float(p[1:])
                    except:
                        continue
                    if axis in STEPPERS:
                        move_args[axis] = int(round(val * STEPS_PER_MM[axis]))
                move_linear(move_args)
