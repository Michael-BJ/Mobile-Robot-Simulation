# 05-02-2025
# Michael Bryan Jahanto

# merupakan update dari versi terdahulu, yang sudah berhasil berkomunikasi mengirimkan index motor, kecepatan linear roda, kecepatan sudut roda 
# ke raspberry pi (pada program main_version2.py)

# kekurangannya, masih belum mengirimkan data kecepatan linear dan sudut dari robot secara keseluruhan


from machine import Pin, Timer, PWM
import select
import sys
import time

machine.freq(200000000)

# Inisialisasi LED bawaan
led = Pin(25, Pin.OUT)

# Variabel global untuk menyimpan data yang diterima
input_data = None

# Define GPIO pins for each encoder channel
encoder_channels = [(8, 9), (27, 26), (10, 11), (21, 20)]  # Encoder untuk keempat roda

# Define driver pins for Mecanum wheels (FR, FL, BL, BR)
motor_pins = [(6, 7), (19, 18), (14, 15), (17, 16)]  # (RPWM, LPWM) untuk setiap motor

# Variabel untuk menyimpan jumlah pulsa encoder
encoder_counts = [0, 0, 0, 0]
last_encoder_time = [time.ticks_ms()] * 4

# PWM frequency for motor control
PWM_FREQ = 1000

# Jumlah pulsa per putaran encoder
PULSES_PER_REVOLUTION = 884

# Jari-jari roda dalam meter
WHEEL_RADIUS = 0.03

# Initialize motor PWM controls
motor_pwms = []
for rpwm_pin, lpwm_pin in motor_pins:
    rpwm = PWM(Pin(rpwm_pin))
    lpwm = PWM(Pin(lpwm_pin))
    rpwm.freq(PWM_FREQ)
    lpwm.freq(PWM_FREQ)
    motor_pwms.append((rpwm, lpwm))

# Initialize encoder pins
encoders = []
for channel_a, channel_b in encoder_channels:
    encoders.append((Pin(channel_a, Pin.IN), Pin(channel_b, Pin.IN)))

# Fungsi untuk menangani interrupt encoder
def encoder_isr(motor_index):
    def isr(pin):
        global encoder_counts
        encoder_counts[motor_index] += 1
    return isr

# Attach interrupt untuk setiap encoder
for i, (channel_a, channel_b) in enumerate(encoders):
    channel_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr(i))

# Fungsi untuk menangani input UART
def handle_uart_input(timer):
    global input_data
    try:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            data = sys.stdin.readline().strip()
            if data:
                input_data = data
    except Exception as e:
        print(f"Error reading input: {e}")

# Timer untuk memeriksa input secara berkala
uart_timer = Timer(-1)
uart_timer.init(period=100, mode=Timer.PERIODIC, callback=handle_uart_input)

# Fungsi untuk mengendalikan motor dengan umpan balik encoder
def set_motor_speed_with_feedback(motor_index, target_speed):
    global encoder_counts
    rpwm, lpwm = motor_pwms[motor_index]
    
    target_duty = int(abs(target_speed) * 65535 / 100)
    
    current_count = encoder_counts[motor_index]
    time.sleep(0.1)
    delta_count = encoder_counts[motor_index] - current_count
    actual_speed = delta_count
    
    if target_speed > 0:
        lpwm.duty_u16(target_duty)
        rpwm.duty_u16(0)
    elif target_speed < 0:
        rpwm.duty_u16(target_duty)
        lpwm.duty_u16(0)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)

# Fungsi untuk mengatur kecepatan semua motor secara bersamaan
def set_all_motors_speed(target_speed):
    for i in range(4):
        set_motor_speed_with_feedback(i, target_speed)

def calculate_speeds():
    global encoder_counts, last_encoder_time
    output_data = []

    for i in range(4):
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, last_encoder_time[i]) / 1000.0

        if delta_time > 0:
            angular_speed = (encoder_counts[i] / PULSES_PER_REVOLUTION) * (2 * 3.14159) / delta_time
            linear_speed = angular_speed * WHEEL_RADIUS
            output_data.append(f"{i},{angular_speed:.2f},{linear_speed:.2f}")

            encoder_counts[i] = 0
            last_encoder_time[i] = current_time

    print(";".join(output_data))  # Jangan gunakan sys.stdout.flush()

speed_timer = Timer(-1)
speed_timer.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: calculate_speeds())

# **Fungsi untuk Mengontrol Gerakan**
def forward():
    led.value(1)
    set_all_motors_speed(40)
    time.sleep(10)
    stop()

def backward():
    led.value(1)
    set_all_motors_speed(-40)
    time.sleep(1)
    stop()

def strafe_right():
    led.value(1)
    set_motor_speed_with_feedback(0, 40)
    set_motor_speed_with_feedback(1, -40)
    set_motor_speed_with_feedback(2, -40)
    set_motor_speed_with_feedback(3, 40)
    time.sleep(1)
    stop()

def strafe_left():
    led.value(1)
    set_motor_speed_with_feedback(0, -40)
    set_motor_speed_with_feedback(1, 40)
    set_motor_speed_with_feedback(2, 40)
    set_motor_speed_with_feedback(3, -40)
    time.sleep(1)
    stop()

def stop():
    set_all_motors_speed(0)
    led.value(0)

while True:
    if input_data:
        if input_data == "maju":
            forward()
        elif input_data == "mundur":
            backward()
        elif input_data == "kanan":
            strafe_right()
        elif input_data == "kiri":
            strafe_left()
        elif input_data == "stop":
            stop()
        
        input_data = None

