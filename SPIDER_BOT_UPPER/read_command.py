import serial
import time
from pynput import keyboard

def direction():
    with keyboard.Events() as events:
        # Block for as much as possible
        event = events.get(1e6)
        if event.key == keyboard.KeyCode.from_char('w'):
            print("w")
            return 1
        elif event.key == keyboard.KeyCode.from_char('a'):
            print("a")
            return 2
        elif event.key == keyboard.KeyCode.from_char('s'):
            print("s")
            return 3
        elif event.key == keyboard.KeyCode.from_char('d'):
            print("d")
            return 4
        elif event.key == keyboard.KeyCode.from_char('q'):   
            return -1
        else:
            return 0


port = serial.Serial()
port.port = "/dev/ttyUSB0"
port.baudrate = 9600
port.bytesize = 8
port.parity = 'N'
port.stopbits = 1

if __name__=="__main__":
    port.open()
    
    while True:
        packet = bytearray()
        dir_val = direction()
        if dir_val  == -1:
            break
        packet.append(dir_val)
        time.sleep(0.05)
        port.write(packet)
        
    port.close()