from pynput import keyboard
def direction():
    while True:
        with keyboard.Events() as events:
            # Block for as much as possible
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('w'):
                print("w")
            elif event.key == keyboard.KeyCode.from_char('a'):
                print("a")
            elif event.key == keyboard.KeyCode.from_char('s'):
                print("s")
            elif event.key == keyboard.KeyCode.from_char('d'):
                print("d")
            elif event.key == keyboard.KeyCode.from_char('q'):   
                break
if __name__=="__main__":
    direction()

