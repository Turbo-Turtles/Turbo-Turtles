import pynput

pressed_key = "None" 

def on_press(key):
    global pressed_key
    str = format(key).strip("'")
    pressed_key = str
    print(pressed_key)

def on_release(key):
    global pressed_key
    pressed_key = "None"
    print(pressed_key)
    
with pynput.keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()



while(True):
    print(1)
    if pressed_key == None:
        print("None")
    elif pressed_key == "q":
        break
    else:
        print(pressed_key)
    