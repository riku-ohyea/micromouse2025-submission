from microbit import *
from keyes_Bit_Car_Driver import *
bitCar = Bit_Car_Driver()
val_LL = 0

bitCar.headlights(0, 0, 0)

bitCar.motorL(1,70)
bitCar.motorR(1,70)
while True:
    if button_a.is_pressed():
        break
    #if pin2.is_touched() and button_b.is_pressed():
        #print("obstacle forward")
        #bitCar.motorL(1,70)
        #bitCar.motorR(0,70)
        #while pin2.is_touched() or button_b.is_pressed():
            #sleep(100)
        #bitCar.motorR(1,70)

    elif pin2.is_touched():
        print("obstacle left")
        bitCar.motorR(1,0)
        while pin2.is_touched():
            sleep(100)
        bitCar.motorR(1,70)

    elif button_b.is_pressed():
        print("obstacle right")
        bitCar.motorL(1,0)
        while button_b.is_pressed():
            sleep(100)
        bitCar.motorL(1,70)

    sleep(200)
bitCar.motorL(1,0)
bitCar.motorR(1,0)
