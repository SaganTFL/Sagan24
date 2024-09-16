from time import sleep

from MainSystem import AUVController

controller = AUVController('udpin:0.0.0.0:14550')
controller.arm_vehicle()
controller.set_mode("STABILIZE")
i=1
while i<1000:

    controller.go_to_vehicle(0)

    # controller.go_to_vehicle_raw(2000,0,500,0,0,0,0,1)
    sleep(1)
    i+=i+1

controller.go_to_vehicle(0)

# controller.go_to_vehicle_raw(0,0,500,0,0,0,0,1)
# if you just run 1 times a move command it will move like 2-3 secs
# if you run 2 command in the same time (like go full straight and go full left) it won't work as you expect.
# if you want to go diagonal you must give 2 or more axis motor data with in the same command
# controller.arm_vehicle()
# while True:
#     controller.set_servo(11,2000,1)
    #sleep(5)
#controller.disarm_vehicle()
#while True:
#    controller.go_full_straight()
#controller.go_full_left()
#while True:
# controller.go_to_vehicle(0,0,1000,200,0,200,200,1)