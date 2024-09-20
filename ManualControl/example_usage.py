import carla
import random

import manual_control

def main():

    ############################
    # Carla client and world setup
    client = carla.Client('localhost',2000)
    client.set_timeout(2)
    world = client.load_world('Town02')
    bplib = world.get_blueprint_library()
    vehicle_bp = random.choice(bplib.filter('vehicle.bmw.*'))
    spawn_point = carla.Transform(carla.Location(x=125,y=302,z=5),carla.Rotation(yaw=180))

    vehicle = world.spawn_actor(vehicle_bp,spawn_point)
    ############################

    # Rest of your code here ..

    # Create the controller object
    controller = manual_control.controller(vehicle, "joystick") # "joystick" or "keyboard"

    done = False
    while not done:
        done = controller.perform_vehicle_control()

if __name__ == '__main__':
    main()