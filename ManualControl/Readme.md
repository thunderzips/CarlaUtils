# Manual Control with Keyboard and Joystick
This helps to seamlessly use manual control of vehicle via a keyboard or joystick.

## Usage

import the manual_control file.
Create a controller object from manual_controller.controller(vehicle)
Pass the carla vehicle from your main script to the constructor
```
controller = manual_control.controller(vehicle, "joystick") # or "keyboard"
```

put `.perform_vehicle_control()` in your main loop. It returns the `done` value. Which tells if the newly created pygame window is closed.

```
while not done:
    done = controller.perform_vehicle_control()
```

## Controls:
### Keyboard

    w -> forward throttle
    a,d -> steering
    s -> reverse
    space -> brake

### Joystick (More precise control on throttle and steering)
For Xbox-like controller

    LT -> throttle
    RT -> reverse
    Left joystick -> steering
    B button -> brake
