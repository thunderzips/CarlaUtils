import pygame
import carla

class controller:
    def __init__(self, vehicle, mode = "keyboard"):
        '''
        This class is used to control the vehicle manually using either keyboard or joystick.
        
        :param vehicle: carla.Vehicle
        :param mode: str, default = "keyboard"
            "keyboard" or "joystick"
        '''
        self.vehicle = vehicle
        self.mode = mode 

        pygame.init()
        self.screen = pygame.display.set_mode((300, 300))

        for _ in range(50):
            self.vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0,brake = 0, reverse = 0))

        if mode == "keyboard":
            self.keyboard_control = keyboard_control(vehicle)
        elif mode == "joystick":
            self.joystick_control = joystick_control(vehicle)
    
    def perform_vehicle_control(self):
        '''
        This function is used to control the vehicle manually using either keyboard or joystick.

        :return: bool
            True if quit, False otherwise
        '''
        
        done = False

        self.screen.fill((0, 0, 0))
        pygame.display.flip()

        if self.mode == "keyboard":
            done = self.keyboard_control.perform_vehicle_control()
        elif self.mode == "joystick":
            done = self.joystick_control.perform_vehicle_control()

        if done:
            pygame.quit()
        
        return done        

class joystick_control:
    def __init__(self, vehicle) -> None:
        """
        This class is used to control the vehicle using joystick.

        :param vehicle: carla.Vehicle
        """

        self.vehicle = vehicle

        self.joysticks = {}

        self.axis_map = {"lud":1,"llr":0,"rud":3,"rlr":2,"lt":5,"rt":4} # left up down, left left right, right up down, right left right, left trigger, right trigger
        self.buttons_map = {"A":0,"B":1,"X":3,"Y":4,"LB":6,"RB":7,"LT":8,"RT":9,"START":11}
        self.controls = {"axis":{i:0 for i in self.axis_map},"button":{i:0 for i in self.buttons_map}}
        self.done = False
    
    def get_input(self):
        """
        This function is used to get the input from the joystick.

        :return: dict of controls
            controls["axis"] = {"lud":float,"llr":float,"rud":float,"rlr":float,"lt":float,"rt":float}
                left up down, left left right, right up down, right left right, left trigger, right trigger

            controls["button"] = {"A":bool,"B":bool,"X":bool,"Y":bool,"LB":bool,"RB":bool,"LT":bool,"RT":bool,"START":bool}
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True

            if event.type == pygame.JOYDEVICEADDED:
                joy = pygame.joystick.Joystick(event.device_index)
                self.joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del self.joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

            for joystick in self.joysticks.values():
                jid = joystick.get_instance_id()
                name = joystick.get_name()

                axes = joystick.get_numaxes()

                for i in range(axes):
                    self.controls["axis"]["lud"] = joystick.get_axis(self.axis_map["lud"])
                    self.controls["axis"]["llr"] = joystick.get_axis(self.axis_map["llr"])
                    self.controls["axis"]["rud"] = joystick.get_axis(self.axis_map["rud"])
                    self.controls["axis"]["rlr"] = joystick.get_axis(self.axis_map["rlr"])
                    self.controls["axis"]["lt"] = joystick.get_axis(self.axis_map["lt"])
                    self.controls["axis"]["rt"] = joystick.get_axis(self.axis_map["rt"])
                
                buttons = joystick.get_numbuttons()

                for i in range(buttons):
                    self.controls["button"]["A"] = joystick.get_button(self.buttons_map["A"])
                    self.controls["button"]["B"] = joystick.get_button(self.buttons_map["B"])
                    self.controls["button"]["X"] = joystick.get_button(self.buttons_map["X"])
                    self.controls["button"]["Y"] = joystick.get_button(self.buttons_map["Y"])
                    self.controls["button"]["LB"] = joystick.get_button(self.buttons_map["LB"])
                    self.controls["button"]["RB"] = joystick.get_button(self.buttons_map["RB"])
                    self.controls["button"]["LT"] = joystick.get_button(self.buttons_map["LT"])
                    self.controls["button"]["RT"] = joystick.get_button(self.buttons_map["RT"])
                    self.controls["button"]["START"] = joystick.get_button(self.buttons_map["START"])

        return self.controls

    def perform_vehicle_control(self):
        """
        This function is used to control the vehicle using joystick.

        :return: bool
            True if quit, False otherwise
        """

        controls = self.get_input()

        rev = False
        throt = 0.0
        steer = 0.0
        brake = 0.0

        if controls["axis"]["lt"] > -0.8:
            rev = False
            throt = (controls["axis"]["lt"]+1)/2

        if abs(controls["axis"]["llr"]) > 0.1:
            steer = controls["axis"]["llr"]
        
        if controls["button"]["B"]:
            brake = 0.8

        if controls["axis"]["rt"]>-0.8:
            rev = True
            throt = (controls["axis"]["rt"]+1)/2
        
        self.vehicle.apply_control(carla.VehicleControl(throttle=throt, steer=steer,brake = brake, reverse = rev))

        return self.done

class keyboard_control:

    def __init__(self, vehicle) -> None:
        """
        This class is used to control the vehicle using keyboard.
        
        :param vehicle: carla.Vehicle
        """

        self.vehicle = vehicle
        self.done = False
        self.keys = set()

    def get_input(self, pressed_keys):
        """
        This function is used to get the input from the keyboard.

        :param pressed_keys: set
            set of keys pressed previously
        
        :return: set
            set of keys pressed currently
        """
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    pressed_keys.add('w')
                elif event.key == pygame.K_a:
                    pressed_keys.add('a')
                elif event.key == pygame.K_s:
                    pressed_keys.add('s')
                elif event.key == pygame.K_d:
                    pressed_keys.add('d')
                elif event.key == pygame.K_SPACE:
                    pressed_keys.add('space')
            
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    try:
                        pressed_keys.remove('w')
                    except:
                        pass
                elif event.key == pygame.K_a:
                    try:
                        pressed_keys.remove('a')
                    except:
                        pass
                elif event.key == pygame.K_s:
                    try:
                        pressed_keys.remove('s')
                    except:
                        pass
                elif event.key == pygame.K_d:
                    try:
                        pressed_keys.remove('d')
                    except:
                        pass
                elif event.key == pygame.K_SPACE:
                    try:
                        pressed_keys.remove('space')
                    except:
                        pass

        return pressed_keys
    
    def perform_vehicle_control(self):
        """
        This function is used to control the vehicle using keyboard.
        
        :return: bool
            True if quit, False otherwise
        """

        self.keys = self.get_input(self.keys)
        keys = self.keys

        rev = False
        throt = 0.0
        steer = 0.0
        brake = 0.0

        if 'w' in keys:
            rev = False
            throt = 0.8

        if 'a' in keys:
            steer = -0.5
        
        if 's' in keys:
            rev = True
            throt = 0.8

        if 'd' in keys:
            steer = 0.5
            
        if 'space' in keys:
            brake = 1.0

        self.vehicle.apply_control(carla.VehicleControl(throttle=throt, steer=steer,brake = brake, reverse = rev))
        return self.done

