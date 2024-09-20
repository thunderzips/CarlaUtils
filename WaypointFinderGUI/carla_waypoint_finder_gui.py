import carla
import math
import numpy as np
import pygame
import pygame.font
import json
import os


image_size = 2000

def setup_camera(world, height=image_size, width=image_size, camera_height=300):
    '''
    Used to initialize the camera actor in Carla

    :param world: Carla's world object
    :param height: Height of the image
    :param width: Width of the image
    :param camera_height: Height at whoch the camera is placed at
    :return camera: Carla sensor object of the created camera
    '''
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_blueprint.set_attribute('image_size_x', str(width))
    camera_blueprint.set_attribute('image_size_y', str(height))
    camera_blueprint.set_attribute('fov', '90')

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0]

    camera_transform = carla.Transform(
        carla.Location(x=0, y=0, z=camera_height),
        # carla.Location(x=spawn_point.location.x, y=spawn_point.location.y, z=camera_height),
        carla.Rotation(pitch=-90)
    )
    camera = world.spawn_actor(camera_blueprint, camera_transform)
    
    return camera


def get_camera_world_view(pixel_cor: np.array, K_inverse: np.array, c2w: np.array, desired_z: float, sensor_location:np.array):
    '''
    Converts the pixel to a point on the carla world

    :param pixel_cor: array [[1,1][px,py]]
    :param K_inverse: inverse of the camera intrinsic matrix
    :param c2w: Transformation matrix from camera (3d) to world
    :param desired_z: The desired elevation at which the point is to be assumed
    :param sensor_location: [x,y,z] of the camera

    :return: Pixel to 3D world transformation matrix
    '''
    n = pixel_cor.shape[0]
    v = np.concatenate((pixel_cor, np.array([[1.] * n]).T),axis = 1).T 
    F = np.array([[ 0,  1,  0 ], [ 0,  0, -1 ], [ 1,  0,  0 ]], dtype=np.float64)
    F = np.linalg.inv(F)
    to_camera = np.matmul(K_inverse, v)
    to_camera = np.matmul(F, to_camera)
    to_camera = np.concatenate((to_camera, np.array([[1] * n])))
    to_world = np.matmul(c2w, to_camera).T

    vec = to_world[:,:3] - sensor_location.reshape(1,3)
    vec = vec / vec[:,2].reshape(-1,1) * (to_world[:,2].reshape(-1,1) - desired_z)

    return to_world[:,:3] - vec


def zoom_transform(image_w, image_h, zoom, zoom_center, px, py):
    '''
    Transformation of pixel point while zooming and panning
    
    :param image_w: Width of the camera image
    :param image_h: Height of the camera image
    :param zoom: Zoomed scale
    :param zoom_center: Center of the panned image

    :param px: x location of the pixel
    :param py: y location of the pixel

    :return: transformed x and y values of the pixel
    '''

    px = image_w//2 + (px-image_w//2)/zoom + (zoom_center[0]-image_w//2)
    py = image_h//2 + (py-image_h//2)/zoom + (zoom_center[1]-image_h//2)

    return px, py


def get_camera_intrinsic(image_w: int, image_h: int, fov: int):
    '''
    Computes the intrinsic matrix K of the camera

    :param image_w: Width of the camera image
    :param image_h: Height of the camera image
    :param fov: Feild of vision of the camera image

    :return: Intrinsic matric of the camera
    '''
    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0
    return K

def distance_func(x, waypoint):
    '''
    Computes the distance between the waypoint and the vehicle

    :param x: Tuple of the vehicle id and the vehicle object
    :param waypoint: Waypoint object

    :return: Distance between the waypoint and the vehicle
    '''
    return math.sqrt((x[1].get_location().x - waypoint.transform.location.x)**2 + (x[1].get_location().y - waypoint.transform.location.y)**2)

def display_interactive_bev(world, image_array, camera, path_to_json):
    '''
    Displays the BEV and performs actions based on used inputs (mouse and keyboard).

    :param world: Carla's world object
    :param image_array: Image from the camera sensor
    :param camera: Carla's camera sensor object
    :param path_to_json: Provide the path to the .json file where the data is to be saved
    '''
    pygame.init()
    pygame.font.init()
    font = pygame.font.Font(None, 36)
    
    height, width = image_array.shape[:2]
    display = pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA BEV - Click to get world coordinates")

    map = world.get_map()

    zoom = 1
    zoom_center = [height//2,width//2]

    pygame_image = pygame.surfarray.make_surface(image_array)

    data_to_write = {"ego":None,"npc":[],"waypoints":[]}

    actor_ids = {"npc":0,"waypoints":0}

    mode = "None"

    bplib = world.get_blueprint_library()
    vehicle_bp = bplib.filter('vehicle.bmw.*')[0]
    npc_vehicles = {} 

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    x, y = event.pos

                    sensor_location = np.array([camera.get_transform().location.x,camera.get_transform().location.y,camera.get_transform().location.z])

                    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
                    c2w = np.linalg.inv(world_2_camera)

                    image_w = int(camera.attributes['image_size_x'])
                    image_h = int(camera.attributes['image_size_y'])
                    fov = float(camera.attributes['fov'])

                    x, y = zoom_transform(image_w, image_h, zoom, zoom_center, x, y)


                    K = get_camera_intrinsic(image_w, image_h, fov)
                    
                    K_inverse = np.linalg.inv(K)

                    world_point = get_camera_world_view(np.array([[1,1.],[x,y]]), K_inverse, c2w, 0, sensor_location)
                    sensor_location = np.array([camera.get_transform().location.x,camera.get_transform().location.y,camera.get_transform().location.z])

                    world_x = -world_point[1][1] + sensor_location[1]
                    world_y = -world_point[1][0] + sensor_location[0]

                    waypoint01 = map.get_waypoint(carla.Location(x=world_x,y=world_y,z=0),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

                    print("\n#####################")
                    print(f"Clicked pixel corresponds to world point: \n{world_point[1][:2]}")
                    print(f"Closest waypoint for the clicked point: \n{waypoint01.transform.location.x, waypoint01.transform.location.y, waypoint01.transform.rotation.yaw}")
                    print("#####################\n")

                    waypoint_list = {"id": None, "x": waypoint01.transform.location.x, "y": waypoint01.transform.location.y, "yaw": waypoint01.transform.rotation.yaw}

                    if not mode is "None":
                        if mode is "ego":
                            waypoint_list.pop("id")
                            data_to_write[mode] = waypoint_list
                        
                        if mode is "npc" or mode is "waypoints":
                            waypoint_list["id"] = actor_ids[mode]
                            actor_ids[mode] += 1
                            data_to_write[mode].append(waypoint_list)

                            if mode is "npc":
                                npc_vehicles[waypoint_list["id"]] = world.spawn_actor(vehicle_bp,carla.Transform(carla.Location(x=waypoint01.transform.location.x,y=waypoint01.transform.location.y,z=waypoint01.transform.location.z+2),waypoint01.transform.rotation))

                        if mode is "del":
                            # sorted_npcs = sorted(npc_vehicles.items(), key=cmp_to_key(compare))
                            sorted_npcs = sorted(npc_vehicles.items(), key=lambda x: distance_func(x, waypoint01))
                            # print(sorted_npcs)
                            if len(sorted_npcs) > 0:
                                if distance_func(sorted_npcs[0], waypoint01) < 2:
                                    for npc_d in data_to_write["npc"]:
                                        if npc_d["id"] == sorted_npcs[0][0]:
                                            data_to_write["npc"].remove(npc_d)
                                    npc_vehicles[sorted_npcs[0][0]].destroy()
                                    npc_vehicles.pop(sorted_npcs[0][0])

                    world.debug.draw_string(carla.Location(x=world_x,y=world_y), 'o', life_time=10, persistent_lines=False)
                    
                    world.debug.draw_string(waypoint01.transform.location, 'x', life_time=100, persistent_lines=False)

                    pygame.display.flip()
            
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFTBRACKET:
                    if zoom>1:
                        zoom -= 1
                    print(f"zoom = {zoom}")
                if event.key == pygame.K_RIGHTBRACKET  :
                    if zoom<5:
                        zoom += 1
                    print(f"zoom = {zoom}")

                if event.key == pygame.K_UP  :
                    zoom_center[1] -= 100
                if event.key == pygame.K_DOWN :
                    zoom_center[1] += 100
                if event.key == pygame.K_LEFT  :
                    zoom_center[0] -= 100
                if event.key == pygame.K_RIGHT  :
                    zoom_center[0] += 100

                if event.key == pygame.K_r  :
                    zoom_center = [height//2,width//2]
                    zoom = 1
                
                if event.key == pygame.K_e  :
                    '''
                    TODO: Need to implement direction choosing 
                    '''
                    mode = "ego"
                    print(f"mode is set to {mode}, press q to quit this mode")
                
                if event.key == pygame.K_o  :
                    '''
                    TODO: Need to choose speed also
                    '''
                    mode = "npc"
                    print(f"mode is set to {mode}, press q to quit this mode")

                if event.key == pygame.K_d  :
                    '''
                    '''
                    mode = "del"
                    print(f"mode is set to {mode}, press q to quit this mode")


                if event.key == pygame.K_w  :
                    '''
                    TODO: Need to generate equally spaced waypoints between the points
                    '''
                    mode = "waypoints"
                    print(f"mode is set to {mode}, press q to quit this mode")

                if event.key == pygame.K_q  :
                    mode = "None"
                    print(f"mode is set to {mode}")

                if event.key == pygame.K_s  :
                    print(f"Saved the data at {path_to_json} ")
                    with open(path_to_json, 'w', encoding='utf-8') as f:
                        json.dump(data_to_write, f, ensure_ascii=False, indent=4)
                    
        """
        Display the image & zooming
        """

        wnd_w, wnd_h = display.get_size()
        zoom_size = (round(wnd_w/zoom), round(wnd_h/zoom))

        zoom_area = pygame.Rect(0, 0, *zoom_size)
        zoom_area.center = zoom_center

        zoom_surf = pygame.Surface(zoom_area.size)
        zoom_surf.blit(pygame_image, (0, 0), zoom_area)

        zoom_surf = pygame.transform.scale(zoom_surf, (wnd_w, wnd_h))

        display.blit(zoom_surf, (0, 0))

        pygame_image = pygame.surfarray.make_surface(image_array)
        pygame.display.flip()

    pygame.quit()

    for npc in npc_vehicles.values():
        npc.destroy()

def bev_callback(image, image_array):
    '''
    Converts the image into a easily processable format
    '''
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Keep only RGB channels
    array = array[:, :, ::-1]  # Convert BGR to RGB
    image_array[:] = array

def capture_image(camera, height=image_size, width=image_size):
    '''
    Gets the camera image
    '''
    image_array = np.zeros((height, width, 3), dtype=np.uint8)
    camera.listen(lambda image: bev_callback(image, image_array))
    camera.get_world().tick()
    camera.get_world().wait_for_tick()
    return image_array

def main():

    folder_to_json = "."

    file_prefix = "scene_"
    file_suffix = ".json"

    max_scene_num = 0
    for file in os.listdir(folder_to_json):
        if file.endswith(file_suffix) and file.startswith(file_prefix):
            curr_scene_num = int(file[len(file_prefix):len(file)-len(file_suffix)])
            max_scene_num = max(max_scene_num, curr_scene_num)
    
    path_to_json = os.path.join(folder_to_json, file_prefix+str(max_scene_num+1)+file_suffix)
    print(path_to_json)

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    # world = client.load_world('Town02')
    world = client.get_world()

    try:
        camera = setup_camera(world)
        image_array = capture_image(camera)
        display_interactive_bev(world, image_array, camera, path_to_json)
    finally:
        if 'camera' in locals():
            camera.destroy()

if __name__ == '__main__':
    main()