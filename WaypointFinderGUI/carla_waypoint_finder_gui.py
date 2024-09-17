import carla
import math
import numpy as np
import pygame
import pygame.font

def setup_camera(world, height=1000, width=1000, camera_height=200):
    '''
    Used to initialize the camera actor in Carla
    Inputs:
        :param world: Carla world
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

def display_interactive_bev(world, image_array, camera):
    pygame.init()
    pygame.font.init()
    font = pygame.font.Font(None, 36)
    
    height, width = image_array.shape[:2]
    display = pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA BEV - Click to get world coordinates")

    map = world.get_map()

    zoom = 1
    zoom_center = [height//2,width//2]

    # spawn_points = world.get_map().get_spawn_points()
    # waypoint01 = map.get_waypoint(carla.Location(x=0,y=0,z=0),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

    pygame_image = pygame.surfarray.make_surface(image_array)

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

                    print("#####################")
                    print(f"Clicked pixel corresponds to world point: \n{world_point[1][:2]}")
                    print(f"Closest waypoint for the clicked point: \n{waypoint01.transform.location.x, waypoint01.transform.location.y}")
                    print("#####################\n\n")


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

        # Display the image

        """
        Pygame zoom
        """

        wnd_w, wnd_h = display.get_size()
        zoom_size = (round(wnd_w/zoom), round(wnd_h/zoom))

        zoom_area = pygame.Rect(0, 0, *zoom_size)
        zoom_area.center = zoom_center
        # zoom_area.center = (zoom_center[0], zoom_center[1])
        # zoom_area.center = (pos_x, pos_y)


        zoom_surf = pygame.Surface(zoom_area.size)
        zoom_surf.blit(pygame_image, (0, 0), zoom_area)

        zoom_surf = pygame.transform.scale(zoom_surf, (wnd_w, wnd_h))

        display.blit(zoom_surf, (0, 0))


        pygame_image = pygame.surfarray.make_surface(image_array)
        # display.blit(pygame_image, (0, 0))
        pygame.display.flip()

    pygame.quit()

def bev_callback(image, image_array):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Keep only RGB channels
    array = array[:, :, ::-1]  # Convert BGR to RGB
    image_array[:] = array


def capture_image(camera, height=1000, width=1000):
    image_array = np.zeros((height, width, 3), dtype=np.uint8)
    camera.listen(lambda image: bev_callback(image, image_array))
    camera.get_world().tick()
    camera.get_world().wait_for_tick()
    return image_array


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    # world = client.load_world('Town06')
    world = client.get_world()

    try:
        camera = setup_camera(world)
        image_array = capture_image(camera)
        display_interactive_bev(world, image_array, camera)
    finally:
        if 'camera' in locals():
            camera.destroy()

if __name__ == '__main__':
    main()