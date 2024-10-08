import carla
import numpy as np
import random
import time
import pickle

import cv2

import os
import glob


class CarlaEnv:
    def __init__(self):

        self.save_camera_imgs = True
        self.display_wpts = True

        self.datapath = "data/TOWN01"

        self.data = {"lidar": [], "camera": {}, "waypoints": {}, "obstacles": {}, "ego": {}}

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.load_world("TOWN01")
        settings = world.get_settings()
        settings.synchronous_mode = True
        FPS = 1/2
        self.FPS = FPS
        settings.fixed_delta_seconds = FPS  # FPS = 1/0.05 = 20
        world.apply_settings(settings)
        world.tick()
        self.world = world

        self.spawn_points = world.get_map().get_spawn_points()

        EGOLOC = random.choice(self.spawn_points)

        self.tm = client.get_trafficmanager()
        self.tm_port = self.tm.get_port()
        self.tm.set_synchronous_mode(True)
        # tm.global_percentage_speed_difference(80)

        egobp = world.get_blueprint_library().find('vehicle.tesla.model3')
        egobp.set_attribute('role_name', 'ego')
        ego = world.spawn_actor(egobp, EGOLOC)
        self.ego = ego

        dummy_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        dummy_transform = carla.Transform(carla.Location(
            x=-1, z=31), carla.Rotation(pitch=0.0))
        dummy = world.spawn_actor(dummy_bp, dummy_transform, attach_to=ego,
                                    attachment_type=carla.AttachmentType.SpringArm)
        dummy.listen(lambda image: self.dummy_function(image))
        spectator = world.get_spectator()
        self.dummy = dummy

        self.spectator = spectator

        spectator.set_transform(dummy.get_transform())

        self.spawn_lidar()
        self.spawn_cameras()

        ego.set_autopilot(True,self.tm_port)

        self.spawn_traffic(500)

        files = glob.glob(self.datapath+'/*')
        for f in files:
            os.remove(f)

        with open(self.datapath+'/config.txt', 'w') as file:
            file.write(f"TOWN : TOWN01\n")
            file.write(f"Ego Vehicle location : {EGOLOC}\n")
            file.write(f"Number of obstacles : {len(self.npc_vehicles)}\n")


    def spawn_lidar(self):
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('channels', str(16))
        # Set the fps of simulator same as this
        lidar_bp.set_attribute('rotation_frequency', str(self.FPS))
        lidar_bp.set_attribute('range', str(80))
        lidar_bp.set_attribute('lower_fov', str(-15))
        lidar_bp.set_attribute('upper_fov', str(15))
        lidar_bp.set_attribute('points_per_second', str(300000))
        # lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
        lidar_location = carla.Location(0, 0, 1.75)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar_sen = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego)
        lidar_sen.listen(
            lambda point_cloud: self.process_point_cloud(point_cloud))
        self.lidar_sen = lidar_sen
    
    def spawn_cameras(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x','800')
        camera_bp.set_attribute('image_size_y','800')
        camera_bp.set_attribute('fov','90')

        cameras = []
        
        for id in range(6):
            
            camera_transform = carla.Transform(carla.Location(x=0,z=2),carla.Rotation(yaw=id*60))
            cameras.append(self.world.spawn_actor(camera_bp,camera_transform, attach_to=self.ego))

        cameras[0].listen(lambda image: self.process_camera(image, id=0))
        cameras[1].listen(lambda image: self.process_camera(image, id=1))
        cameras[2].listen(lambda image: self.process_camera(image, id=2))
        cameras[3].listen(lambda image: self.process_camera(image, id=3))
        cameras[4].listen(lambda image: self.process_camera(image, id=4))
        cameras[5].listen(lambda image: self.process_camera(image, id=5))
        
        self.cameras = cameras

    def spawn_traffic(self, amount=500):

        self.npc_vehicles = []

        for _ in range(amount):
            try:
                spawn_point = random.choice(self.spawn_points)
                vehicle_bp = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                self.npc_vehicles.append(vehicle)
                vehicle.set_autopilot(True, self.tm_port)
            except:
                pass
        
        # for actor in self.npc_vehicles:
            # actor.set_autopilot(True, self.tm_port)

    def run_step(self):
        self.world.tick()
        self.spectator.set_transform(self.dummy.get_transform())
        
        self.obstacle_data()
        self.ego_data()
        self.lane_data()
    
    def save_data(self, i):
        
        with open(self.datapath+f'/{i}.pkl', 'wb') as file: 
            pickle.dump(self.data, file) 

    def lane_data(self):

        v_x = self.ego.get_location().x
        v_y = self.ego.get_location().y

        waypoint01 = self.world.get_map().get_waypoint(carla.Location(x=float(v_x),y=float(v_y),z=0),project_to_road=True, lane_type=(carla.LaneType.Driving))

        waypoints = waypoint01.next_until_lane_end(1.0)
        prev_waypoints = waypoint01.previous_until_lane_start(1.0)

        for waypoint in waypoints:
            if self.display_wpts:
                self.world.debug.draw_string(waypoint.transform.location, 'w', life_time=0.05, color=carla.Color(255,0,0))
        
        for waypoint in prev_waypoints:
            if self.display_wpts:
                self.world.debug.draw_string(waypoint.transform.location, 'pw', life_time=0.05, color=carla.Color(255,0,0))


        #####################
        ### FORWARD DIRECTION
        #####################



        ###########################
        ### LEFT LANE WAYPOINTS

        try:
            opposite_waypoints = []
            for waypoint in waypoints:
                opposite_waypoint = waypoint.get_left_lane()
                opposite_waypoints.append(opposite_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(opposite_waypoint.transform.location, 'o', life_time=0.05, color=carla.Color(255,0,0))
        except:
            opposite_waypoints = None
        
        
        ######################
        ### LEFT WAYPOINTS
        
        try:
            left_waypoints = []
            for i, waypoint in enumerate(waypoints):
                opposite_waypoint = opposite_waypoints[i]
                left_waypoint = [(waypoint.transform.location.x+opposite_waypoint.transform.location.x)/2, (waypoint.transform.location.y+opposite_waypoint.transform.location.y)/2, (waypoint.transform.location.z+opposite_waypoint.transform.location.z)/2]
                left_waypoint = carla.Waypoint(carla.Location(x=left_waypoint[0], y=left_waypoint[1], z=left_waypoint[2]))
                left_waypoints.append(left_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(left_waypoint.transform.location, 'l', life_time=0.05, color=carla.Color(255,0,0))
        except:
            left_waypoints = None


        ###################
        ### RIGHT WAYPOINTS
        try:
            right_waypoints = []
            for waypoint in waypoints:
                right_waypoint = waypoint.get_right_lane()
                right_waypoints.append(right_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(right_waypoint.transform.location, 'r', life_time=0.05, color=carla.Color(255,0,0))
        except:
            right_waypoints = None



        ######################
        ### BACKWARD DIRECTION
        ######################


        ################################
        ### BACKWARD LEFT LANE WAYPOINTS
        try:
            prev_opposite_waypoints = []
            for waypoint in prev_waypoints:
                opposite_waypoint = waypoint.get_left_lane()
                prev_opposite_waypoints.append(opposite_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(opposite_waypoint.transform.location, 'po', life_time=0.05, color=carla.Color(255,0,0))
        except:
            prev_opposite_waypoints = None

        ############################
        ### BACKWARD RIGHT WAYPOINTS
        try:
            prev_right_waypoints = []
            for waypoint in prev_waypoints:
                right_waypoint = waypoint.get_right_lane()
                prev_right_waypoints.append(right_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(right_waypoint.transform.location, 'pr', life_time=0.05, color=carla.Color(255,0,0))
        except:
            prev_right_waypoints = None

        ###########################
        ### BACKWARD LEFT WAYPOINTS
        try:
            prev_left_waypoints = []
            for i, waypoint in enumerate(prev_waypoints):
                opposite_waypoint = prev_opposite_waypoints[i]
                left_waypoint = [(waypoint.transform.location.x+opposite_waypoint.transform.location.x)/2, (waypoint.transform.location.y+opposite_waypoint.transform.location.y)/2, (waypoint.transform.location.z+opposite_waypoint.transform.location.z)/2]
                left_waypoint = carla.Waypoint(carla.Location(x=left_waypoint[0], y=left_waypoint[1], z=left_waypoint[2]))
                prev_left_waypoints.append(left_waypoint)
                if self.display_wpts:
                    self.world.debug.draw_string(left_waypoint.transform.location, 'pl', life_time=0.05, color=carla.Color(255,0,0))
        except:
            prev_left_waypoints = None


        #########################
        ### SAVING WAYPOINTS DATA
        #########################

        waypoints_list = []
        for waypoint in waypoints:
            waypoint_location = waypoint.transform.location
            waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        prev_waypoints_list = []
        for waypoint in prev_waypoints:
            waypoint_location = waypoint.transform.location
            prev_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        opposite_waypoints_list = []
        if opposite_waypoints:
            for waypoint in opposite_waypoints:
                waypoint_location = waypoint.transform.location
                opposite_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        left_waypoints_list = []
        if left_waypoints:
            for waypoint in left_waypoints:
                waypoint_location = waypoint.transform.location
                left_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        right_waypoints_list = []
        if right_waypoints:
            for waypoint in right_waypoints:
                waypoint_location = waypoint.transform.location
                right_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        prev_opposite_waypoints_list = []
        if prev_opposite_waypoints:
            for waypoint in prev_opposite_waypoints:
                waypoint_location = waypoint.transform.location
                prev_opposite_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        prev_right_waypoints_list = []
        if prev_right_waypoints:
            for waypoint in prev_right_waypoints:
                waypoint_location = waypoint.transform.location
                prev_right_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        prev_left_waypoints_list = []
        if prev_left_waypoints:
            for waypoint in prev_left_waypoints:
                waypoint_location = waypoint.transform.location
                prev_left_waypoints_list.append([waypoint_location.x, waypoint_location.y, waypoint_location.z])

        self.data["waypoints"]["waypoints"] = waypoints_list
        self.data["waypoints"]["prev_waypoints"] = prev_waypoints_list
        self.data["waypoints"]["opposite_waypoints"] = opposite_waypoints_list
        self.data["waypoints"]["left_waypoints"] = left_waypoints_list
        self.data["waypoints"]["right_waypoints"] = right_waypoints_list
        self.data["waypoints"]["prev_opposite_waypoints"] = prev_opposite_waypoints_list
        self.data["waypoints"]["prev_right_waypoints"] = prev_right_waypoints_list
        self.data["waypoints"]["prev_left_waypoints"] = prev_left_waypoints_list

    def obstacle_data(self):
        obstacles = self.npc_vehicles
        obstacle_data = []
        for obstacle in obstacles:
            obstacle_transform = obstacle.get_transform()
            obstacle_location = obstacle_transform.location
            obstacle_velocity = obstacle.get_velocity()
            obstacle_data.append({
                "position": (obstacle_location.x, obstacle_location.y, obstacle_location.z),
                "velocity": (obstacle_velocity.x, obstacle_velocity.y, obstacle_velocity.z)
            })
        self.data["obstacles"] = obstacle_data

    def ego_data(self):
        ego_transform = self.ego.get_transform()
        ego_location = ego_transform.location
        ego_velocity = self.ego.get_velocity()
        self.data["ego"] = {
            "position": (ego_location.x, ego_location.y, ego_location.z),
            "velocity": (ego_velocity.x, ego_velocity.y, ego_velocity.z)
        }

    def process_camera(self, image, id):
        image = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        image = image.reshape((800,800,4))
        image = image[:, :, :3]

        if self.save_camera_imgs:
            cv2.imwrite(f'camera_view_{id}.jpg', image)

        self.data["camera"][id] = image


    def process_point_cloud(self, point_cloud_carla):

        pcd = np.copy(np.frombuffer(point_cloud_carla.raw_data,
                                    dtype=np.dtype("f4, f4, f4, f4, u4, u4")))
        pcd = np.array(pcd.tolist())

        # The 4th column is considered as intensity in ros, hence making it one
        pcd[:, 3] = 1
        # Flipping Y  | Carla works in in LHCS
        pcd[:, 1] = -pcd[:, 1]

        pcd_xyz = pcd[:, :3]
        
        # pcd_sem = pcd[:, 5].reshape(-1, 1)    # Semantic Information | Might be helpful later
        # pcd_intensity = pcd[:, 4].reshape(-1, 1)

        self.data["lidar"] = pcd_xyz

        # Do something with the point cloud

    def dummy_function(self, image):
        pass

def main():
    env = CarlaEnv()

    i = 0


    while True:
        env.run_step()
        env.save_data(i)

        i += 1

        # time.sleep(0.02)

if __name__ == "__main__":
    main()