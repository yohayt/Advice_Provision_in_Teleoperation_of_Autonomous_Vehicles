from queue import Queue
import math
import numpy as np
import time
from sympy import symbols, Eq, solve
REACTION_TIME_INITIALIZE = 1.0
import sys
import os
import glob
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class RecommendationSystem:

    def __init__(self, args, scenario, actors, hero, world_map, file_name, world, shouldShowSuggestions):
        self.world = world
        self.file_name = file_name
        self.args = args

        self.uniqueRunIdentifier = str(args.participantId) + '_' + str(args.scenarioId) + '_' + str(args.scenarioName)

        with open('./' + str(self.args.participantId) + '/data/TL_data_log_' + str(self.args.scenarioId) + ".csv", 'w+') as f:
            f.write("time,my_traffic_light_id,TL_x,TL_y,TL_z,TL_yaw,traffic_light_color,my_traffic_light_waypoint_lane_id,hero_speed,hero_x,hero_y,hero_z,hero_yaw\n")

        try:
            with open(self.file_name, 'r') as f:
                reaction_time = float(f.readline())
        except Exception as e:
            reaction_time = REACTION_TIME_INITIALIZE

        if not 0 <= reaction_time <= 2:
            reaction_time = REACTION_TIME_INITIALIZE

        self.scenario = scenario
        self.hero = hero
        self.actors = actors
        self.world_map = world_map
        self.reaction_time = reaction_time   # 1-10
        self.scenario_name = str(args.scenarioName)
        self.scenario_changed = False
        self.hero_lane = None
        self.hero_lane_id = None
        self.green_time_counter = 0
        self.yellow_time_counter = 0
        self.red_time_counter = 0
        self.hero_speed = None    # m/s
        self.hero_speed_kmh = None
        self.hero_waypoint = None
        self.distance_forward = None
        self.forward_car_speed = None
        self.changed_TL = None
        self.traffic_light_color = None
        self.shouldShowSuggestions = shouldShowSuggestions

        self.first_time_recommendation_shown = 0
        self.current_recommendation = "do nothing"
        self.recommendations_queue = Queue(20)
        self.recommendations_counter = {'showTurnLeftArrow': 0, "do nothing": 0, 'showSlowWarning': 0}
        self.frame_number = 0
        self.show_slow_counter = 0
        self.last_TL = None

    def write_to_fast_debug_log(self, text):
        debugFastOutputFile = open('./logs/debugFastOutput.log', 'w+')  # a+    %10
        debugFastOutputFile.write(str(text))
        debugFastOutputFile.close()

    def save_to_executing_table(self, advice, status):
        with open('./IO/advices_table_' + self.uniqueRunIdentifier + ".txt", 'a+') as f:
            f.write(str(time.time()) + "," + str(advice) + "," + str(status) + "\n")

    def  write_info_log(self, my_traffic_light_waypoint, my_traffic_light_waypoint_lane_id, my_traffic_light_id, traffic_light_color):
        hero_speed = self.hero_speed
        if my_traffic_light_waypoint is not None:
            TL_x = my_traffic_light_waypoint.transform.location.x
            TL_y = my_traffic_light_waypoint.transform.location.y
            TL_z = my_traffic_light_waypoint.transform.location.z
            TL_yaw = my_traffic_light_waypoint.transform.rotation.yaw
        else:
            TL_x = "None"
            TL_y = "None"
            TL_z = "None"
            TL_yaw = "None"
        hero_x = self.hero.get_transform().location.x
        hero_y = self.hero.get_transform().location.y
        hero_z = self.hero.get_transform().location.z
        hero_yaw = self.hero.get_transform().rotation.yaw
        with open('./' + str(self.args.participantId) + '/data/TL_data_log_' + str(self.args.scenarioId) + ".csv", 'a+') as f:
            f.write(str(time.time()) + ","
                    + str(my_traffic_light_id) + ","
                    + str(TL_x) + ","
                    + str(TL_y) + ","
                    + str(TL_z) + ","
                    + str(TL_yaw) + ","
                    + str(traffic_light_color) + ","
                    + str(my_traffic_light_waypoint_lane_id) + ","
                    + str(hero_speed) + ","
                    + str(hero_x) + ","
                    + str(hero_y) + ","
                    + str(hero_z) + ","
                    + str(hero_yaw)
                    + "\n")


    def SampleAndRecommend(self):

        last_recommendation = self.current_recommendation
        self.hero_speed = self.get_actor_speed(self.hero)
        self.hero_speed_kmh = self.hero_speed * 3.6
        self.hero_lane = self.world_map.get_waypoint(self.hero.get_transform().location, project_to_road=True)
        if self.hero_lane is not None:
            self.hero_lane_id = self.hero_lane.lane_id
        else:
            self.hero_lane_id = 100


        self.forward_car_speed = math.inf
        self.distance_forward = math.inf

        my_traffic_light_waypoint_lane_id = "None"
        traffic_light_color = "None"
        traffic_light_waypoints = "none TL"
        my_traffic_light = None
        traffic_lights = self.world.get_traffic_lights_from_waypoint(self.hero_lane, 250)
        distance_to_TL = math.inf
        how_many_TL = len(traffic_lights)
        my_traffic_light_waypoint = None
        if traffic_lights:
            for traffic_light_check in traffic_lights:   # can be more than 1 traffic light in a row
                if self.get_distance_between_vehicles(traffic_light_check, self.hero) < distance_to_TL:
                    distance_to_TL = self.get_distance_between_vehicles(traffic_light_check, self.hero)
                    traffic_light = traffic_light_check


            traffic_light_waypoints = traffic_light.get_affected_lane_waypoints()  # returns waypoints
            break1 = False
            for traffic_light_waypoint in traffic_light_waypoints:
                if not break1 and np.sign(traffic_light_waypoint.lane_id) == np.sign(self.hero_lane_id): #and my_traffic_light_waypoint.road_id == self.hero_lane.road_id:
                    my_traffic_light_waypoint = traffic_light_waypoint
                    my_traffic_light = traffic_light
                    my_traffic_light_waypoint_lane_id = my_traffic_light_waypoint.lane_id
                    break1 = True
        """
        Red
        Yellow
        Green
        """

        time_to_junction = -1
        distance = -math.inf
        is_behind = False
        hero_waypoint = self.world_map.get_waypoint(self.hero.get_location())
        at_tl = hero_waypoint.is_junction
        if my_traffic_light:
            traffic_light_color = str(my_traffic_light.get_state())
            distance = self.get_distance_between_vehicles(my_traffic_light, self.hero)
            time_to_junction = self.time_to_crash(self.hero_speed/3.6, 0, distance)
            is_behind = self.is_behind(my_traffic_light, self.hero)

            if distance <= 70 and (not self.scenario_changed) and self.scenario_name == "scenario1TL":
                if self.green_time_counter == 0:
                    self.green_time_counter = time.time()
                    my_traffic_light.set_state(carla.TrafficLightState.Green)
                elif time.time() - self.green_time_counter < 4.1:
                    my_traffic_light.set_state(carla.TrafficLightState.Green)
                elif self.yellow_time_counter == 0:
                    self.yellow_time_counter = time.time()
                    my_traffic_light.set_state(carla.TrafficLightState.Yellow)
                elif time.time() - self.yellow_time_counter < 1.5:
                    my_traffic_light.set_state(carla.TrafficLightState.Yellow)
                elif self.red_time_counter == 0 :
                    my_traffic_light.set_state(carla.TrafficLightState.Red)
                    self.red_time_counter = time.time()
                elif time.time() - self.red_time_counter < 5:
                    my_traffic_light.set_state(carla.TrafficLightState.Red)
                else:
                    self.scenario_changed = True

            elif distance <= 17 and (not self.scenario_changed) and self.scenario_name == "scenario2TL":
                if self.green_time_counter == 0:
                    self.green_time_counter = time.time()
                    self.changed_TL = my_traffic_light
                    my_traffic_light.set_state(carla.TrafficLightState.Green)
                elif time.time() - self.green_time_counter < 5:
                    self.changed_TL.set_state(carla.TrafficLightState.Green)
                else:
                    self.scenario_changed = True


            elif distance <= 60 and (not self.scenario_changed) and self.scenario_name == "scenario3TL":
                if self.green_time_counter == 0:
                    self.green_time_counter = time.time()
                    my_traffic_light.set_state(carla.TrafficLightState.Green)
                elif time.time() - self.green_time_counter < 4.5:
                    my_traffic_light.set_state(carla.TrafficLightState.Green)
                elif self.yellow_time_counter == 0:
                    self.yellow_time_counter = time.time()
                    my_traffic_light.set_state(carla.TrafficLightState.Yellow)
                elif time.time() - self.yellow_time_counter < 1.5:
                    my_traffic_light.set_state(carla.TrafficLightState.Yellow)
                elif self.red_time_counter == 0 :
                    my_traffic_light.set_state(carla.TrafficLightState.Red)
                    self.red_time_counter = time.time()
                elif time.time() - self.red_time_counter < 5:
                    my_traffic_light.set_state(carla.TrafficLightState.Red)
                else:
                    self.scenario_changed = True

        if (traffic_light_color == "Red" or traffic_light_color == "Yellow") and not is_behind and not at_tl:
            if 45 <= distance < 60 and self.hero_speed_kmh > 60:
                self.current_recommendation = "showSlowWarning"
                self.show_slow_counter += 1
            elif 35 <= distance < 45 and self.hero_speed_kmh > 45:
                self.current_recommendation = "showSlowWarning"
                self.show_slow_counter += 1
            elif 25 <= distance < 35 and self.hero_speed_kmh > 35:
                self.current_recommendation = "showSlowWarning"
                self.show_slow_counter += 1
            elif traffic_light_color == "Yellow" and distance < 10 and self.hero_speed_kmh > 20:
                self.current_recommendation = "do nothing"
                self.show_slow_counter = 0
            elif distance < 25 and self.hero_speed_kmh > 25:
                self.current_recommendation = "showSlowWarning"
                self.show_slow_counter += 1
            elif 5 < distance < 12 and self.hero_speed_kmh > 15:
                self.current_recommendation = "showSlowWarning"
                self.show_slow_counter += 1
            else:
                self.current_recommendation = "do nothing"
                self.show_slow_counter = 0
        else:
            self.current_recommendation = "do nothing"
            self.show_slow_counter = 0


        same_traffic_light = "none"
        if my_traffic_light is not None:
            my_traffic_light_id = my_traffic_light.get_opendrive_id()
        else:
            my_traffic_light_id = None

        if last_recommendation == "showSlowWarning" and self.current_recommendation != "showSlowWarning" and (time.time() - self.first_time_recommendation_shown > 0.1):
            if self.last_TL == my_traffic_light_id:
                same_traffic_light = "True"
                if traffic_light_color == "green":
                    self.save_to_executing_table(last_recommendation, "turned_to_green_while_slowing")
                else:
                    self.save_to_executing_table(last_recommendation, "yes")
            else:
                self.save_to_executing_table(last_recommendation, "no")

        if my_traffic_light is not None:
            self.last_TL = my_traffic_light.get_opendrive_id()
        else:
            self.last_TL = None

        if last_recommendation != self.current_recommendation:
            self.first_time_recommendation_shown = time.time()

        #is behind - actor hero

        if 'showSlowWarning' == self.current_recommendation and self.show_slow_counter >= 5 and self.shouldShowSuggestions:
            self.scenario.sendShowChangeLaneArrowRequest(False)
            self.scenario.sendShowStopWarningRequest(True)


        else:
            self.scenario.sendShowChangeLaneArrowRequest(False)
            self.scenario.sendShowStopWarningRequest(False)

        self.frame_number += 1

        self.write_info_log(my_traffic_light_waypoint, my_traffic_light_waypoint_lane_id, my_traffic_light_id, traffic_light_color)

    def is_behind(self, hero, actor):
        yaw = 90 - hero.get_transform().rotation.yaw
        rotationMatrix = self.calculate_rotation_matrix(yaw)
        actorVehicleLocalPose = self.calculate_transform_in_frame(rotationMatrix, actor.get_transform().location, hero.get_transform().location)
        return actorVehicleLocalPose[1] > 0

    def calculate_rotation_matrix(self, yaw):
        yaw = yaw % 360
        if yaw < 0:
            yaw += 360
        theta = np.radians(yaw)
        return np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta))))

    def calculate_transform_in_frame(self, rotation_matrix, translation, origin):
        translationVector = np.array((translation.x - origin.x, translation.y - origin.y))
        rotatedTransform = rotation_matrix.dot(translationVector)
        return rotatedTransform

    def get_actor_speed(self, actor):
        return self.calculate_actor_velocity(actor)

    def calculate_actor_velocity(self, actor):
        """
        Method to calculate the velocity of a actor
        """
        velocity_squared = actor.get_velocity().x**2
        velocity_squared += actor.get_velocity().y**2
        return math.sqrt(velocity_squared)

    def get_distance_between_vehicles(self, actor1, actor2):
        return self.get_distance_between_transforms(actor1.get_transform(), actor2.get_transform())

    def get_distance_between_transforms(self, transform1, transform2):
        return abs(transform1.location.distance(transform2.location))

    def calculate_distance(self, p1x, p1y, p1z, p2x, p2y, p2z):
        return math.sqrt((p2y - p1y) ** 2 + (p2x - p1x) ** 2 + (p2z - p1z) ** 2) - 2   # -2 cause of the cars size

    # speed1 - backward, speed2 - forward, distance
    def time_to_crash(self, speed1, speed2, distance):
        if speed1 <= speed2:
            return math.inf
        return distance / (speed1 - speed2)
