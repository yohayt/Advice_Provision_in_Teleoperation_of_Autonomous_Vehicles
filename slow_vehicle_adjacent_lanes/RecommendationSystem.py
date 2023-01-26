from queue import Queue
import math
import numpy as np
import time
from sympy import symbols, Eq, solve

START_RECOMMENDATION_FROM_DISTANCE = 50
ALPHA = 0.125
BETA = 0.1
SAFETY_MARGIN_INITIALIZE = 0.25
REACTION_TIME_INITIALIZE = 2.0
RADAR_SIMULATION = 150

class RecommendationSystem:

    def __init__(self, args, scenario, actors, hero, world_map, file_name, world):
        self.world = world
        self.file_name = file_name

        self.uniqueRunIdentifier = str(args.participantId) + '_' + str(args.scenarioId) + '_' + str(args.scenarioName)

        try:
            with open(self.file_name, 'r') as f:
                reaction_time = float(f.readline())
        except Exception as e:
            reaction_time = REACTION_TIME_INITIALIZE

        if reaction_time < 1:
            reaction_time += 0.5
        if not 0 <= reaction_time <= 2:
            reaction_time = REACTION_TIME_INITIALIZE

        with open(self.file_name + "_table.txt", 'a+') as f1:
            f1.write(str(reaction_time)+"\n")

        self.scenario = scenario
        self.hero = hero
        self.actors = actors
        self.world_map = world_map
        self.reaction_time = reaction_time   # 1-10
        self.safety_margin = SAFETY_MARGIN_INITIALIZE
        self.minimum_time_to_move_from_lane = self.reaction_time + 4 * (1-BETA) * self.safety_margin  # seconds
        self.hero_lane = None
        self.hero_lane_id = None
        self.hero_left_lane = None
        self.hero_left_lane_id = None
        self.left_lane_is_available = None
        self.hero_speed = None    # m/s
        self.hero_waypoint = None
        self.distance_forward = None
        self.distance_left_forward = None
        self.distance_left_backward = None
        self.forward_car_speed = None
        self.left_forward_car_speed = None
        self.left_backward_car_speed = None
        self.lane_to_move_to = None
        self.first_time_recommendation_shown = 0.0
        self.executing_recommendation_time = 0
        self.current_recommendation = "do nothing"
        self.recommendations_queue = Queue(20)
        self.recommendations_counter = {'showTurnLeftArrow': 0, "do nothing": 0, 'showSlowWarning': 0}
        self.frame_number = 0


    def write_to_fast_debug_log(self, text):
        debugFastOutputFile = open('./logs/debugFastOutput.log', 'w+')  # a+    %10
        debugFastOutputFile.write(str(text))
        debugFastOutputFile.close()

    def save_driver_reaction_time(self):
        with open(self.file_name, 'w+') as f:
            f.write(str(self.reaction_time))

        with open(self.file_name + "_table.txt", 'a+') as f1:
            f1.write(str(self.reaction_time)+"\n")

    def save_to_executing_table(self, advice, status):
        with open('./advices_table_' + self.uniqueRunIdentifier + ".txt", 'a+') as f:
            f.write(str(time.time()) + "," + str(advice) + "," + str(status) + "\n")

    def calculate_and_save_reaction_time(self):
        updated = False
        self.executing_recommendation_time = time.time() - self.first_time_recommendation_shown
        self.executing_recommendation_time = min(4.0, self.executing_recommendation_time)
        self.save_to_executing_table(self.current_recommendation, "yes")
        if self.executing_recommendation_time > 0.5:
            self.update_reaction_time()
            self.calculate_safety_margin()
            self.calculate_minimum_time_to_move_from_lane()
            self.save_driver_reaction_time()
            updated = True
        return updated

    def calculate_and_save_reaction_time_for_failed_execution(self):
        self.executing_recommendation_time = time.time() - self.first_time_recommendation_shown
        if self.executing_recommendation_time > self.reaction_time:
            self.executing_recommendation_time = min(4.0, self.executing_recommendation_time)
            self.update_reaction_time()
            self.calculate_safety_margin()
            self.calculate_minimum_time_to_move_from_lane()
            self.save_driver_reaction_time()
            self.save_to_executing_table(self.current_recommendation, "no")

    def update_reaction_time(self):
        self.reaction_time = self.executing_recommendation_time * ALPHA + self.reaction_time * (1 - ALPHA)
        if self.reaction_time < 0.5:
            self.reaction_time = 0.5
        if self.reaction_time > 4:
            self.reaction_time = 4


    def calculate_minimum_time_to_move_from_lane(self):
        self.minimum_time_to_move_from_lane = self.reaction_time + 4 * self.safety_margin

    def calculate_safety_margin(self):
        self.safety_margin = (1-BETA) * self.safety_margin + BETA * abs(self.reaction_time - self.executing_recommendation_time)

    # my_info = location (x,y or distance,azimuth  ??), heading (which direction am I driving to), speed (KM Per Hour), lane
    # actors = same info as "my_info but in array - for each other car than mine.
    # num_of_lanes_in_the_road = number of the lanes in the current road
    def SampleAndRecommend(self):

        last_recommendation = self.current_recommendation
        self.hero_speed = self.get_actor_speed(self.hero)
        self.hero_lane = self.world_map.get_waypoint(self.hero.get_transform().location, project_to_road=True)

        moved_lane = False

        if self.hero_lane is not None:
            self.hero_lane_id = self.hero_lane.lane_id
        else:
            self.hero_lane_id = 100
        if self.current_recommendation == 'showTurnLeftArrow' and self.hero_lane_id == self.lane_to_move_to:
            updated = self.calculate_and_save_reaction_time()
            moved_lane = True
            if updated:
                self.recommendations_queue = Queue(20)
                self.current_recommendation = "do nothing"
                self.recommendations_counter = {'showTurnLeftArrow': 0, "do nothing": 0, 'showSlowWarning': 0}
                self.first_time_recommendation_shown = 0.0

        self.hero_left_lane = self.world_map.get_waypoint(self.hero.get_transform().location, project_to_road=True).get_left_lane()
        if self.hero_left_lane is not None:
            self.hero_left_lane_id = self.hero_left_lane.lane_id
        else:
            self.hero_left_lane_id = 1000
        if (self.hero_left_lane_id == self.hero_lane_id + 1) or (self.hero_left_lane_id == self.hero_lane_id - 1):
            self.left_lane_is_available = True
        else:
            self.left_lane_is_available = False

        self.forward_car_speed = math.inf
        self.left_forward_car_speed = math.inf
        self.left_backward_car_speed = -math.inf
        self.distance_forward = math.inf
        self.distance_left_forward = math.inf
        self.distance_left_backward = math.inf

        for actor in self.actors:
            if actor != self.hero and 'vehicle' in actor.type_id:
                actor_lane_id = self.world_map.get_waypoint(actor.get_transform().location, project_to_road=True).lane_id
                if actor_lane_id == self.hero_lane_id:
                    if self.is_behind(self.hero, actor):
                        distance = self.get_distance_between_vehicles(self.hero, actor) - 4    # pure distance
                        if distance < RADAR_SIMULATION:  # simulates radar
                            if 0 < distance < self.distance_forward:
                                self.distance_forward = distance
                                self.forward_car_speed = self.get_actor_speed(actor)
                elif self.left_lane_is_available:
                    if actor_lane_id == self.hero_left_lane_id:
                        distance = self.get_distance_between_vehicles(self.hero, actor) - 1.5  # avg distance
                        if distance < 100:  # simulates radar
                            if self.is_behind(self.hero, actor):
                                if distance < self.distance_left_forward:
                                    self.distance_left_forward = distance
                                    self.left_forward_car_speed = self.get_actor_speed(actor)
                            else:
                                if distance < self.distance_left_backward:
                                    self.distance_left_backward = distance
                                    self.left_backward_car_speed = self.get_actor_speed(actor)

        next_recommendation = "do nothing"
        if 0 <= self.time_to_crash(self.hero_speed, self.forward_car_speed, self.distance_forward) < 2 or self.distance_forward < 8:
            if self.hero_speed > 1.2:
                next_recommendation = 'showSlowWarning'
        elif self.left_lane_is_available:
            if (self.forward_car_speed <= self.left_forward_car_speed) and ((self.time_to_crash(self.hero_speed, self.forward_car_speed, self.distance_forward) < 12)
                                                                            or (self.distance_forward < START_RECOMMENDATION_FROM_DISTANCE)):  # and self.car_speed_forward < speed_limit
                if self.distance_left_forward > 7 and \
                        self.time_to_crash(self.left_backward_car_speed, self.hero_speed, self.distance_left_backward) >= self.minimum_time_to_move_from_lane:
                    if self.time_to_crash(self.hero_speed, self.left_forward_car_speed, self.distance_left_forward) >= self.minimum_time_to_move_from_lane and \
                            self.distance_left_backward > 7:
                        next_recommendation = 'showTurnLeftArrow'
        else:
            next_recommendation = "do nothing"

        if self.recommendations_queue.full():
            self.recommendations_counter[self.recommendations_queue.get()] -= 1
            self.recommendations_queue.put(next_recommendation)
        else:
            self.recommendations_queue.put(next_recommendation)
        self.recommendations_counter[next_recommendation] += 1


        new_recommendation = ""
        # if action is done - decrease 0.3 sec from firs time rec shown ------ set by tics
        if time.time() - self.first_time_recommendation_shown > 0.3 and self.recommendations_queue.full():
            num_of_rec = 0
            for key, val in self.recommendations_counter.items():
                if val > num_of_rec:
                    new_recommendation = key
                    num_of_rec = val

            if self.current_recommendation == 'showTurnLeftArrow':
                if new_recommendation != self.current_recommendation and self.lane_to_move_to != self.hero_lane_id:
                    self.calculate_and_save_reaction_time_for_failed_execution()

            if new_recommendation != self.current_recommendation:
                self.first_time_recommendation_shown = time.time()

            self.current_recommendation = new_recommendation
            if self.current_recommendation == 'showTurnLeftArrow':
                self.lane_to_move_to = self.hero_left_lane_id


            # how long the sign stays
            if not moved_lane:
                if 'showTurnLeftArrow' == self.current_recommendation:
                    self.scenario.sendShowChangeLaneArrowRequest(True)
                    self.scenario.sendShowStopWarningRequest(False)

                elif 'showSlowWarning' == self.current_recommendation:
                    self.scenario.sendShowChangeLaneArrowRequest(False)
                    self.scenario.sendShowStopWarningRequest(True)

                else:
                    self.scenario.sendShowChangeLaneArrowRequest(False)
                    self.scenario.sendShowStopWarningRequest(False)
            else:
                self.scenario.sendShowChangeLaneArrowRequest(False)
                self.scenario.sendShowStopWarningRequest(False)

        self.frame_number += 1

        self.write_to_fast_debug_log("hero speed: " + str(self.hero_speed) + "\n"
                                     + "forward speed: " + str(self.forward_car_speed) + "\n"
                                     + "distance forward: " + str(self.distance_forward) + "\n"
                                     + "left forward speed: " + str(self.left_forward_car_speed) + "\n"
                                     + "distance left forward: " + str(self.distance_left_forward) + "\n"
                                     + "left backward speed: " + str(self.left_backward_car_speed) + "\n"
                                     + "distance left backward: " + str(self.distance_left_backward) + "\n"
                                     + "hero lane: " + str(self.hero_lane_id) + "\n"
                                     + "lane to move to: " + str(self.lane_to_move_to) + "\n"
                                     + "reaction time: " + str(self.reaction_time) + "\n"
                                     + "safety margin: " + str(self.safety_margin) + "\n"
                                     + "recommendation start time: " + str(self.first_time_recommendation_shown) + "\n"
                                     + "current time: " + str(time.time()) + "\n"
                                     + "last recommendation: " + str(last_recommendation) + "\n"
                                     + "next recommendation: " + str(new_recommendation) + "\n"
                                     + "minimum time to move form lane: " + str(self.minimum_time_to_move_from_lane) + "\n"
                                     + "executing_recommendation_time: " + str(self.executing_recommendation_time) + "\n"
                                     + "frame_number: " + str(self.frame_number) + "\n"
                                     + "queue length: " + str(self.recommendations_queue.qsize()) + "\n"
                                     + "recommendation: " + str(self.current_recommendation) + "\n"
                                     + " hero x: " + str(self.hero.get_transform().location.x) + "\n"
                                     + " hero y: " + str(self.hero.get_transform().location.y) + "\n"
                                     + " hero z: " + str(self.hero.get_transform().location.z) + "\n"
                                     + " hero yaw: " + str(self.hero.get_transform().rotation.yaw) + "\n"

                                     )


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

    def time_to_crash(self, speed1, speed2, distance):
        if speed1 <= speed2:
            return math.inf
        return distance / (speed1 - speed2)
