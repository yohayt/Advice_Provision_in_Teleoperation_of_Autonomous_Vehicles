#!/usr/bin/env python

import glob
import os
import sys
import time
import math
import traceback

import rospy
from std_msgs.msg import Bool
from carla_msgs.msg import CarlaLaneInvasionEvent


sys.path.insert(1, './RecommendationSystem')
from RecommendationSystem import RecommendationSystem

import numpy as np


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

import py_trees
import random

import os
import sys
import csv
import time
import json


def get_waypoint_in_distance(waypoint, distance):
    """
    Obtain a waypoint in a given distance from the current actor's location.
    Note: Search is stopped on first intersection.
    @return obtained waypoint and the traveled distance
    """
    traveled_distance = 0
    while None != waypoint and not waypoint.is_intersection and traveled_distance < distance:
        waypoint_new = waypoint.next(1.0)[-1]
        traveled_distance += waypoint_new.transform.location.distance(waypoint.transform.location)
        waypoint = waypoint_new

    return waypoint, traveled_distance


def get_waypoint_in_distance_backwards(waypoint, distance):
    """
    Obtain a waypoint in a given distance from the current actor's location.
    Note: Search is stopped on first intersection.
    @return obtained waypoint and the traveled distance
    """
    traveled_distance = 0
    while traveled_distance < distance and None != waypoint:
        waypoint_new = waypoint.previous(1.0)[-1]
        traveled_distance += waypoint_new.transform.location.distance(waypoint.transform.location)
        waypoint = waypoint_new

    return waypoint, traveled_distance

class SlowVehicleAdjacentLanesControl():

    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    def __init__(self, client, args, regularTrafficManager, slowTrafficManager):

        self.args = args
        self.client = client
        self.world = client.get_world()
        self.regularTrafficManager = regularTrafficManager
        self.slowTrafficManager = slowTrafficManager

        self.takeOverControlNotifierPub = rospy.Publisher("/carla/hero/take_over_control_enabled", Bool, queue_size = 1)
        
        self.vehicleControlManualOverridePub = rospy.Publisher("/carla/hero/vehicle_control_manual_override", Bool, queue_size = 1)
        self.enableAutopilotPub = rospy.Publisher("/carla/hero/enable_autopilot", Bool, queue_size = 1)
        self.showChangeLaneArrowPub = rospy.Publisher("/carla/hero/change_lane_arrow_switch", Bool, queue_size = 1)
        self.showStopWarningPub = rospy.Publisher("/carla/hero/stop_warning_switch", Bool, queue_size = 1)

        self.mainCameraUpViewX = 0.5
        self.mainCameraUpViewY = -1.2
        self.mainCameraUpViewZ = 30.0
        self.mainCameraUpViewPitch = -90

        
        self.mainCameraDriverPoseX = 0.3
        self.mainCameraDriverPoseY = -0.40
        self.mainCameraDriverPoseZ = 1.3
        self.mainCameraDriverPosePitch = 0

        self.spawnedFastVehiclesIds = []
        self.spawnedSlowVehiclesIds = []

        self.slowVehicleId = -1
        self.firstFastVehicleId = -1

        self.vehiclesBehindOfEgoCount_ = self.args.fastVehiclesAmount
    
    def getDistanceBetweenTransforms(self, transform1, transform2):

        return self.getDistance(transform1.location.x, transform1.location.y, transform2.location.x, transform2.location.y)

        return math.sqrt((p2Y - p1Y)**2 + (p2X - p1X)**2)
        
    def getDistance(self, p1X, p1Y, p2X, p2Y):
        return math.sqrt((p2Y - p1Y)**2 + (p2X - p1X)**2)
        
    def getSpawnPositionsFromWaypoint(self, waypoint):

        prevVehicleWaypoint = waypoint

        spawnPositions = []

        for currActorNum in range(self.vehiclesBehindOfEgoCount_):

            distanceFromOtherWaypoint = self.args.initialDistanceBetweenFastVehicles

            currVehicleWaypoint, d_ = get_waypoint_in_distance_backwards(prevVehicleWaypoint, distanceFromOtherWaypoint)

            tempTransform = self.getSpawnPointTransformFromWayPoint(currVehicleWaypoint)


            prevVehicleWaypoint = currVehicleWaypoint

            spawnPositions.append(tempTransform)

        return spawnPositions


    def initialize(self):

        self.vehiclesSpawnPositions = []
        self.objectsSpawnPositions = []

        spawnPointX = self.args.spawnPointX
        spawnPointY = self.args.spawnPointY
        spawnPointZ = self.args.spawnPointZ
        spawnPointYaw = self.args.spawnPointYaw

        while True:

            hero_vehicles = [actor for actor in self.world.get_actors()
                            if 'vehicle' in actor.type_id and actor.attributes['role_name'] == 'hero']

            if (len(hero_vehicles) == 0):

                print('Waiting for ego vehicle')
                time.sleep(1)
                continue

            self.hero_actor = hero_vehicles[0]

            egoVehicleSpawnPointTransform = carla.Transform(
                carla.Location(spawnPointX, spawnPointY, spawnPointZ + 0.1), carla.Rotation(yaw=spawnPointYaw))

            self.hero_actor.set_transform(egoVehicleSpawnPointTransform)

            break


        egoVehicleLocation = egoVehicleSpawnPointTransform.location


        worldMap = self.world.get_map()

        closestWaypoint = worldMap.get_waypoint(egoVehicleLocation, project_to_road=True, lane_type=(carla.LaneType.Driving))

        if (self.args.slowVehicleDistance != 0):

            slowVehicleWaypoint, d_ = get_waypoint_in_distance(closestWaypoint, self.args.slowVehicleDistance)

            tempTransform = self.getSpawnPointTransformFromWayPoint(slowVehicleWaypoint)

            spawnedVehiclesIds = self.spawnVehicles([tempTransform], True, True)

            self.slowVehicleId = spawnedVehiclesIds[0]

        leftLaneWaypoint = closestWaypoint.get_left_lane()

        spawnPositions = []

        if (self.args.firstFastVehicleInitialDistance < 0):
            leftLaneWaypoint, d_ = get_waypoint_in_distance_backwards(leftLaneWaypoint, -self.args.firstFastVehicleInitialDistance)
        else:
            leftLaneWaypoint, d_ = get_waypoint_in_distance(leftLaneWaypoint, self.args.firstFastVehicleInitialDistance)

        leftLaneWaypoint, d_ = get_waypoint_in_distance_backwards(leftLaneWaypoint, -50)

        spawnPositions = spawnPositions + self.getSpawnPositionsFromWaypoint(leftLaneWaypoint)        

        self.spawnVehicles(spawnPositions)

        if (len(self.spawnedFastVehiclesIds) > 0):
            self.firstFastVehicleId = self.spawnedFastVehiclesIds[0]

        tm = self.client.get_trafficmanager(8001)
        
        self.hero_actor.set_autopilot(False, 8001)

        msg = Bool()
        msg.data = False
        self.enableAutopilotPub.publish(msg)

        msg = Bool()
        msg.data = True
        self.takeOverControlNotifierPub.publish(msg)

        actor_list = self.world.get_actors()
        
        for vehicleId in self.spawnedFastVehiclesIds:

            currentActor = actor_list.find(vehicleId)

            self.regularTrafficManager.auto_lane_change(currentActor, False)

            currentActor.set_autopilot(True, self.regularTrafficManager.get_port())

            self.regularTrafficManager.vehicle_percentage_speed_difference(currentActor, -1 * self.args.fastVehiclesExceedingSpeedLimitPercentage)

        for vehicleId in self.spawnedSlowVehiclesIds:

            currentActor = actor_list.find(vehicleId)

            self.slowTrafficManager.auto_lane_change(currentActor, False)

            currentActor.set_autopilot(True, self.slowTrafficManager.get_port())

            self.slowTrafficManager.vehicle_percentage_speed_difference(currentActor, self.args.slowVehicleSpeedLimitPercentage)

            self.regularTrafficManager.collision_detection(self.hero_actor, currentActor, False)

        self.hero_actor.set_autopilot(False, 8001)

        msg = Bool()
        msg.data = False
        self.takeOverControlNotifierPub.publish(msg)

        time.sleep(0.3)

        msg = Bool()
        msg.data = False
        self.enableAutopilotPub.publish(msg)

    def calculateActorVelocity(self, actor):
        """
        Method to calculate the velocity of a actor
        """
        velocity_squared = actor.get_velocity().x**2
        velocity_squared += actor.get_velocity().y**2
        return math.sqrt(velocity_squared)

    def getDistanceBetweenVehicles(self, actor1, actor2):

        return self.getDistanceBetweenTransforms(actor1.get_transform(), actor2.get_transform())

    def getSlowVehicle(self):

        actor_list = self.world.get_actors()

        return actor_list.find(self.slowVehicleId)

    def getFirstFastVehicle(self):

        actor_list = self.world.get_actors()

        return actor_list.find(self.firstFastVehicleId)
    
    def getFastVehicles(self):

        fastVehicles = []

        actor_list = self.world.get_actors()

        for fastVehicleId in self.spawnedFastVehiclesIds:

            fastVehicles.append(actor_list.find(fastVehicleId))

        return fastVehicles
    
    def getActorSpeed(self, actor):

        return self.calculateActorVelocity(actor)

    def getDistanceToSlowVehicle(self):

        slowVehicleTransform = self.getSlowVehicleTransform()

        if (None == slowVehicleTransform):
            return -1

        return self.getDistanceBetweenTransforms(slowVehicleTransform, self.hero_actor.get_transform())

    def getDistanceToFirstFastVehicle(self):

        firstFastVehicleTransform = self.getFirstFastVehicleTransform()

        if (None == firstFastVehicleTransform):
            return -1

        return self.getDistanceBetweenTransforms(firstFastVehicleTransform, self.hero_actor.get_transform())

    def getSlowVehicleTransform(self):

        if (self.slowVehicleId == -1):
            return None

        actor_list = self.world.get_actors()
        
        slowVehicleActor = actor_list.find(self.slowVehicleId)

        return slowVehicleActor.get_transform()

    def getFirstFastVehicleTransform(self):

        if (self.firstFastVehicleId == -1):
            return None

        actor_list = self.world.get_actors()
        
        firstFastVehicleActor = actor_list.find(self.firstFastVehicleId)

        return firstFastVehicleActor.get_transform()

    def getSpawnPointTransformFromWayPoint(self, waypoint):

        return carla.Transform(
            carla.Location(waypoint.transform.location.x,
                        waypoint.transform.location.y,
                        waypoint.transform.location.z + 0.5),
            waypoint.transform.rotation)

    def spawnVehicles(self, spawnPositions, autopilotEnabled = True, isSlowVehicle = False):

        spawnedVehiclesIds = []

        try:

            blueprints = self.world.get_blueprint_library().filter('vehicle.*')

            if True:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]

            blueprints = sorted(blueprints, key=lambda bp: bp.id)

            # @todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            SetVehicleLightState = carla.command.SetVehicleLightState
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicle
            # --------------
            batch = []

            vehicleNum = 0

            for spawnPosition in spawnPositions:
                    
                blueprint = random.choice(blueprints)

                blueprint.set_attribute('role_name', 'autopilot_'+str(vehicleNum))

                vehicleNum += 1

                # prepare the light state of the cars to spawn
                light_state = vls.NONE

                batch.append(SpawnActor(blueprint, spawnPosition))

            for response in self.client.apply_batch_sync(batch, False):
                if response.error:
                    logging.error(response.error)
                else:
                    if (isSlowVehicle):
                        self.spawnedSlowVehiclesIds.append(response.actor_id)
                    else:
                        self.spawnedFastVehiclesIds.append(response.actor_id)

                    spawnedVehiclesIds.append(response.actor_id)

        except Exception as e:
            print(e)
            return []
        
        return spawnedVehiclesIds

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        # self.remove_all_actors()
        pass


    def putCamerasToInitialPose(self):

        actors = self.world.get_actors()

        for actor in actors:

            if ('sensor.camera.rgb' == actor.type_id):
                
                if ('front' == actor.attributes['role_name']):

                    actor.set_transform(carla.Transform(
                                carla.Location(
                                    x=self.mainCameraUpViewX,
                                    y=self.mainCameraUpViewY,
                                    z=self.mainCameraUpViewZ), carla.Rotation(roll=0, pitch=self.mainCameraUpViewPitch, yaw=0)))

                elif ('rear_left' == actor.attributes['role_name']):
                    actor.set_transform(carla.Transform(
                                carla.Location(
                                    x=1,
                                    y=0,
                                    z=0.5), carla.Rotation(roll=0, pitch=0, yaw=0)))

                elif ('rear_right' == actor.attributes['role_name']):
                    actor.set_transform(carla.Transform(
                                carla.Location(
                                    x=1,
                                    y=0,
                                    z=0.5), carla.Rotation(roll=0, pitch=0, yaw=0)))

    def putCamerasToDriverPose(self):

        secondsToAnimate = 3
        iterations = 1000

        actors = self.world.get_actors()

        for actor in actors:

            if ('sensor.camera.rgb' == actor.type_id):
                
                if ('front' == actor.attributes['role_name']):
                    stepX = (self.mainCameraDriverPoseX - self.mainCameraUpViewX) / float(iterations)
                    stepY = (self.mainCameraDriverPoseY - self.mainCameraUpViewY) / float(iterations)
                    stepZ = (self.mainCameraDriverPoseZ - self.mainCameraUpViewZ) / float(iterations)
                    stepPitch = (self.mainCameraDriverPosePitch - self.mainCameraUpViewPitch) / float(iterations)

                    currX = self.mainCameraUpViewX
                    currY = self.mainCameraUpViewY
                    currZ = self.mainCameraUpViewZ
                    currPitch = self.mainCameraUpViewPitch

                    for i in range(iterations):

                        currX += stepX
                        currY += stepY
                        currZ += stepZ
                        currPitch += stepPitch

                        actor.set_transform(carla.Transform(
                                    carla.Location(
                                        x=currX,
                                        y=currY,
                                        z=currZ), carla.Rotation(roll=0, pitch=currPitch, yaw=0)))

                        time.sleep(secondsToAnimate / float(iterations + i * secondsToAnimate))

                elif ('rear_left' == actor.attributes['role_name']):
                        actor.set_transform(carla.Transform(
                                    carla.Location(
                                        x=0.61,
                                        y=-0.95,
                                        z=1.1), carla.Rotation(roll=0, pitch=0, yaw=-160.0)))

                elif ('rear_right' == actor.attributes['role_name']):
                    actor.set_transform(carla.Transform(
                                carla.Location(
                                    x=0.61,
                                    y=0.95,
                                    z=1.1), carla.Rotation(roll=0, pitch=0, yaw=160.0)))

    def sendShowChangeLaneArrowRequest(self, shouldShow):

        msg = Bool()
        msg.data = shouldShow
        self.showChangeLaneArrowPub.publish(msg)

    def sendShowStopWarningRequest(self, shouldShow):

        msg = Bool()
        msg.data = shouldShow
        self.showStopWarningPub.publish(msg)

laneInvasionRegistered = False

def laneInvasionCallback(msg):

    global laneInvasionRegistered
    laneInvasionRegistered = not laneInvasionRegistered

def calculateRotationMatrix(yaw):

    yaw = yaw  % 360

    if (yaw < 0):
        yaw += 360

    theta = np.radians(yaw)


    return np.array(( (np.cos(theta), -np.sin(theta)),
                (np.sin(theta),  np.cos(theta)) ))

def calculateTransformInFrame(rotationMatrix, translation, origin):

    translationVector = np.array((translation.x - origin.x, translation.y - origin.y))

    rotatedTransorm = rotationMatrix.dot(translationVector)

    return rotatedTransorm

def main():

    global laneInvasionRegistered

    laneInvasionRegistered = False

    changeLaneArrowFilePath = './show_change_lane_arrow'
    stopWarningFilePath = './show_stop_warning'

    if os.path.exists(changeLaneArrowFilePath):
        os.remove(changeLaneArrowFilePath)

    if os.path.exists(stopWarningFilePath):
        os.remove(stopWarningFilePath)

    rospy.init_node('vehicles_control_node', anonymous=True)

    random.seed(10)

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)

    print('connected to carla')

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')

    argparser.add_argument(
        '--scenarioId',
        default='none',
        # required=True
        required=False
        )
    argparser.add_argument(
        '--townId',
        default='none',
        required=True
        )
    argparser.add_argument(
        '--participantId',
        default='none',
        # required=True
        required=False
        )
    argparser.add_argument(
        '--advisingAgentEnabled',
        default='true',
        # required=True
        required=False
        )
    argparser.add_argument(
        '--scenarioName',
        default='none',
        # required=True
        required=False
        )
    argparser.add_argument(
        '--foreignObjectNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--staticVehicleNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--normalVehicleNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--slowVehicleNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--pedestriansRunningNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--pedestriansCrossingNumber',
        default=0.0,
        type=int,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--cloudiness',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--precipitation',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--precipitation_deposits',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--wind_intensity',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--sun_azimuth_angle',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--sun_altitude_angle',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--fog_density',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--fog_distance',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--fog_falloff',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--wetness',
        default=0.0,
        type=float,
        # required=True
        required=False
        )
    argparser.add_argument(
        '--startWithAutopilot',
        default='False',
        required=False
        )
    argparser.add_argument(
        '--spawnPointX',
        default=0.0,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--spawnPointY',
        default=0.0,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--spawnPointZ',
        default=0.0,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--spawnPointYaw',
        default=0.0,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--goalPointX',
        default=0.0,
        type=float,
        required=False
        )
    argparser.add_argument(
        '--goalPointY',
        default=0.0,
        type=float,
        required=False
        )
    argparser.add_argument(
        '--goalReachedTolerance',
        default=0.5,
        type=float,
        required=False
        )

    argparser.add_argument(
        '--fastVehiclesAmount',
        default=30,
        type=int,
        required=True
        )
    argparser.add_argument(
        '--initialDistanceBetweenFastVehicles',
        default=5,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--slowVehicleDistance',
        default=200,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--firstFastVehicleInitialDistance',
        default=60,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--fastVehiclesExceedingSpeedLimitPercentage',
        default=150,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--slowVehicleSpeedLimitPercentage',
        default=50,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--egoVehicleExceedingSpeedLimitPercentage',
        default=170,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--secondsToWaitBeforeDriverViewSetup',
        default=7,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--secondsToWaitBeforeDisablingEgoAutopilot',
        default=5,
        type=float,
        required=True
        )
    argparser.add_argument(
        '--experimentorAvgResponseTime',
        default=-1,
        type=float,
        required=False
        )

    # Parse arguments
    args, unknown = argparser.parse_known_args()
    
    print('scenarioId: ' + str(args.scenarioId))
    print('townId: ' + str(args.townId))
    print('participantId: ' + str(args.participantId))
    print('advisingAgentEnabled: ' + str(args.advisingAgentEnabled))
    print('scenarioName: ' + str(args.scenarioName))
    print('experimentorAvgResponseTime: ' + str(args.experimentorAvgResponseTime))

    client.load_world(args.townId)
    
    regularTrafficManager = client.get_trafficmanager(8000)

    regularTrafficManager.set_random_device_seed(1)
    regularTrafficManager.set_hybrid_physics_mode(False)
    regularTrafficManager.set_synchronous_mode(False)
    regularTrafficManager.set_global_distance_to_leading_vehicle(0.3)

    slowTrafficManager = client.get_trafficmanager(8020)
    
    slowTrafficManager.set_random_device_seed(1)
    slowTrafficManager.set_hybrid_physics_mode(False)
    slowTrafficManager.set_synchronous_mode(False)
    slowTrafficManager.set_global_distance_to_leading_vehicle(1.0)

    world = client.get_world()

    weather = carla.WeatherParameters(
            cloudiness=args.cloudiness,
            precipitation=args.precipitation,
            precipitation_deposits=args.precipitation_deposits,
            wind_intensity=args.wind_intensity,
            sun_azimuth_angle=args.sun_azimuth_angle,
            sun_altitude_angle=args.sun_altitude_angle,
            fog_density=args.fog_density,
            fog_distance=args.fog_distance,
            fog_falloff=args.fog_falloff,
            wetness=args.wetness)

    world.set_weather(weather)

    scenario = SlowVehicleAdjacentLanesControl(client, args, regularTrafficManager, slowTrafficManager)

    scenario.initialize()

    try:

        for actor in world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.set_green_time(4)

        worldMap = world.get_map()

        shouldShowSuggestions = 'false' != args.advisingAgentEnabled

        recommendationSystem = RecommendationSystem.RecommendationSystem(args, scenario, world.get_actors(), scenario.hero_actor, worldMap, './IO/reaction_time_'+args.participantId+'.txt', world, shouldShowSuggestions)
            
        while True:
            
            world.wait_for_tick()

            recommendationSystem.SampleAndRecommend()

    except Exception as e:
        print(e)

if __name__ == '__main__':

    main()
