Source code and video examples for paper **Advice Provision in Teleoperation of Autonomous Vehicles**.
Authors: Yohai Trabelsim, Or Shabat, Joel Lanir, Oleg Maksimov and Sarit Kraus.


Running instructions:

# 
# Requirements:
# 

1. Carla simulator (version 0.9.10 or 0.9.13):
https://carla.readthedocs.io/en/latest/start_quickstart/

2. ScenarioRunner:
https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/

# 
# Running instructions:
# 

# 
# Environment variables:
# 
export CARLA_ROOT=YOUR_CARLA_INSTALLATION_DIRECTORY
export SCENARIO_RUNNER_ROOT=${CARLA_ROOT}/scenario_runner
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI


# Run the carla simulator
bash ${YOUR_CARLA_INSTALLATION_DIRECTORY}/CarlaUE4.sh &

sleep 10s

# Specify current scenario params. For instance:
$scenarioParams = '--scenarioId 50016032Town04 --townId Town04 --participantId test1 --scenarioName 50016032Town04 --foreignObjectNumber 42 --staticVehicleNumber 16 --normalVehicleNumber 55 --slowVehicleNumber 21 --pedestriansRunningNumber 60 --pedestriansCrossingNumber 71 --cloudiness 1 --precipitation 1 --precipitation_deposits 0 --wind_intensity 70 --sun_azimuth_angle 0 --sun_altitude_angle 70 --fog_density 0 --fog_distance 0 --fog_falloff 0 --wetness 5 --spawnPointX 222.86488342285156 --spawnPointY -175.6774139404297 --spawnPointZ 0.27344533801078796 --spawnPointYaw -84.516 --goalPointX 204.50672912597656 --goalPointY -210.0525360107422 --goalReachedTolerance 4.0'


# And run the scenario
python YOUR_PATH/scenarios/traffic_lights_scenario_control_spawner.py $scenarioParams > ./logs/errors_log.txt 2>&1 &
# OR
python YOUR_PATH/scenarios/slow_vehicle_adjacent_lanes_control_spawner.py $scenarioParams > ./logs/errors_log.txt 2>&1 &

