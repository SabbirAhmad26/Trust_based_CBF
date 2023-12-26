#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time
import traci

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from scipy.io import loadmat
import numpy as np
import init
from checkArrival import check_arrival
from scipy.integrate import odeint
import main_OCBF as controller
import getxy
from control import update_table, OCBF_time, Event_detector, OCBF_event
from conflictCAVS import search_for_conflictCAVS, search_for_conflictCAVS_trustversion
try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position


# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """

    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.sumo = sumo_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        traffic_manager = self.carla.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]
            # print(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(),
                                                                   sumo_actor.signals)
            else:
                carla_lights = None

            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        # Updates traffic lights in carla based on sumo information.
        if self.tls_manager == 'sumo':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)

    file_path1 = '/home/akua/Downloads/withoutmitigation/dataset_xcoor.xlsx'
    file_path2 = '/home/akua/Downloads/withoutmitigation/dataset_ycoor.xlsx'
    file_path3 = '/home/akua/Downloads/withoutmitigation/dataset_angle.xlsx'
    file_path = '/home/akua/Downloads/withoutmitigation/dataset_init.xlsx'
    data1 = pd.read_excel(file_path1)
    data2 = pd.read_excel(file_path2)
    data3 = pd.read_excel(file_path3)
    data = pd.read_excel(file_path)
    data_array1 = data1.values
    data_array2 = data2.values
    data_array3 = data3.values
    init_queue = data.values


    simulation_step = 0
    pointer = 0
    pen = 1
    temp = 0
    total = len(init_queue)
    with open('Position Values for ABCD', 'r') as file:
        trajs = file.read()

    global beta
    global cnt

    cnt = 0
    max_range = 400
    car, metric, CAV_e = init.init(total, max_range)

    try:
        while simulation_step < max_range:
            start = time.time()

            while pointer <= total - 1 and simulation_step == int(init_queue[pointer][2] * 10):
                car, pen = check_arrival(simulation_step, init_queue[pointer], car, pen, pointer, trajs)
                length = car['cars']
                pointer += 1
                car['order'] = np.append(car['order'], length)
                car = update_table(car)



            for vehicle in car['order']:

                vehicle = int(vehicle) - 1
                ego = car['que1'][vehicle]
                xi = ego['state'][0]
                vi = ego['state'][1]
                ui = ego['state'][2]
                id = ego["id"][1]



                CAV_e['acc'][simulation_step, id] = ui
                CAV_e['vel'][simulation_step, id] = vi
                CAV_e['pos'][simulation_step, id] = xi

                #ip, index, position = search_for_conflictCAVS(car["table"], ego)# order is from 1 to total but indices must start from 0
                ip, index, position = search_for_conflictCAVS_trustversion(car['que1'], car["table"], ego, 1, 0)


                if len(ip) > 0:
                    if ip[0] != -1: # first car in the list
                        xip = car['que1'][int(ip[0])-1]['state'][0]
                        vip = car['que1'][int(ip[0])-1]['state'][1]
                        phiRearEnd = ego["phiRearEnd"]
                        deltaSafetyDistance = ego["carlength"]
                        k_rear = ego["k_rear"]
                        ego["rearendconstraint"] = xip - xi - phiRearEnd * vi - deltaSafetyDistance
                        CAV_e['rear_end_CBF'][simulation_step, id] = vip - vi -phiRearEnd*ui + k_rear * (
                                xip - xi - phiRearEnd * vi - deltaSafetyDistance)
                        CAV_e['rear_end'][simulation_step, id] = ego["rearendconstraint"]


                for k in range(len(index)):
                    if index[k][0] == -1:
                        continue
                    else:
                        d1 = car['que1'][index[k][0] - 1]['metric'][position[k][0] + 3] - car['que1'][index[k][0] - 1]['state'][0]
                        d2 = ego['metric'][k + 4] - xi

                    xic = car['que1'][index[k][0] -1]['state'][0]
                    vic = car['que1'][index[k][0] -1]['state'][1]
                    deltaSafetyDistance = ego["carlength"]
                    phiLateral = ego['phiLateral']
                    k_lateral = ego['k_lateral'][k]
                    L = ego['metric'][k + 4]
                    bigPhi = phiLateral * xi / L
                    ego['lateralconstraint'][k] = d2 - d1 - bigPhi*vi - deltaSafetyDistance
                    CAV_e['lateral'][simulation_step, id, k] = ego['lateralconstraint'][k]
                    CAV_e['lateral_CBF'][simulation_step, id, k] = vic - vi - phiLateral * vi**2/L - bigPhi * ui +\
                    k_lateral*(d2 - d1 - bigPhi*vi - deltaSafetyDistance)


                ip_seen = -1
                flags = Event_detector(ego, car['que1'], ip, ip_seen, index, CAV_e)

                if 1 in flags:
                    CAV_e["x_tk"][id][0][0] = ego['state'][0]
                    CAV_e["v_tk"][id][0][0] = ego['state'][1]

                    for k in range(len(ip)):
                        vip = car["que1"][int(ip[k])-1]['state'][1]
                        xip = car["que1"][int(ip[k])-1]['state'][0]
                        CAV_e["v_tk"][ego["id"][1]][2 + k] = vip
                        CAV_e["x_tk"][ego["id"][1]][2 + k] = xip


                    for k in range(len(index)):
                        for j in range(len(index[k])):
                            if index[k][j] == -1:
                                continue
                            else:
                                vic = car["que1"][index[k][j] - 1]['state'][1]
                                xic = car["que1"][index[k][j] - 1]['state'][0]
                                CAV_e["v_tk"][ego["id"][1]][2 + k][j] = vic
                                CAV_e["x_tk"][ego["id"][1]][2 + k][j] = xic


                ego['prestate'] = ego['state']

                #ego['state'], ego['infeasibility'] = OCBF_time(simulation_step, ego, car['que1'], ip, index, position)
                ego['state'], ego['infeasibility'] = OCBF_event(simulation_step, ego, car['que1'], ip, index, position, flags)


            # update the position of each vehicle
            for vehicle in car['order']:
                vehicle = int(vehicle) - 1
                ego = car['que1'][vehicle]
                id = ego["id"][1]
                position = getxy.getXY(ego['lane'], ego['decision'],ego['state'][0],ego['prestate'][0],ego['j'],
                                       ego['realpose'], ego['prerealpose'])
                positionX = position[0]
                positionY = position[1]
                angle = position[2]
                ego['j'] = position[3]
                ego['prerealpose'] = ego['realpose']
                ego['realpose'] = [position[0], position[1]]

                if ego['state'][0] < ego["metric"][4]:
                    traci.vehicle.moveToXY(id, "", -1, positionX, positionY, angle)


            # check leave
            ids = [int(element) for element in traci.vehicle.getIDList() if element != 'carla0']
            if len(ids) - temp < 0:
                car['que1'] = [item for item in car['que1'] if item['id'][1] in ids]
                car['cars'] -= temp - len(ids)
                car['order'] = car['order'][2:-1] - 1
                car = update_table(car)
            temp = len(ids)


            synchronization.tick()
            simulation_step += 1
            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)
    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')

        synchronization.close()
        # return CAV_e


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.1,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)


    # CAV_e = synchronization_loop(arguments)
    # pointer = 1
    # total = 5
    # dt = 0.1
    # fig, axs = plt.subplots(2, figsize=(8, 10))
    # while pointer <= total:
    #     indicies = np.where(~np.isnan(CAV_e['acc'][:, pointer]))[0]
    #     time_samples = dt * indicies
    #
    #     acceleration = CAV_e['acc'][indicies, pointer]
    #     velocity = CAV_e['vel'][indicies, pointer]
    #     # position = CAV_e['pos'][non_nan_indices, pointer]
    #
    #
    #
    #
    #     axs[0].plot(time_samples, acceleration, linestyle='-')
    #     # axs[0].set_title('Non-NaN Values vs Indices')
    #     axs[0].set_xlabel('Time')
    #     axs[0].set_ylabel('Acceleration')
    #
    #     axs[1].plot(time_samples, velocity, linestyle='-')
    #     axs[1].set_xlabel('Time')
    #     axs[1].set_ylabel('Velocity')
    #     pointer += 1
    #
    # plt.tight_layout()
    # plt.show()



