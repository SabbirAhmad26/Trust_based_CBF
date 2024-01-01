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
from metavariable_accident import s1, s2, s3, dt, noise1, noise2, const1, const2, const3, const4, max_range,L_end,trust_threshold
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
import vision
from control_accident import update_table, OCBF_time, Event_detector, OCBF_event, no_model_attacker,random_init_attacker,strategic_attacker,no_rule_attacker
from conflictCAVS import search_for_conflictCAVS, search_for_conflictCAVS_trustversion
from TrustCal import calculate_trust
from attack_mitigation import mitigation_function
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

runMode = 'False'

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


    file_path = './Dataset/Accident_dataset/dataset_init.xlsx'
    data = pd.read_excel(file_path)
    init_queue = data.values

    #
    # init_queue_spoofed = np.array([[1, 'vehicle.audi.a2', 4, 'best', '0.0.00', '-26.0.00', 5, 0, 1, 2, 3],
    #                                [2, 'vehicle.audi.a2', 4.1, 'best', '-39.0.00', '25.0.00', 5, 0, 1, 6, 3],
    #                                [3, 'vehicle.audi.a2', 4.2, 'best', '-29.0.00', '39.0.00', 5, 0, 1, 4, 3],
    #                                [4, 'vehicle.audi.a2', 4.3, 'best', '-16.0.00', '-0.0.00', 5, 0, 1, 8, 3],
    #                                [5, 'vehicle.audi.a2', 4.4, 'best', '0.0.00', '25.0.00', 5, 0, 1, 1, 2],
    #                                [6, 'vehicle.audi.a2', 4.5, 'best', '-39.0.00', '-26.0.00', 5, 0, 1, 5, 2]], dtype=object)

    init_queue_spoofed = np.array([[1, 'vehicle.audi.a2', 2.5, 'best', '-39.0.00', '-26.0.00', 5, 0, 1, 5, 2] ], dtype=object)


    total_spooefed = len(init_queue_spoofed)
    pointer_spoofed = 0

    simulation_step = 0
    pointer = 0

    pen = 1
    total = len(init_queue)
    with open('Position Values for ABCD', 'r') as file:
        trajs = file.read()
    car, metric, CAV_e = init.init(total + total_spooefed, max_range)
    trust = True
    mitigation = False
    update_class_k_function = True

    try:
        while simulation_step < max_range:
            start = time.time()

            while (pointer <= total - 1 and simulation_step == int(init_queue[pointer][2] * 10)) or\
                   (pointer_spoofed <= total_spooefed - 1 and simulation_step == int(init_queue_spoofed[pointer_spoofed][2] * 10)):

                if pointer <= total - 1 and simulation_step == int(init_queue[pointer][2] * 10):
                    car, pen = check_arrival(simulation_step, init_queue[pointer], car, pen, trajs)
                    pointer += 1
                else:
                    car, pen = check_arrival(simulation_step, init_queue_spoofed[pointer_spoofed], car, pen, trajs)
                    pointer_spoofed += 1

                length = car['cars']
                car['order'] = np.append(car['order'], length)
                car = update_table(car)
                CAV_e['arrivalexit'][length, 0] = dt * simulation_step




            # print(simulation_step, car['order'])

            for vehicle in car['order']:
                vehicle = vehicle - 1
                ego = car['que'][vehicle]
                xi = ego['state'][0]
                vi = ego['state'][1]
                ui = ego['state'][2]
                id = ego["id"][1]

                CAV_e['acc'][simulation_step, id] = ui
                CAV_e['vel'][simulation_step, id] = vi
                CAV_e['pos'][simulation_step, id] = xi

                ego['ip'], ego['index'], ego['position'] = search_for_conflictCAVS_trustversion(car['que'],
                                                                                                car["table"], ego, 1,
                                                                                                trust_threshold)
                ip, index, position = ego['ip'], ego['index'], ego['position']

                if ego['agent'] == 1:
                    ego['prestate'] = ego['state']
                    ego['state'] = no_model_attacker(simulation_step, ego)
                elif ego['agent'] == 2:
                    ego['prestate'] = ego['state']
                    ego['state'] = random_init_attacker(simulation_step, ego)
                elif ego['agent'] == 3:
                    ego['prestate'] = ego['state']
                    ego['state'] = no_rule_attacker(simulation_step, ego)
                elif ego['agent'] == 4:
                    ego['prestate'] = ego['state']
                    ego['state'] = strategic_attacker(simulation_step, ego)
                else:

                    if len(ip) > 0:
                        if ip[0] != -1: # first car in the list
                            ip_index = ip[0] -1
                            xip = car['que'][ip_index]['state'][0]
                            vip = car['que'][ip_index]['state'][1]
                            phiRearEnd = ego["phiRearEnd"]
                            deltaSafetyDistance = ego["carlength"]
                            k_rear = ego["k_rear_end"]
                            ego["rearendconstraint"] = xip - xi - phiRearEnd * vi - deltaSafetyDistance
                            CAV_e['rear_end_CBF'][simulation_step, id] = vip - vi -phiRearEnd*ui + k_rear * (
                                    xip - xi - phiRearEnd * vi - deltaSafetyDistance)
                            CAV_e['rear_end'][simulation_step, id] = ego["rearendconstraint"]
                            #print(xip - xi,CAV_e['rear_end_CBF'][simulation_step, id])


                    for k in range(len(index)):
                        if index[k][0] == -1:
                            continue
                        else:
                            ic_index = index[k][0] - 1
                            d1 = car['que'][ic_index]['metric'][position[k][0] + 3] - car['que'][ic_index]['state'][0]
                            d2 = ego['metric'][k + 4] - xi

                        xic = car['que'][ic_index]['state'][0]
                        vic = car['que'][ic_index]['state'][1]
                        deltaSafetyDistance = ego["carlength"]
                        phiLateral = ego['phiLateral']
                        k_lateral = ego['k_lateral'][k]
                        L = ego['metric'][k + 4]
                        bigPhi = phiLateral * xi / L
                        ego['lateralconstraint'][k] = d2 - d1 - bigPhi*vi - deltaSafetyDistance
                        CAV_e['lateral'][simulation_step, id, k] = ego['lateralconstraint'][k]
                        CAV_e['lateral_CBF'][simulation_step, id, k] = vic - vi - phiLateral * vi**2/L - bigPhi * ui +\
                        k_lateral*(d2 - d1 - bigPhi*vi - deltaSafetyDistance)

                    ego['see'] = vision.vision(car['que'], ego)
                    flags = Event_detector(ego, car['que'], ip, index, CAV_e)

                    if 1 in flags:
                        CAV_e["x_tk"][id][0][0] = ego['state'][0]
                        CAV_e["v_tk"][id][0][0] = ego['state'][1]

                        for k in range(len(ip)):
                            ip_index = ip[k]-1
                            vip = car["que"][ip_index]['state'][1]
                            xip = car["que"][ip_index]['state'][0]
                            CAV_e["v_tk"][ego["id"][1]][2 + k] = vip
                            CAV_e["x_tk"][ego["id"][1]][2 + k] = xip


                        for k in range(len(index)):
                            for j in range(len(index[k])):
                                if index[k][j] == -1:
                                    continue
                                else:
                                    ic_index = index[k][j] - 1
                                    vic = car["que"][ic_index]['state'][1]
                                    xic = car["que"][ic_index]['state'][0]
                                    CAV_e["v_tk"][ego["id"][1]][2 + k][j] = vic
                                    CAV_e["x_tk"][ego["id"][1]][2 + k][j] = xic

                        for k in range(len(ego['see'])):
                            if car['que'][ego['see'][k]]['lane'] == ego['lane']:
                                ip_seen_index = ego['see'][k]
                                vip_seen = car['que'][ip_seen_index]['state'][1]
                                xip_seen = car['que'][ip_seen_index]['state'][0]
                                CAV_e["v_tk"][ego["id"][1]][6][k] = vip_seen
                                CAV_e["x_tk"][ego["id"][1]][6][k] = xip_seen

                    ego['prestate'] = ego['state']

                    if runMode == 'True': 
                        ego['state'], ego['infeasibility'] = OCBF_time(simulation_step, ego, car['que'])
                    else: 
                        ego['state'], ego['infeasibility'] = OCBF_event(simulation_step, ego, car['que'], flags)

            for vehicle in car['order']:
                if trust:
                    vehicle = vehicle - 1
                    ego = car['que'][vehicle]
                    if ego['state'][0] <= ego['metric'][-1] and ~ego['mustleave']:
                        ego['trust'][1] = ego['trust'][0]
                        ego['score'], ego['trust'][0] = calculate_trust(ego, simulation_step, car['que'],
                                                                     car['table'], trust_threshold)

                    if update_class_k_function:
                        if len(ego['ip']) > 0:
                            if ego['ip'][0] != -1:  # first car in the list
                                ip_index = ego['ip'][0] - 1
                                ego['k_rear_end'] = np.max([car['que'][ip_index]['trust'][0] - s3, 0.2], axis = 0)

                        for k in range(len(ego['index'])):
                            if ego['index'][k][0] == -1:
                                continue
                            else:
                                ic_index = ego['index'][k][0] - 1
                                ego['k_lateral'][k] = np.max([car['que'][ic_index]['trust'][0] - s3, 0.2], axis = 0)

                    if (simulation_step >= CAV_e['arrivalexit'][vehicle-1, 0] /dt + 20) and \
                        (ego['trust'][0] <= trust_threshold['low']) and (ego['trust'][0] - ego['trust'][1] <= 0.01):
                        ego['warning1'] += 1
                    else:
                        ego['warning1'] = 0

                    if ego['trust'][0] < trust_threshold['low']:
                        ego['warning2'] += 1

                    if (ego['warning1'] >= 40 or ego['warning2'] >= 60) and mitigation:
                        ego['mustleave'] = 1

            if mitigation:
                car['order'] = mitigation_function(car['que'], car['table'], car['order'], CAV_e)
                car = update_table(car)

            # update the position of each vehicle
            for vehicle in car['order']:
                vehicle = int(vehicle) - 1
                ego = car['que'][vehicle]
                position = getxy.getXY(ego['lane'], ego['decision'],ego['state'][0],ego['prestate'][0],ego['j'],
                                       ego['realpose'], ego['prerealpose'])
                positionX = position[0]
                positionY = position[1]
                angle = position[2]
                ego['j'] = position[3]
                ego['prerealpose'] = ego['realpose']
                ego['realpose'] = [position[0], position[1]]

                if ego['state'][0] < ego["metric"][4] and ego['traciID'] != -1:
                    traci.vehicle.moveToXY(ego['traciID'], "", -1, positionX, positionY, angle)


            real_cars_id_traci = [int(element) for element in traci.vehicle.getIDList() if element != 'carla0']
            spoofed_cars_id = []
            real_cars_id_queue = []
            curr_que_id = []

            for kk in range(len(car['que'])):
                if car['que'][kk]['traciID'] == -1:
                    if car['que'][kk]['state'][0] < car['que'][kk]['metric'][-1] + L_end:
                        spoofed_cars_id.append(car['que'][kk]['id'][1])
                else:
                    if car['que'][kk]['traciID'] in real_cars_id_traci:
                        real_cars_id_queue.append(car['que'][kk]['id'][1])
                curr_que_id.append(car['que'][kk]['id'][1])
            updated_cars_id = real_cars_id_queue + spoofed_cars_id

            if len(updated_cars_id) - len(curr_que_id) < 0:
                set0 = set(curr_que_id)
                set2 = set(updated_cars_id)
                id_left_car = list(set0 - set2)
                id_left_car.sort(reverse=True)
                queue_row = []
                for i in id_left_car:
                    CAV_e['arrivalexit'][id_left_car, 1] = dt * simulation_step
                    for k in range(car['cars']):
                        if car['que'][k]['id'][1] == i:
                            queue_row.append(k)
                            break
                queue_row.sort(reverse=True)
                for i in range(len(queue_row)):
                    mask = car['order'] == queue_row[i] + 1  # Create a mask for the element to remove
                    index_in_order = np.where(mask)[0]  # Get the index where the element is located
                    car['order'] = np.delete(car['order'], index_in_order)  # Remove the element from the array
                    car['order'] = np.array([orders - 1 if orders > queue_row[i] + 1 else orders for orders in car['order']])


                car['cars'] -= len(curr_que_id) - len(updated_cars_id)
                car['que'] = [item for item in car['que'] if item['id'][1] in updated_cars_id]
                car = update_table(car)


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
    argparser.add_argument('--accident',
                           type=str,
                           help="select wethear to simulate attack resulting in acceident or vice versa.",
                           default='False')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()
    runMode = arguments.accident

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)


    # CAV_e = synchronization_loop(arguments)
    # pointer = 12
    # total = 12
    # dt = 0.1
    # fig_states, axs_states = plt.subplots(3, figsize=(8, 10))
    # # fig_constraint, (ax_const1, ax_const2) = plt.subplots(1, 2, figsize=(10, 4))
    # plt.figure(figsize=(6, 4))
    # while pointer <= total:
    #     indicies1 = np.where(~np.isnan(CAV_e['acc'][:, pointer]))[0]
    #
    #     time_samples = dt * indicies1
    #     acceleration = CAV_e['acc'][indicies1, pointer]
    #     velocity = CAV_e['vel'][indicies1, pointer]
    #     position = CAV_e['pos'][indicies1, pointer]
    #     axs_states[0].plot(time_samples, acceleration, linestyle='-')
    #     axs_states[0].set_xlabel('Time')
    #     axs_states[0].set_ylabel('Acceleration')
    #
    #     axs_states[1].plot(time_samples, velocity, linestyle='-')
    #     axs_states[1].set_xlabel('Time')
    #     axs_states[1].set_ylabel('Velocity')
    #
    #     axs_states[2].plot(time_samples, position, linestyle='-')
    #     axs_states[2].set_xlabel('Time')
    #     axs_states[2].set_ylabel('Velocity')
    #
    #     indicies2 = np.where(~np.isnan(CAV_e['rear_end'][:, pointer]))[0]
    #     time_samples = dt * indicies2[0:-1]
    #     rear_end_constraint = CAV_e['rear_end'][indicies2[0:-1], pointer]
    #     plt.plot(time_samples, rear_end_constraint, label='rear-end-constraint', color='green')
    #     plt.savefig('time-drivenplot.eps', format='eps')
    #     plt.title('Rear End Constraint')
    #     plt.xlabel('Time')
    #     plt.ylabel('Value')
    #     plt.legend()
    #     pointer += 1
    #
    # plt.tight_layout()
    # plt.show()

