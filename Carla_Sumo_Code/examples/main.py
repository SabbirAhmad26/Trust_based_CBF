#!/usr/bin/env python

import os
import sys
import optparse

# we need to import some python modules from the $SUMO_HOME/tools directory
8873
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # Checks for the binary in environ vars
import traci


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


# contains TraCI control loop
def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(step)

        # if step == 100:
        #     traci.vehicle.changeTarget("1", "e9")
        #     traci.vehicle.changeTarget("3", "e9")

        step += 1

    traci.close()
    sys.stdout.flush()


# 8873
# main entry point
if __name__ == "__main__":
    options = get_options()
    #sumoBinary = '/home/akua/opt/carla-simulator/Co-Simulation/Sumo/examples/sumo-gui'
    sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    sumoCmd = [sumoBinary, "-c", "Town05.sumocfg"]
    traci.start(sumoCmd, port=8813, label="sim1")
    conn1 = traci.getConnection("sim1")
    conn1.setOrder(1)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        conn1.simulationStep()
        # traci.vehicle.changeTarget("1", "-9.0.00")
        # vehID = "1"
        # edgeID = "45.0.00"
        # lane = 4
        # x = 91.45
        # y = 336.89
        if step == 1:
            phi = 0
            pos = traci.vehicle.getPosition("1")
            x = pos[0]
            y = pos[1]
            # traci.vehicle.moveToXY("1", "45.0.00", -1, x, y, keepRoute=1)
            # traci.vehicle.setSpeed("1", 0.1)
            # traci.vehicle.moveTo("1", "3", 103.1)
            # acc=traci.vehicle.setAccel("vehicle.carlamotors.carlacola", 0.2)
            pos2 = (pos[0] + 2, pos[1])
            traci.vehicle.moveToXY('1', "45.0.00", -1, pos2[0], pos2[1], 0, keepRoute=1)
            # traci.vehicle.moveToXY("1", "45.0.00", 4, x, y+2, phi, keepRoute=1)
            # traci.vehicle.moveToXY("1", "45.0.00", -1, x, y, keepRoute=1)
        step += 1
    conn1.close()

    traci.init(port=8813, label="sim2")
    conn2 = traci.getConnection("sim2")
    conn2.setOrder(2)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        conn2.simulationStep()
        step += 1
    conn2.close()
