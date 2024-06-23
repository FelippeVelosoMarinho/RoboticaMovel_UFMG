#!/usr/bin/python

'Example of UAVs represented as stations using CoppeliaSim'

import time
import os

from mininet.log import setLogLevel, info
from mn_wifi.link import wmediumd, adhoc
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.telemetry import telemetry
from mn_wifi.wmediumdConnector import interference

def create_data_directory():
    data_dir = 'examples/uav/data'
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

def topology():
    "Create a network."
    net = Mininet_wifi(link=wmediumd, wmediumd_mode=interference)

    info("*** Creating nodes\n")
    dr1 = net.addStation('dr1', mac='00:00:00:00:00:01', ip='10.0.0.1/8',
                         position='30,60,0')
    dr2 = net.addStation('dr2', mac='00:00:00:00:00:02', ip='10.0.0.2/8',
                         position='70,30,0')
    dr3 = net.addStation('dr3', mac='00:00:00:00:00:03', ip='10.0.0.3/8',
                         position='10,20,0')

    net.setPropagationModel(model="logDistance", exp=4.5)

    info("*** Configuring nodes\n")
    net.configureNodes()

    net.addLink(dr1, cls=adhoc, intf='dr1-wlan0',
                ssid='adhocNet', proto='batman_adv',
                mode='g', channel=5, ht_cap='HT40+')

    net.addLink(dr2, cls=adhoc, intf='dr2-wlan0',
                ssid='adhocNet', proto='batman_adv',
                mode='g', channel=5, ht_cap='HT40+')

    net.addLink(dr3, cls=adhoc, intf='dr3-wlan0',
                ssid='adhocNet', proto='batman_adv',
                mode='g', channel=5, ht_cap='HT40+')

    info("*** Starting network\n")
    net.build()

    nodes = net.stations
    telemetry(nodes=nodes, single=True, data_type='position')

    sta_drone = []
    for n in net.stations:
        sta_drone.append(n.name)
    sta_drone_send = ' '.join(map(str, sta_drone))

    # # set_socket_ip: localhost must be replaced by ip address
    # # of the network interface of your system
    # # The same must be done with socket_client.py
    info("*** Starting Socket Server\n")
    net.socketServer(ip='127.0.0.1', port=12345)

    info("*** Starting CoppeliaSim\n")
    path = os.path.dirname(os.path.abspath(__file__))
 
    coppelia_sim_path = '/home/felippeveloso/Documentos/GitHub/mininet-wifi/examples/uav/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu22_04'
    os.system(f'{coppelia_sim_path}/coppeliaSim.sh -s 10000 -f {path}/simulation.ttt -gGUIITEMS_2 &')
    time.sleep(10)

    info("*** Perform a simple test\n")
    simpleTest = f'python {path}/simpleTest.py ' + sta_drone_send + ' &'
    os.system(simpleTest)

    time.sleep(5)

    info("*** Configure the node position\n")
    setNodePosition = f'python {path}/setNodePosition.py ' + sta_drone_send + ' &'
    os.system(setNodePosition)

    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    kill_process()
    net.stop()

def kill_process():
    os.system('pkill -9 -f coppeliaSim')
    os.system('pkill -9 -f simpleTest.py')
    os.system('pkill -9 -f setNodePosition.py')
    if os.path.exists('examples/uav/data/'):
        os.system('rm examples/uav/data/*')

if __name__ == '__main__':
    setLogLevel('info')
    create_data_directory()
    kill_process()
    topology()
