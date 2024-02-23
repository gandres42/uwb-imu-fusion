import decawave_ble
from threading import Thread
import time

def set_devices(tag, anchors):
    def set_anchor(device):
        decawave_ble.set_config(device, device_type_name='Anchor', uwb_mode_name='Active', initiator=True, location_engine=False)
    
    def set_tag(device):
        decawave_ble.set_config(device, device_type_name='Tag', uwb_mode_name='Active', initiator=True, location_engine=False)

    threads = []
    if anchors is not None:
        for node in anchors:
            threads.append(Thread(target=set_anchor, args=(node, )))
    threads.append(Thread(target=set_tag, args=(tag, )))

    for node_thread in threads:
        node_thread.start()
    for node_thread in threads:
        node_thread.join()

def get_location_data(device, count):
    data = []
    peripheral = decawave_ble.get_decawave_peripheral(device)
    for i in range(0, count):
        try:
            data.append(decawave_ble.get_location_data_from_peripheral(peripheral))
        except:
            peripheral = decawave_ble.get_decawave_peripheral(device)
        print(time.monotonic(), end=": ")
        print(data[-1])
    peripheral.disconnect()
    return data

start_time = time.monotonic()

# expected devices in network
friends = ['DW091D', 'DW1D21', 'DWCD14', 'DW0F07', 'DWD100', 'DW1737', 'DWC09D']

# identify scanned devices in the list of approved devices
print("scanning for nodes...")
scanned_devices = decawave_ble.scan_for_decawave_devices()
devices = {}
for stranger in scanned_devices.keys():
    if stranger in friends:
        devices[stranger] = scanned_devices[stranger]
print("using " + str(devices.keys()))

# measure distances between each peripheral
device_distances = {}
for node_a in devices.keys():
    device_distances[node_a] = {}
    for node_b in devices.keys():
        if node_a != node_b:
            device_distances[node_a][node_b] = []

# cycle through tag/anchor configurations
for tag in devices.items():
    print(tag[0])
    anchors = []
    for anchor in devices.items():
        if tag != anchor:
            anchors.append(anchor[1])
    set_devices(tag[1], anchors)
    print("config set")
    get_location_data(tag[1], 100)
    