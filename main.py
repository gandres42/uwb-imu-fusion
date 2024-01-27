import decawave_ble
from threading import Thread, Lock

def set_nodes(anchors, tags):
    def set_anchor(node):
        global complete_anchors, complete_anchors_lock
        decawave_ble.set_config(node, device_type_name='Anchor', uwb_mode_name='Active', initiator=True, low_power_mode=False, accelerometer_enable=False)
    
    def set_tag(node):
        global complete_anchors, complete_anchors_lock
        decawave_ble.set_config(node, device_type_name='Tag', uwb_mode_name='Active', initiator=True, low_power_mode=False, accelerometer_enable=False, moving_update_rate=100)

    node_threads = []
    if anchors is not None:
        for node in anchors:
            node_threads.append(Thread(target=set_anchor, args=(node, )))

    if tags is not None:
        for node in tags:
            node_threads.append(Thread(target=set_tag, args=(node, )))

    for node_thread in node_threads:
        node_thread.start()

    for node_thread in node_threads:
        node_thread.join()

# expected devices in network
anchors = ['DWD100', 'DW0F07', 'DWC09D', 'DW1737', 'DW0F07']
tags = ['DWCD14']

# prune devices that aren't found
print("scanning for nodes...")
discovered_devices = decawave_ble.scan_for_decawave_devices()
for device in anchors:
    if device not in discovered_devices.keys():
        anchors.remove(device)
for device in tags:
    if device not in discovered_devices.keys():
        tags.remove(device)
print("found ", end=str(anchors + tags) + "\n")

if len(anchors) < 4:
    raise RuntimeError("insufficient anchor count, less than 4 anchors detected")


node_distances = {}
for node_a in anchors + tags:
    node_distances[node_a] = {}
    for node_b in anchors + tags:
        if node_a != node_b:
            node_distances[node_a][node_b] = None

# take turns setting node as tag and all others as anchor to get distances
# print("setting all nodes to tags")
# set_nodes(None, discovered_devices.values())
# print('done!')


for node_a in discovered_devices.values():
    anchors = []
    for node_b in discovered_devices.values():
        if node_a != node_b:
            anchors.append(node_b)
    set_nodes(anchors, [node_a])
# use distances to estimate anchor poses