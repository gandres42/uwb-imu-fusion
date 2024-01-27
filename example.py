import decawave_ble
import logging

logger = logging.getLogger(__name__)

MINIMAL_CONFIG = {
    'Anchor': [
        'network_id',
        'device_type_name',
        'uwb_mode_name',
        'initiator',
        'x_position',
        'y_position',
        'z_position'],
    'Tag': [
        'network_id',
        'device_type_name',
        'uwb_mode_name',
        'moving_update_rate',
        'stationary_update_rate']}

# Function for configuring multiple devices from a database
def configure_devices_from_database(configuration_database):
    logger.info('Getting target device names')
    target_device_names = configuration_database.get_target_device_names()
    logger.info('Target device names: {}'.format(target_device_names))
    logger.info('Scanning for Decawave devices')
    devices = decawave_ble.scan_for_decawave_devices()
    logger.info('Found {} Decawave devices'.format(len(devices)))
    device_names = list(devices.keys())
    target_devices_not_present = set(target_device_names) - set(device_names)
    if len(target_devices_not_present) > 0:
        raise ValueError('Target devices not found: {}'.format(target_devices_not_present))
    else:
        logger.info('Found all target devices')
    present_devices_not_targeted = set(device_names) - set(target_device_names)
    if len(present_devices_not_targeted) > 0:
        logger.warning('Devices found but not mentioned in config data: {}'.format(present_devices_not_targeted))
    else:
        logger.info('All found devices are mentioned in config data')
    for target_device_name in target_device_names:
        logger.info('Getting target data for {}'.format(target_device_name))
        target_data = configuration_database.get_target_data(target_device_name)
        logger.info('Target data: {}'.format(target_data))
        device_type_name = target_data.get('device_type_name')
        if device_type_name is None:
            raise ValueError('Device type must be specified when configuring devices')
        if device_type_name not in MINIMAL_CONFIG.keys():
            raise ValueError('Device type {} not recognized'.format(device_type_name))
        minimal_config = MINIMAL_CONFIG[device_type_name]
        specified_fields =[]
        for key in target_data.keys():
            if target_data.get(key) is not None:
                specified_fields.append(key)
        if not set(minimal_config).issubset(specified_fields):
            logger.warning('Device type for {} is set to {} but some important fields are not specified in config: {}'.format(
                target_device_name,
                device_type_name,
                list(set(minimal_config) - set(specified_fields))))
        if target_data.get('network_id') is not None:
            try:
                network_id = int(target_data.get('network_id'))
            except:
                try:
                    network_id = int(target_data.get('network_id'), base=0)
                except:
                    raise ValueError('Network ID {} is not of a recognized type'.format(network_id))
        logger.info('Writing target data')
        decawave_ble.set_config(
            devices[target_device_name],
            device_type_name=target_data.get('device_type_name'),
            uwb_mode_name=target_data.get('uwb_mode_name'),
            accelerometer_enable=target_data.get('accelerometer_enable'),
            led_enable=target_data.get('led_enable'),
            initiator=target_data.get('initiator'),
            low_power_mode=target_data.get('low_power_mode'),
            location_engine=target_data.get('location_engine'),
            network_id=network_id,
            moving_update_rate=target_data.get('moving_update_rate'),
            stationary_update_rate=target_data.get('stationary_update_rate'),
            x_position=target_data.get('x_position'),
            y_position=target_data.get('y_position'),
            z_position=target_data.get('z_position'),
            quality=target_data.get('quality'),
            check_config_enabled = True)
        logger.info('Configuration is set correctly')
