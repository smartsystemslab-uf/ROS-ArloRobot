#! /usr/bin/env/python3

from pynq import Overlay
from pynq.pl_server.device import Device
import json

overlay = Overlay("/home/xilinx/RC_Overlays/Top.bit")
print(overlay.ip_dict.keys())

uart_addr = overlay.ip_dict['axi_uartlite_1']['phys_addr']

gpio_addr = overlay.ip_dict['axi_gpio_1']['phys_addr']
gpio_addr_range = overlay.ip_dict['axi_gpio_1']['addr_range']

mem_mapped = Device.active_device.has_capability('MEMORY_MAPPED')
register_rw = Device.active_device.has_capability('REGISTER_RW')

ip_addr_dict = {'uart_addr': uart_addr, 'gpio_addr': gpio_addr, 'gpio_addr_range': gpio_addr_range, 'mem_mapped': mem_mapped, 'register_rw': register_rw}

for key in ip_addr_dict:
    print(key, ip_addr_dict[key])
    
print('LOADED OVERLAY')
config_file = open('ip_addr.config', 'w+')
json.dump(ip_addr_dict, config_file)
