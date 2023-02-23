import time
import struct
import numpy as np
from matplotlib import pyplot as plt
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.debug.elf.symbols import ELFSymbolProvider


# Replace ST_LINK_ID with the ID of your st-link, on the command promt
# enter "pyocd list" to get the ID.
ST_LINK_ID='002900364741500520383733'
options = {'connect_mode': 'attach', 'target_override': 'stm32g474retx'}
session = ConnectHelper.session_with_chosen_probe(ST_LINK_ID, options=options)
session.open()
session.target.elf = '.pio/build/owntech_power_converter/firmware.elf'
provider = ELFSymbolProvider(session.target.elf)
addr = provider.get_symbol_value("record_array") 
RECORD_SIZE = 2047
curves = [
        {'name': 'I1_low_value', 'format': 'f',},
        {'name': 'I2_low_value', 'format': 'f',},
        {'name': 'V1_low_value', 'format': 'f',},
        {'name': 'V2_low_value', 'format': 'f',},
        {'name': 'Vhigh_value', 'format': 'f',},
        {'name': 'Ihigh_value', 'format': 'f',}
        ]

print(f"record_slave addr = {addr:x}")


tic = time.time()
datas = session.target.read_memory_block32(addr, 6*RECORD_SIZE) 
toc = time.time()
print(f"time : {toc-tic}")
datas = np.reshape(datas, (-1, len(curves)))
results = {}
for k, curve in enumerate(curves):
    results[curve['name']] = [struct.unpack(curve['format'], struct.pack('I', d))[0] for d in datas[:, k]]


fig, axs = plt.subplots(2,sharex=True)

axs[0].plot(results['V1_low_value'],label = 'V1_low_value',zorder=2)
axs[0].plot(results['V2_low_value'],label = 'V2_low_value',zorder=1)
axs[0].plot(results['Vhigh_value'],label = 'Vhigh_value')
axs[0].set(ylabel='voltage')
axs[0].legend()

axs[1].plot(results['I1_low_value'],label = 'I1_low_value')
axs[1].plot(results['I2_low_value'],label = 'I2_low_value')
axs[1].plot(results['Ihigh_value'],label = 'I2_low_value')
axs[1].set(ylabel='current')
axs[1].legend()

plt.show()

session.close()

        
