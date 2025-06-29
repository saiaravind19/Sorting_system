import asyncio
import random
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from enum import Enum
from global_param import deliveryHubParam



"""
ToDo:
    - Add hub ID so that we can spawn multiple instances of the code from bash file for simulation.
    - Add logging along with unique name.

Simulator:
    - Add a client code which monitors the robot request for parcel accpetance and updates the delivary hub -> simlating the parcel dumping.
"""

class modBusDatatype(Enum):
    COIL = 1
    DISCRETE_INPUT = 2
    HOLDING_REGISTER = 3
    INPUT_REGISTER = 4



class DeliveryHub:
    def __init__(self):
        self.store = ModbusSlaveContext(
            # Bins / slots
            co=ModbusSequentialDataBlock(0, [0]*deliveryHubParam.NUM_BINS), 
            #using holding register so that I can reset the values if needed ( mimic the sesison restart)
            hr=ModbusSequentialDataBlock(0, [0]), # stores the number of packages sorted
            zero_mode=True  # Start the Modbus registers from 0

        )
        self.context = ModbusServerContext(slaves=self.store, single=True)


    async def free_slot(self,slot_id):
        await asyncio.sleep(random.uniform(1, 5))
        self.store.setValues(1, slot_id, [0])
        print(f"[Hub] Package accepted Slot {slot_id + 1} clearing the slot.")
        count = self.store.getValues(3, 0)[0]
        print("Current package_count:",count)

    async def monitor_triggers(self):
        prev = [0]*deliveryHubParam.NUM_BINS
        while True:
            current = self.context[0].getValues(modBusDatatype.COIL.value,0,deliveryHubParam.NUM_BINS)
            for i in range(len(current)):
                if current[i] == 1 and prev[i] == 0:
                    count = self.store.getValues(3, deliveryHubParam.PACKAGE_COUNT_REG)[0]
                    self.store.setValues(3, deliveryHubParam.PACKAGE_COUNT_REG, [count + 1])
                    print(f"[Hub] Request for package acceptance for slot {i + 1} received.")
                    asyncio.create_task(self.free_slot(i))
            prev = current
            await asyncio.sleep(0.1)


    async def start(self):
        """
        Starts the Modbus TCP server and monitoring loop concurrently.
        """
        await asyncio.gather(
            StartAsyncTcpServer(
                context=self.context,
                address=("localhost",5020),
            ),
            self.monitor_triggers(),
        )


if __name__ == "__main__":
    hub = DeliveryHub()
    asyncio.run(hub.start())
