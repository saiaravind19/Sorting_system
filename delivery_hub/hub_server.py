import asyncio
import random
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from enum import Enum
import argparse
import logging

logging.basicConfig(level= logging.INFO,format="[%(levelname)s]:[%(name)s]:[%(funcName)s]:%(message)s")


##############Add host ip and port to be taken from command line arguments##############


"""
ToDo:

Simulator:
    - Add a client code which monitors the robot request for parcel accpetance and updates the delivary hub -> simlating the parcel dumping.


Its a simulator for multiple delivery hubs
As we are running on same machine we can have multiple slave id bind to the same port.

In real life scenario we can have multiple delivery hubs running on different machines the differnece is the they have different IPs but same port.


"""

class deliveryHubParam():
    NUM_BINS = 6
    PACKAGE_COUNT_REG = 0 
    SLAVE_COUNT = 1  
    TIME_PERIOD = 5

class modBusDatatype(Enum):
    COIL = 1
    DISCRETE_INPUT = 2
    HOLDING_REGISTER = 3
    INPUT_REGISTER = 4


class DeliveryHub:
    def __init__(self,num_slaves=1):

        self.logger = logging.getLogger(f"DeliveryHub")

        # create multiple instances of ModbusSlaveContext for each delivery hub
        self.store = {
                id :ModbusSlaveContext(
                    # Bins / slots
                    co=ModbusSequentialDataBlock(0, [0]*deliveryHubParam.NUM_BINS), 
                    #using holding register so that I can reset the values if needed ( mimic the sesison restart)
                    hr=ModbusSequentialDataBlock(0, [0]), # stores the number of packages sorted
                    zero_mode=True  # Start the Modbus registers from 0
                    )
                    for id in range(0,num_slaves)
        }
        self.context = ModbusServerContext(slaves=self.store, single=False)
        self.logger.info(f'Intialized {num_slaves} delivery hubs with slave IDs: {list(self.store.keys())}')

    async def free_slot(self,hub_id,slot_id):
        await asyncio.sleep(random.uniform(2, deliveryHubParam.TIME_PERIOD))
        self.store[hub_id].setValues(1, slot_id, [0])
        self.logger.info(f"Package accepted hub_id : {hub_id} Slot: {slot_id + 1} clearing the slot.")
        count_reg = self.store[hub_id].getValues(3, 0)
        count = count_reg[0]
        self.logger.info(f"Current package_count: {count}")

    async def monitor_triggers(self):
        prev = {
            hub_id:[0]*deliveryHubParam.NUM_BINS
            for hub_id in self.store.keys()
            }
        while True:
            # get the state of the coil per hub
            for hub_id,slave_context in self.store.items():

                current = slave_context.getValues(modBusDatatype.COIL.value,0,deliveryHubParam.NUM_BINS)
                for slot_id in range(len(current)):
                    if current[slot_id] == 1 and prev[hub_id][slot_id] == 0:
                        count = self.store[hub_id].getValues(3, deliveryHubParam.PACKAGE_COUNT_REG)[0]
                        self.store[hub_id].setValues(3, deliveryHubParam.PACKAGE_COUNT_REG, [count + 1])
                        self.logger.info(f"Request for package acceptance for hub : {hub_id} slot {slot_id + 1} received.")
                        asyncio.create_task(self.free_slot(hub_id,slot_id))
                prev[hub_id] = current
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
    parser = argparse.ArgumentParser(description ='Delivery Hub Server(Modbus Slaves) uses MODbus TCP.')

    parser.add_argument('--no_of_slaves', type = int, default = 4,
                        help ='Enter the number of ddelivery hubs to be simulated. Default is 1.')
    parser.add_argument('--time_period', type = int, default = 5, help ='TIme in seconds before the bin is cleared (random b/w 1 and time_period).')
    args = parser.parse_args()

    deliveryHubParam.SLAVE_COUNT = args.no_of_slaves
    deliveryHubParam.TIME_PERIOD = args.time_period


    hub = DeliveryHub(args.no_of_slaves)
    asyncio.run(hub.start())
