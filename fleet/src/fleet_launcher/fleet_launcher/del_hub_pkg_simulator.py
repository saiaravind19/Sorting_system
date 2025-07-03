import rclpy
from rclpy.node import Node
import time 
from std_msgs.msg import String
from pymodbus.client import ModbusTcpClient

class CoilToggler(Node):
    def __init__(self):
        super().__init__('coil_toggler')

        # Connect to Modbus server
        self.client = ModbusTcpClient("localhost", port=5020)
        while not self.client.connect():
            self.get_logger().warning(f'Failed to connect to Modbus at localhost:5020',throttle_duration_sec=5.0)
            time.sleep(5) # use rclpy timer 

        self.sub = self.create_subscription(String,'package_drop',self.listener_callback,2)

    def listener_callback(self, msg: String):


        split_str = msg.data.split('_')
        slave_id = int(split_str[-1])
        coil_values = self.client.read_coils(0, 6, slave=slave_id)
        
        coil_values = coil_values.bits[:6]

        try:
            free_coil = coil_values.index(False)
        except ValueError:
            # no False found → all True
            self.get_logger().info(f'All coils are already full for del_hub {msg.data}')
            return
        self.client.write_coil(address=free_coil, value=True, slave=slave_id)

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CoilToggler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
