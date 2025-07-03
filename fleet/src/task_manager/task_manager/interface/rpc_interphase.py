"""
1. Del hub package acceptance
2. Feeder package dump 
3. Available del hubs


"""

from task_manager.interface.proto_gen import del_hub_availability_pb2, del_hub_availability_pb2_grpc
from task_manager.interface.proto_gen import del_hub_pack_accept_pb2, del_hub_pack_accept_pb2_grpc
import grpc

class RPCInterface:
    """
    RPC interface to call the communication hub
    """
    RPC_ADDRESS = "[::1]:50051"

    def __init__(self, **kwargs):
        """Initialize the gRPC channel and PackageStatus stub."""
        self.channel = grpc.insecure_channel(self.RPC_ADDRESS)
        self.avail_stub = del_hub_availability_pb2_grpc.DeliveryHubStatusStub(self.channel)
        self.accept_stub= del_hub_pack_accept_pb2_grpc.PackageStatusStub(self.channel)
        print("RPC interface initialized successfully")
        
    def del_hub_availablity_rpc(self,del_hub_id: str) -> bool:
        try:
################## check for server availablity before calling the RPC

            request = del_hub_availability_pb2.DeliveryHubAvailablityReq(del_hub_id=del_hub_id)
            resp = self.avail_stub.robotToCommHub(request)

            return resp.status
        except Exception as e:
            print(f"Exception while calling del_hub_availablity_rpc: {e}")
            return False
    

    def pack_accpet_rpc(self,robot_id : str, del_hub_id: str) -> bool:
        try:
            request = del_hub_pack_accept_pb2.PackageAcceptanceReq(
                robot_id     = robot_id,
                del_hub_id   = del_hub_id,
            )
            resp = self.accept_stub.robotToServer(request)

            # Perform the RPC and return the response
            return resp.accepted
        
        except Exception as e:
            print(f"Exception occurred during package acceptance RPC: {e}")
            return False
