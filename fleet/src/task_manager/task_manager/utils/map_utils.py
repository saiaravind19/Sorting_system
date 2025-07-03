import json


# other ideas -> instead of config we can map the robot postion with grid type
# Need better indexing for grid mapping blocking and unblocking them


class map_utils:
    """
    1. Store the special grid types and their coordinates.
    2. Transformation from map frame to robot frame

    """
    def __init__(self,grid_filename):
        try:
            with open(grid_filename,'r') as json_file:
                self.__gridtype_to_coord = json.load(json_file)
        except Exception as e :
            print("Exception in map utils :",e)

        self.grid_mapping = self.map_grid()
        self.callbback_mapper = {"feeding_grid" : self.get_available_feeding_grids,
                                 "dumping_grid" : self.get_available_dumpding_grids,
                                 "queuing_grid" : self.get_available_queuing_grids
                                 }


    def map_grid(self):

        # limitation currently only handling one queue and one feeding grid in the layout
        # Can use basic math +/- 1 from x, y check the nearest queting grid with dist of 1
        # Can expand this to handle multiple queuing grids
        
        grid_mapping = []

        # grab the three parallel lists
        feeding_list = self.__gridtype_to_coord.get("feeding_grid", [])
        dumping_list = self.__gridtype_to_coord.get("dumping_grid", [])
        queuing_list = self.__gridtype_to_coord.get("queuing_grid", [])
        hub_list = self.__gridtype_to_coord.get("hub_id", [])
        for f_coords, d_coords, q_coords ,h_id in zip(feeding_list, dumping_list, queuing_list, hub_list):
            template = {"feeding_grid": {"coordinates": [] , "status": None} ,
                        "dumping_grid" : {"coordinates": [] , "status": None , "hub_id": None},
                        "queuing_grid" : {"coordinates": [] , "status": None}
                        }     
            template["feeding_grid"]["coordinates"] = f_coords
            template["feeding_grid"]["status"]      = None

            template["dumping_grid"]["coordinates"] = d_coords
            template["dumping_grid"]["status"]      = None
            template["dumping_grid"]["hub_id"]      = h_id

            template["queuing_grid"]["coordinates"] = q_coords
            template["queuing_grid"]["status"]      = None
            
            grid_mapping.append(template)
        
        return grid_mapping

    def get_feeding_grids(self) -> list:
        return self.__gridtype_to_coord["feeding_grids"]

    def get_dumping_grids(self) -> list:
        return self.__gridtype_to_coord["dumping_grids"]
    
    def get_queue_grids(self) -> list:
        return self.__gridtype_to_coord["queue_grids"]

################feeding grids #####################
    def get_available_feeding_grids(self):
        available_feeding_coords = []
        for map in self.grid_mapping:
            if map["feeding_grid"]["status"] is None:
                available_feeding_coords.append(
                    map["feeding_grid"]["coordinates"]
                )
        return available_feeding_coords
    
    def block_feeding_grid(self,grid : list,robot_id : str) -> None:
        for map in self.grid_mapping:
            if map["feeding_grid"]["coordinates"] == grid:
                map["feeding_grid"]["status"] =  robot_id

    def unblock_feeding_grid(self,grid : list) -> None:
        for map in self.grid_mapping:
            if map["feeding_grid"]["coordinates"] == grid:
                map["feeding_grid"]["status"] =  None



##############dumping grid ######################
    def get_available_dumpding_grids(self) -> None:
        available_dumping_coords = []
        for map in self.grid_mapping:
            if map["dumping_grid"]["status"] is None:
                available_dumping_coords.append(
                    map["dumping_grid"]["coordinates"]
                )
        return available_dumping_coords
      
    def block_dumping_grid(self,grid : list,robot_id : str) -> None:
        for map in self.grid_mapping:
            if map["dumping_grid"]["coordinates"] == grid:
                map["dumping_grid"]["status"] =  robot_id

    def unblock_dumping_grid(self,grid : list) -> None:
        for map in self.grid_mapping:
            if map["dumping_grid"]["coordinates"] == grid:
                map["dumping_grid"]["status"] =  None

    def get_current_hub_id(self,grid : list) -> str:
        for mapping_grid in self.grid_mapping:
            if grid == mapping_grid["dumping_grid"]["coordinates"]:
                return mapping_grid["dumping_grid"]["hub_id"] 
        return None
    
    def get_respective_dumping_grid(self,grid : list):
        for mapping_grid in self.grid_mapping:
            if grid == mapping_grid["feeding_grid"]["coordinates"]:
                return mapping_grid["dumping_grid"]["coordinates"],mapping_grid["dumping_grid"]["hub_id"] 
        return None, None
    
    def check_dumping_grid_status(self,grid : list) -> bool:
        """
        Check if the dumping grid is available or not
        """
        for mapping_grid in self.grid_mapping:
            if grid == mapping_grid["dumping_grid"]["coordinates"]:
                return mapping_grid["dumping_grid"]["status"] is None
        return False
    
################# Queuing grid ##########################
    def get_available_queuing_grids(self) -> list:
        available_queuing_coords = []
        for map in self.grid_mapping:
            if map["queuing_grid"]["status"] is None:
                available_queuing_coords.append(
                    map["queuing_grid"]["coordinates"]
                )
        return available_queuing_coords
      
    def block_queuing_grid(self,grid : list,robot_id : str) -> None:
        for entry in self.grid_mapping:
            if entry["queuing_grid"]["coordinates"] == grid:
                entry["queuing_grid"]["status"] =  robot_id

    def unblock_queuing_grid(self,grid : list) -> None:
        for entry in self.grid_mapping:
            if entry["queuing_grid"]["coordinates"] == grid:
                entry["queuing_grid"]["status"] =  None


    def get_nearest_available_grid(self,grid : list ,grid_type :str):
        """ 
        Prefered to give the current grid
        """
        if grid_type in self.__gridtype_to_coord.keys():
            nearest_coord = None
            min_dist = float('inf')
            available_grids = self.callbback_mapper[grid_type]()
            for f_coord in available_grids :
                dist = abs(f_coord[0] - grid[0]) + abs(f_coord[1] - grid[1])
                if dist < min_dist:
                    min_dist = dist
                    nearest_coord = f_coord
            return nearest_coord
        
        return None
    