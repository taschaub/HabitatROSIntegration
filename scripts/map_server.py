import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData

class MapServer:
    def __init__(self):
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.map_data = None  # Initialize map data as None

    def update_map_data(self, map_data):
        self.map_data = map_data  # Update the map data

    def publish_map(self):
        if self.map_data is None:
            rospy.logwarn('Map data is not yet available.')
            return
        
        map_msg = self.create_map_msg()
        self.map_pub.publish(map_msg)

    def create_map_msg(self):
        map_msg = OccupancyGrid()
        map_meta_data = MapMetaData()

        # Set metadata (you need to fill in your values here)
        map_meta_data.width = self.map_data.shape[1]
        map_meta_data.height = self.map_data.shape[0]
        map_meta_data.resolution = 0.02  # Depends on your map's resolution
        map_meta_data.origin.position.x = -5  # Depends on your map's origin
        map_meta_data.origin.position.y = -5  # Depends on your map's origin

        map_msg.info = map_meta_data

        # Flatten the map data and fill in the data field of the message
        map_msg.data = (self.map_data.flatten() * 100).astype(int).tolist()

        return map_msg