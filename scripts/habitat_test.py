#!/usr/bin/env python3


import rospy
import habitat
from geometry_msgs.msg import Twist
from habitat.config.read_write import read_write


def main():
    # Define the ROS topic to subscribe to
    topic = "chatter"

    # Define the Habitat scene to use
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"

    # Define the Habitat agent configuration
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")
    # agent_config.defrost()
    # agent_config.SIMULATOR.SCENE = scene
    # agent_config.freeze()
    with read_write(agent_config):
        agent_config.habitat.simulator.scene = scene
    print(agent_config.habitat.simulator.scene)

    # Define the ROS callback function
    def callback(data):
        # Convert the ROS twist message to a Habitat action
        action = {
            "action": "move",
            "amount": data.linear.x,
            "rotation": data.angular.z,
        }

        # Move the agent in Habitat
        observations = env.step(1)
        print("node was reached")

    # Initialize the ROS node and subscriber
    rospy.init_node("habitat_ros_bridge")
    rospy.Subscriber(topic, Twist, callback)

    # Create the Habitat environment and agent
    env = habitat.Env(agent_config)
    observations = env.reset()
    # Start the ROS spin loop
    rospy.spin()
    print("loaded")

if __name__ == "__main__":
    main()
