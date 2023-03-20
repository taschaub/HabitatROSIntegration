#!/usr/bin/env python3


import rospy
import habitat
from publish_test.msg import BasicAction
from habitat.config.read_write import read_write


def main():
    # Define the ROS topic to subscribe to
    topic = "chatter"

    # Define the Habitat scene to use
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"

    # Define the Habitat agent configuration
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    with read_write(agent_config):
        agent_config.habitat.simulator.scene = scene
    print(agent_config.habitat.simulator.scene)
    
    # Create the Habitat environment and agent
    env = habitat.Env(agent_config)


    # Define the ROS callback function
    def callback(data, env):
        print(env)
        # Move the agent in Habitat
        print(data.ActionIdx)
        # observations = env.step(1)
        # 
        # print("Node performed {data.Action}")

    # Initialize the ROS node and subscriber
    rospy.init_node("habitat_ros_bridge")
    
    rospy.Subscriber(topic, BasicAction, callback, callback_args=env)
   
    observations = env.reset()
    # Start the ROS spin loop
    print("loaded")
    rospy.spin()
    

if __name__ == "__main__":
    main()
