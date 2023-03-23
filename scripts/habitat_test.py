#!/usr/bin/env python3

import rospy
import habitat
from publish_test.msg import BasicAction
from habitat.config.read_write import read_write
from threading import Thread
from queue import Queue


def habitat_thread(agent_config, scene, action_queue):
    with read_write(agent_config):
        agent_config.habitat.simulator.scene = scene
    print(agent_config.habitat.simulator.scene)

    env = habitat.Env(agent_config)
    observations = env.reset()

    while not rospy.is_shutdown():
        if not action_queue.empty():
            action = action_queue.get()
            print(action)
            observations = env.step(action)
            print(observations)


def main():
    topic = "chatter"
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    action_queue = Queue()

    ht = Thread(target=habitat_thread, args=(agent_config, scene, action_queue))
    ht.start()

    def callback(data):
        print("Action received:", data.ActionIdx)
        action_queue.put(data.ActionIdx)

    rospy.init_node("habitat_ros_bridge")
    rospy.Subscriber(topic, BasicAction, callback)
    print("loaded")
    rospy.spin()


if __name__ == "__main__":
    main()
