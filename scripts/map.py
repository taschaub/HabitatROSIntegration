import cv2
import math
import numpy as np
from PIL import Image as PilImage
from habitat.utils.visualizations import maps
import yaml

def display_top_down_map(sim):
    
    # agent_state = sim.get_agent_state()
    agent = sim.get_agent(0)
    agent_state = agent.state
    top_down_map = maps.get_topdown_map(sim.pathfinder, meters_per_pixel=0.05, height=0.5)
    top_down_map = maps.colorize_topdown_map(top_down_map)

    # Draw the agent's position and orientation on the map
    agent_pos = agent_state.position
    agent_rot = agent_state.rotation
    
    # Get agent's yaw (rotation around the Z-axis) and convert to (-pi, pi)
    agent_angle = -2 * math.atan2(agent_rot.w, agent_rot.y)  

    # Convert the agent position to integer values
    grid_resolution = 0.05
    bounds = sim.pathfinder.get_bounds()
    lower_bounds = bounds[0][[0, 2]]
    upper_bounds = bounds[1][[0, 2]]

    # Calculate the size of the map
    map_size = (upper_bounds - lower_bounds) / grid_resolution

    # Convert the agent position to grid coordinates
    agent_pos_grid = (agent_pos[[0, 2]] - lower_bounds) / grid_resolution
    agent_pos_int = np.round(agent_pos_grid).astype(np.int32)
    agent_pos_int_swapped = [agent_pos_int[1],agent_pos_int[0]]

    top_down_map_with_agent = maps.draw_agent(
        top_down_map,
        agent_pos_int_swapped,
        agent_angle,
        agent_radius_px=8,  # Adjust the agent_radius_px for the size of the agent marker
        # agent_color=(0, 0, 255)
    )
    cv2.imwrite("top_down_map.png", top_down_map_with_agent)
    #cv2.imshow("Top Down Map", top_down_map_with_agent)
    cv2.waitKey(1)
    
    top_down_map = cv2.cvtColor(top_down_map, cv2.COLOR_BGR2GRAY)
    top_down_map = cv2.normalize(top_down_map, None, 0, 255, cv2.NORM_MINMAX)

    save_image_and_yaml(top_down_map, 'test_map.pgm', 'test_map.yaml')

def save_image_and_yaml(top_down_map, pgm_filename, yaml_filename, resolution=0.05, origin=[0.0, 0.0, 0.0]):
    image = PilImage.fromarray(top_down_map)
    image.save(pgm_filename)

    # Create yaml file
    yaml_data = dict(
        image=pgm_filename,
        resolution=resolution,
        origin=origin,
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
    )

    with open(yaml_filename, 'w') as outfile:
        yaml.dump(yaml_data, outfile, default_flow_style=False)
