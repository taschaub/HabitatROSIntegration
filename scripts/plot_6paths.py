import json
import pandas as pd
import ast
import yaml
import numpy as np
import matplotlib.pyplot as plt

def plot_and_save_episode_paths():
    """
    This script visualizes the navigation paths for a robot over a given map.
    
    Workflow:
    - Designed for 7 episodes.
    - Considers 3 different navigation algorithms.
    - Each episode has two starting angles (normal and rotated).
    
    Before Running:
    1. Ensure you have the required libraries installed: pandas, numpy, matplotlib, yaml.
    2. Update the file paths for episodes data, evaluation data, and map files.
    3. If the number of episodes or navigation algorithms changes, adjust the code accordingly.
    """
    
    # Load episode data
    with open("normal_rotated.json", "r") as file:
        all_episodes_data = json.load(file)

    # Extract episodes excluding the 8th one (divider episode)
    normal_episodes = all_episodes_data[:7]
    rotated_episodes = all_episodes_data[9:]

    # Load evaluation data for the three navigation algorithms
    eval_data_dwa = pd.read_csv("dwa6evaluation_data.csv")
    eval_data_rosnav = pd.read_csv("rosnav6evaluation_data.csv")
    eval_data_teb = pd.read_csv("teb6evaluation_data.csv")

    # Load map metadata and image
    with open("5LpN3gDmAk7.yaml", "r") as file:
        map_metadata = yaml.safe_load(file)
    map_image = plt.imread("5LpN3gDmAk7.pgm")

    # Define unique colors for each path
    path_colors = ["blue", "deepskyblue", "green", "limegreen", "red", "lightcoral"]

    # Function to plot paths on the map
    def plot_paths_on_map(data, map_image, ax, color):
        path = ast.literal_eval(data['path'])
        path = [(p[0] / map_metadata['resolution'], p[1] / map_metadata['resolution']) for p in path]
        path_x, path_y = zip(*path)
        
        ax.plot(path_x, path_y, color=color, linewidth=1.5)
        ax.scatter(path_x[0], path_y[0], s=40, color=color, marker="o", edgecolors="k", label="Start")
        if data['aborted'] == True:
            ax.scatter(path_x[-1], path_y[-1], s=40, color="r", marker="X", edgecolors="k", label="Aborted End")
        else:
            ax.scatter(path_x[-1], path_y[-1], s=40, color=color, marker="s", edgecolors="k", label="End")

    # Function to extract path limits
    def get_path_limits(data):
        path = ast.literal_eval(data['path'])
        path = [(p[0] / map_metadata['resolution'], p[1] / map_metadata['resolution']) for p in path]
        x_values, y_values = zip(*path)
        return min(x_values), max(x_values), min(y_values), max(y_values)

    # Function to plot paths for each episode pair and save the figure
    def plot_episode_pair(normal_index, rotated_index):
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(map_image[::-1], cmap="gray", origin="lower", extent=[0, map_image.shape[1], 0, map_image.shape[0]])
        
        # Extract limits and add padding
        x_mins, x_maxs, y_mins, y_maxs = zip(get_path_limits(eval_data_dwa.iloc[normal_index]), 
                                             get_path_limits(eval_data_dwa.iloc[rotated_index]), 
                                             get_path_limits(eval_data_rosnav.iloc[normal_index]), 
                                             get_path_limits(eval_data_rosnav.iloc[rotated_index]), 
                                             get_path_limits(eval_data_teb.iloc[normal_index]), 
                                             get_path_limits(eval_data_teb.iloc[rotated_index]))
        x_min, x_max = min(x_mins), max(x_maxs)
        y_min, y_max = min(y_mins), max(y_maxs)
        x_padding = (x_max - x_min) * 0.2
        y_padding = (y_max - y_min) * 0.2
        x_lims = (x_min - x_padding, x_max + x_padding)
        y_lims = (y_min - y_padding, y_max + y_padding)
        
        # Plot paths for each navigation algorithm
        plot_paths_on_map(eval_data_dwa.iloc[normal_index], map_image, ax, color=path_colors[0])
        plot_paths_on_map(eval_data_dwa.iloc[rotated_index], map_image, ax, color=path_colors[1])
        plot_paths_on_map(eval_data_rosnav.iloc[normal_index], map_image, ax, color=path_colors[2])
        plot_paths_on_map(eval_data_rosnav.iloc[rotated_index], map_image, ax, color=path_colors[3])
        plot_paths_on_map(eval_data_teb.iloc[normal_index], map_image, ax, color=path_colors[4])
        plot_paths_on_map(eval_data_teb.iloc[rotated_index], map_image, ax, color=path_colors[5])

        ax.set_xlim(x_lims)
        ax.set_ylim(y_lims)
        ax.set_title(f"Paths on Map for Episode {normal_index + 1} & {rotated_index + 1}")
        ax.set_xticks([])
        ax.set_yticks([])
        
        # Add legend
        custom_lines = [plt.Line2D([0], [0], color=path_colors[i], lw=2) for i in range(6)]
        ax.legend(custom_lines, [f'DWA Episode {normal_index + 1}', f'DWA Episode {rotated_index + 1}', 
                                 f'ROSNav Episode {normal_index + 1}', f'ROSNav Episode {rotated_index + 1}', 
                                 f'TEB Episode {normal_index + 1}', f'TEB Episode {rotated_index + 1}'])
        plt.tight_layout()
        plt.savefig(f"Episode_{normal_index + 1}_{rotated_index + 1}_Paths.png")

    # Generate visualizations for all episode pairs and save them
    for i in range(7):
        plot_episode_pair(i, i + 8)

# Uncomment the line below to execute the plotting and saving process
# plot_and_save_episode_paths()
