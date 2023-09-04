import pandas as pd
import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Load the CSV data
path_data = pd.read_csv('/mnt/data/path_data_episode_1.csv')
# path_data.head()

# Load the YAML data
with open('/mnt/data/5LpN3gDmAk7.yaml', 'r') as file:
    map_meta_data = yaml.load(file, Loader=yaml.FullLoader)

# map_meta_data

# Load the map image
map_image = mpimg.imread('/mnt/data/5LpN3gDmAk7.pgm')

# Extract x and y coordinates from the CSV, including the first row
x_coordinates = [path_data.columns[0]] + path_data[path_data.columns[0]].tolist()
y_coordinates = [path_data.columns[1]] + path_data[path_data.columns[1]].tolist()
x_coordinates = [float(x) for x in x_coordinates]
y_coordinates = [float(y) for y in y_coordinates]

# Convert real-world coordinates to pixel coordinates
pixel_x = [int(x/map_meta_data['resolution']) for x in x_coordinates]
pixel_y = [map_image.shape[0] - int(y/map_meta_data['resolution']) for y in y_coordinates]  # Invert y-axis since image origin is top-left

# Plot the map and the path
plt.figure(figsize=(10, 10))
plt.imshow(map_image, cmap='gray')
plt.plot(pixel_x, pixel_y, 'r', label="Path")
plt.title("Path on Map")
plt.xlabel("X (pixels)")
plt.ylabel("Y (pixels)")
plt.legend()
plt.show()
