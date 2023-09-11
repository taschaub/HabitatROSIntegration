import pandas as pd
import numpy as np
import ast

def compute_metrics(data):
    # Convert the 'path' strings into lists of coordinates
    data['path'] = data['path'].apply(ast.literal_eval)
    
    # Path Length
    data['path_length'] = data['path'].apply(lambda x: sum(np.linalg.norm(np.array(x[i]) - np.array(x[i+1])) for i in range(len(x)-1)))
    
    # Time to Reach Goal
    data['time_to_reach'] = data['end_time'] - data['start_time']
    
    # Collision Rate
    collision_rate = len(data[data['collisions'] > 0]) / len(data)
    
    # Success Rate
    if 'aborted' in data.columns:
        success_rate = len(data[(data['collisions'] == 0) & (data['aborted'].isna())]) / len(data)
    else:
        success_rate = len(data[data['collisions'] == 0]) / len(data)
    
    # Abortion Rate
    if 'aborted' in data.columns:
        abortion_rate = len(data[~data['aborted'].isna()]) / len(data)
    else:
        abortion_rate = 0
    
    # Average Speed
    data['average_speed'] = data['path_length'] / data['time_to_reach']
    
    metrics = {
        'path_length': data['path_length'].mean(),
        'time_to_reach': data['time_to_reach'].mean(),
        'collision_rate': collision_rate * 100,
        'success_rate': success_rate * 100,
        'abortion_rate': abortion_rate * 100,
        'average_speed': data['average_speed'].mean()
    }
    
    return metrics

def main():
    # Load the CSV file
    csv_file = input("Enter the path to the CSV file: ")
    data = pd.read_csv(csv_file)
    
    # Get the episodes to consider
    episodes = input("Enter the episodes you want to consider (comma-separated, e.g., 1,2,3): ")
    episodes = list(map(int, episodes.split(',')))
    
    # Filter the data based on the episodes using their actual episode numbers (which are same as the row indices + 1)
    data = data.iloc[[episode-1 for episode in episodes]]
    
    # Compute the metrics
    metrics = compute_metrics(data)
    
    # Display the results
    for key, value in metrics.items():
        print(f"{key}: {value:.2f}")
    
if __name__ == "__main__":
    main()

