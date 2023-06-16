import matplotlib
matplotlib.use('Agg')  # use a non-GUI backend
import matplotlib.pyplot as plt


def visualize_pointcloud(xyz_camera):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xyz_camera[:, 0], xyz_camera[:, 1], xyz_camera[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=30, azim=60)  # Set the elevation and azimuth angles

    plt.savefig('pointcloud.png')  # save figure to file