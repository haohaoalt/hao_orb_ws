import sys
import numpy as np
import matplotlib.pyplot as plt

def parse_trajectory_file(file_path):
    timestamps = []
    positions = []
    orientations = []

    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split(' ')
            timestamp = float(data[0])
            position = np.array([float(x) for x in data[1:4]])
            orientation = np.array([float(x) for x in data[4:8]])

            timestamps.append(timestamp)
            positions.append(position)
            orientations.append(orientation)

    return timestamps, positions, orientations

def plot_trajectory(ax, positions, view):
    x = [pos[0] for pos in positions]
    y = [pos[1] for pos in positions]
    z = [pos[2] for pos in positions]

    if view == 'xz':
        ax.plot(x, z)
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
    elif view == 'xy':
        ax.plot(x, y)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
    elif view == 'xyz':
        ax.plot(x, y, z)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script_name.py trajectory_file1.txt trajectory_file2.txt ...")
        sys.exit(1)

    fig = plt.figure()

    # Create subplot for xyz view
    ax_xyz = fig.add_subplot(131, projection='3d')
    ax_xyz.set_title('XYZ View')

    # Create subplot for xz view
    ax_xz = fig.add_subplot(132)
    ax_xz.set_title('XZ View')
    ax_xz.set_aspect('equal')

    # Create subplot for xy view
    ax_xy = fig.add_subplot(133)
    ax_xy.set_title('XY View')
    ax_xy.set_aspect('equal')

    for i in range(1, len(sys.argv)):
        file_path = sys.argv[i]
        timestamps, positions, orientations = parse_trajectory_file(file_path)
        plot_trajectory(ax_xyz, positions, 'xyz')
        plot_trajectory(ax_xz, positions, 'xz')
        plot_trajectory(ax_xy, positions, 'xy')

    plt.tight_layout()
    plt.show()
