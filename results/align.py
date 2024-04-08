import numpy as np

def parse_trajectory_file(file_path):
    positions = []

    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split(' ')
            position = np.array([float(x) for x in data[1:4]])
            positions.append(position)

    return positions
def save_trajectory_to_file_tum_format(trajectory, filename):
    with open(filename, 'w') as file:
        timestamp = 0  # 初始时间戳
        for point in trajectory:
            tx, ty, tz = point[0], point[1], point[2]
            qx, qy, qz, qw = 0, 0, 0, 1  # 旋转量设为0，单位四元数
            file.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
            timestamp += 1  # 增加时间戳，这里简单地递增

def align_trajectories(trajectory1, trajectory2):
    # Calculate the translation vector to align the first trajectory's start point with the second trajectory's end point
    print("trajectory1[0]:", trajectory1[0])
    print("trajectory2[-1]:", trajectory2[-1])
    translation_vector = trajectory2[-1] - trajectory1[0]

    # Apply the translation vector to the first trajectory
    aligned_trajectory1 = [point + translation_vector for point in trajectory1]

    # 使用上面的函数保存 aligned_trajectory1 到新的 TUM 轨迹文件
    save_trajectory_to_file_tum_format(aligned_trajectory1, "aligned_trajectory1.txt")

if __name__ == "__main__":
    # Assuming trajectory1_file, trajectory2_file, and trajectory3_file are the paths to the three TUM format trajectory files
    trajectory1_file = "gpstrajectory.txt"
    trajectory2_file = "f_zed_1f_loop_gnss.txt"
    trajectory3_file = "f_g07_kitti_2011_09_30_drive_0027_synced.txt"

    # Parse the trajectory files
    trajectory1 = parse_trajectory_file(trajectory1_file)
    trajectory2 = parse_trajectory_file(trajectory2_file)
    trajectory3 = parse_trajectory_file(trajectory3_file)

    # Align trajectory1 with trajectory2
    aligned_trajectory1 = align_trajectories(trajectory1, trajectory2)

    # Now aligned_trajectory1 starts from the end point of trajectory2
    # You can save aligned_trajectory1 to a new file or use it for further processing
