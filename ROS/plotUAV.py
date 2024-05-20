import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def read_data_from_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)
    return data

def parse_coordinates(data):
    target_coord = []
    prd_target_coord = []
    tracker_coord = []

    for row in data:
        target_coord.append((float(row[0]), float(row[1]), float(row[2])))
        prd_target_coord.append((float(row[3]), float(row[4]), float(row[5])))
        tracker_coord.append((float(row[6]), float(row[7]), float(row[8])))
    
    return target_coord, prd_target_coord, tracker_coord

def update_plot(num, data, lines, ax):
    # Adjust the number of points to plot for each line
    num_points = [min(num, len(coord)) for coord in data]

    for line, coord, np in zip(lines, data, num_points):
        if np > 0 and coord:  # Ensure there are points to plot
            x, y, z = zip(*coord[:np])
            line.set_data(x, y)
            line.set_3d_properties(z)
        else:
            line.set_data([], [])
            line.set_3d_properties([])

    # Update axes limits dynamically
    all_coords = [c for sublist in data for c in sublist[:max(num_points)]]
    if all_coords:
        all_x, all_y, all_z = zip(*all_coords)
        ax.set_xlim3d([min(all_x), max(all_x)])
        ax.set_ylim3d([min(all_y), max(all_y)])
        ax.set_zlim3d([min(all_z), max(all_z)])

    return lines

def animate_trajectories(target_coord, prd_target_coord, tracker_coord):
    interval = 500  # 设置为 500 毫秒，即每 0.5 秒更新一次

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Prepare lines for the plot
    line1, = ax.plot([], [], [], 'r-', label='Target Coordinate')
    line2, = ax.plot([], [], [], 'b-', label='Predicted Target Coordinate')
    line3, = ax.plot([], [], [], 'g-', label='Tracker Coordinate')
    lines = [line1, line2, line3]
    lines = [line1, line2]

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('3D Trajectories')
    ax.legend()

    # Adjust the data lists to align them
    adjusted_target_coord = target_coord[4:]  # Skip first four points
    adjusted_tracker_coord = tracker_coord[4:]  # Skip first four points

    data = [adjusted_target_coord, prd_target_coord, adjusted_tracker_coord]
    ani = FuncAnimation(fig, update_plot, frames=max(len(adjusted_target_coord), len(prd_target_coord), len(adjusted_tracker_coord)), fargs=(data, lines, ax), interval=interval, blit=False, repeat=False)

    plt.show()


filename = 'output.csv'
csv_data = read_data_from_csv(filename)
target_coordinate, prd_target_coordinate, tracker_coordinate = parse_coordinates(csv_data)

animate_trajectories(target_coordinate, prd_target_coordinate, tracker_coordinate)
