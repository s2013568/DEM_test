import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants (adjust as needed)
filename = 'Ageng_based_simulation\\Chraibi_God_like_Code\\traj_133_av0.00_v01.00.txt'  # Replace with your actual file path
fps = 8  # frames per second
Length = 200  # length of the corridor

# Function to read the data from the file
def read_data(filename):
    """
    Reads the state data from the txt file and returns it in a structured way.
    """
    data = np.loadtxt(filename)
    ids = np.unique(data[:, 0])  # Get unique pedestrian IDs
    times = np.unique(data[:, 1])  # Get unique time steps
    positions = {}

    for t in times:
        positions[t] = data[data[:, 1] == t][:, 2]  # Extract positions for time t
    return times, ids, positions

# Set up figure and axis for animation
fig, ax = plt.subplots()
scat = ax.scatter([], [], c='blue', s=50)

# Set up the plot limits based on the corridor length
ax.set_xlim(0, Length)
ax.set_ylim(-1, 1)  # Assume 1D movement on x-axis, y is fixed

# Initialize scatter plot
def init():
    # Initialize with an empty 2D array
    scat.set_offsets(np.empty((0, 2)))
    return scat,

# Update scatter plot for each frame
def update(frame, times, positions):
    t = times[frame]
    x_n = positions[t]  # Get current positions for time t
    y = np.zeros_like(x_n)  # Since it's 1D, all pedestrians are on the same y level
    data = np.vstack([x_n, y]).T  # Combine x and y positions into 2D array
    scat.set_offsets(data)
    return scat,

def animate_from_file(filename, max_frames=100):
    # Read the data from the file
    times, ids, positions = read_data(filename)
    
    # Limit the number of frames
    times = times[:max_frames]  # Restrict to the first 'max_frames' steps

    # Animation setup
    anim = FuncAnimation(
        fig, update, frames=len(times), init_func=init, fargs=(times, positions),
        blit=True, interval=1000/fps
    )

    # Show the animation
    plt.show()

if __name__ == "__main__":
    # Call the animation function with the filename of the txt file
    animate_from_file(filename)
