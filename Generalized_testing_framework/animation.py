import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants (adjust as needed)
filename = 'traj_tau0.50_v01.50_Length100.00_Width10.00_periodicTrue_A2000.00_B0.08_delta_t0.20_N100.txt'  # Replace with your actual file path

fps = 8  # frames per second
Length = 100  # length of the corridor
Width = 400  # width of the corridor
agent_radius = 0.5  # radius of the surrounding circle

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
        positions[t] = data[data[:, 1] == t][:, 2:4]  # Extract x and y positions for time t
    return times, ids, positions

# Set up figure and axis for animation
fig, ax = plt.subplots()
scat = ax.scatter([], [], c='blue', s=50)
circles = []  # List to store circle patches

# Set up the plot limits based on the corridor length and width
ax.set_xlim(0, 100)
ax.set_ylim(0, 10)  # Set y limits according to the width of the corridor

# Initialize scatter plot
def init():
    # Initialize with an empty 2D array
    scat.set_offsets(np.empty((0, 2)))
    return scat,

# Update scatter plot and circles for each frame
def update(frame, times, positions):
    global circles
    t = times[frame]
    pos = positions[t]  # Get current positions for time t

    # Apply periodic boundary conditions
    pos[:, 0] = pos[:, 0] % Length  # Wrap x positions around Length
    pos[:, 1] = pos[:, 1]  # Wrap y positions around Width

    # Update scatter plot with new positions
    scat.set_offsets(pos)
    
    # Remove old circles
    for circle in circles:
        circle.remove()
    circles = []

    # Add new circles around each agent
    for x, y in pos:
        circle = plt.Circle((x, y), agent_radius, color='blue', fill=False, linestyle='--', alpha=0.5)
        ax.add_patch(circle)
        circles.append(circle)
    
    return scat,

def animate_from_file(filename, max_frames=1000):
    # Read the data from the file
    times, ids, positions = read_data(filename)
    
    # Limit the number of frames
    times = times[:max_frames]  # Restrict to the first 'max_frames' steps

    # Animation setup
    anim = FuncAnimation(
        fig, update, frames=len(times), init_func=init, fargs=(times, positions),
        blit=False, interval=1000/fps
    )

    # Show the animation
    plt.show()

if __name__ == "__main__":
    # Call the animation function with the filename of the txt file
    animate_from_file(filename)
