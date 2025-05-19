import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# Load trajectory CSV
traj_path = "./particle_filter/csv/A1.csv"
if not os.path.exists(traj_path):
    raise FileNotFoundError("Could not find trajectory CSV at: {}".format(traj_path))

traj_df = pd.read_csv(traj_path)
traj_df['time'] -= traj_df['time'].iloc[0]  # Set t = 0

# Load raceline CSV (no header, columns: x, y, velocity)
raceline_path = "./pure_pursuit/racelines/may18.csv"
if not os.path.exists(raceline_path):
    raise FileNotFoundError("Could not find raceline CSV at: {}".format(raceline_path))

raceline_df = pd.read_csv(raceline_path, header=None, names=['x', 'y', 'v'])

# Setup plot
fig, ax = plt.subplots()
line, = ax.plot([], [], 'bo-', lw=2, label='Estimated Trajectory')
ax.plot(raceline_df['x'], raceline_df['y'], 'r.', markersize=3, label='Raceline')

# Set plot limits
all_x = pd.concat([traj_df['x'], raceline_df['x']])
all_y = pd.concat([traj_df['y'], raceline_df['y']])
ax.set_xlim(all_x.min() - 0.1, all_x.max() + 0.1)
ax.set_ylim(all_y.min() - 0.1, all_y.max() + 0.1)

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Trajectory Animation with Raceline Overlay")
ax.legend()

# Animation init
def init():
    line.set_data([], [])
    return line,

# Animation step
def animate(i):
    line.set_data(traj_df['x'][:i+1], traj_df['y'][:i+1])
    return line,

# Create animation
ani = animation.FuncAnimation(
    fig, animate, frames=len(traj_df), init_func=init,
    blit=True, interval=100
)

# Save animation as MP4
from matplotlib.animation import FFMpegWriter
writer = FFMpegWriter(fps=10)
ani.save("trajectory_animation.mp4", writer=writer)
print("✅ Animation saved as 'trajectory_animation.mp4'")
