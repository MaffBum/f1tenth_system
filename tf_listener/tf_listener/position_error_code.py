import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.signal import find_peaks

def load_csv(filepath):
    data = np.loadtxt(filepath, delimiter=',')
    return data[:, 0], data[:, 1], data[:, 2]

def lap_average(x, y, num_bins=500):
    # Compute angles from the track center (assumes loop around origin)
    angles = np.arctan2(y - np.mean(y), x - np.mean(x))
    angles = (angles + 2 * np.pi) % (2 * np.pi)  # wrap to [0, 2π)

    bins = np.linspace(0, 2*np.pi, num_bins+1)
    indices = np.digitize(angles, bins) - 1

    x_sum = np.zeros(num_bins)
    y_sum = np.zeros(num_bins)
    counts = np.zeros(num_bins)

    for i in range(len(x)):
        idx = indices[i]
        if 0 <= idx < num_bins:
            x_sum[idx] += x[i]
            y_sum[idx] += y[i]
            counts[idx] += 1

    valid = counts > 0
    x_avg = np.full_like(x_sum, np.nan, dtype=float)
    y_avg = np.full_like(y_sum, np.nan, dtype=float)
    x_avg[valid] = x_sum[valid] / counts[valid]
    y_avg[valid] = y_sum[valid] / counts[valid]

    return x_avg[valid], y_avg[valid]

def compute_errors(theoretical_x, theoretical_y, actual_x, actual_y):
    tree = cKDTree(np.c_[actual_x, actual_y])
    distances, _ = tree.query(np.c_[theoretical_x, theoretical_y])
    return distances

def compute_velocity(timestamps, x, y, dt=0.1):
    t_uniform = np.arange(timestamps[0], timestamps[-1], dt)
    x_interp = interp1d(timestamps, x, kind='linear')(t_uniform)
    y_interp = interp1d(timestamps, y, kind='linear')(t_uniform)

    dx = np.diff(x_interp)
    dy = np.diff(y_interp)
    speed = np.sqrt(dx**2 + dy**2) / dt

    # Pad to match the number of points
    speed = np.append(speed, speed[-1])
    return t_uniform, x_interp, y_interp, speed


def main():
    # Paths for the csvs for raceline and the actual car locations.
    # run from f1tenth_system directory
    theoretical_path = './pure_pursuit/racelines/7AprilGP.csv' # raceline from raceline optimisation code
    actual_path = './tf_listener/data/tf_data.csv' # true position from tf_listener

    # Load both racelines
    theo_x, theo_y, theo_v = load_csv(theoretical_path)
    act_t_raw, act_x_raw, act_y_raw = load_csv(actual_path)

    # Average actual raceline (remove multi-lap data)
    act_x, act_y = lap_average(act_x_raw, act_y_raw)
    

    # Compute position errors
    errors = compute_errors(theo_x, theo_y, act_x, act_y)
    max_e = np.max(errors)
    min_e = np.min(errors)
    mean_e = np.mean(errors)
    median_e = np.median(errors)
    std_e = np.std(errors)

    print(f"Position Errors:")
    print(f"  Max    : {max_e:.4f} m")
    print(f"  Min    : {min_e:.4f} m")
    print(f"  Mean   : {mean_e:.4f} m")
    print(f"  Median : {median_e:.4f} m")
    print(f"  Std Dev: {std_e:.4f} m")

    # Plot
    fig, ax = plt.subplots()
    sc = ax.scatter(theo_x, theo_y, c=errors, cmap='viridis', s=10, label='Theoretical Raceline')
    ax.plot(act_x, act_y, 'r-', label='Actual Raceline (Averaged)')
    plt.colorbar(sc, ax=ax, label='Position Error (m)')
    ax.set_title('Raceline Comparison with Color-Coded Errors')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.legend()
    ax.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
