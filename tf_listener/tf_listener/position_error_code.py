import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.signal import find_peaks
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter



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
    distances, indices = tree.query(np.c_[theoretical_x, theoretical_y])
    return distances, indices

# def compute_velocity(timestamps, x, y, dt=0.1):
#     t_uniform = np.arange(timestamps[0], timestamps[-1], dt)
#     x_interp = interp1d(timestamps, x, kind='linear')(t_uniform)
#     y_interp = interp1d(timestamps, y, kind='linear')(t_uniform)

#     dx = np.diff(x_interp)
#     dy = np.diff(y_interp)
#     speed = np.sqrt(dx**2 + dy**2) / dt

#     # Pad to match the number of points
#     speed = np.append(speed, speed[-1])
#     return t_uniform, x_interp, y_interp, speed


def compute_velocity_savgol(timestamps, x, y, window=11, poly=3):
    dt = np.mean(np.diff(timestamps))  # Approximate dt

    x_smooth = savgol_filter(x, window, poly)
    y_smooth = savgol_filter(y, window, poly)
    dx = savgol_filter(x, window, poly, deriv=1, delta=dt)
    dy = savgol_filter(y, window, poly, deriv=1, delta=dt)

    speed = np.sqrt(dx**2 + dy**2)
    return timestamps, speed



def main():
    # Paths for the csvs for raceline and the actual car locations.
    # run from f1tenth_system directory
    velocity_scale = 0.9

    theoretical_path = './pure_pursuit/racelines/U_GP.csv' # raceline from raceline optimisation code
    actual_path = './tf_listener/data/tf_data.csv' # true position from tf_listener

    # Load both racelines
    theo_x, theo_y, theo_v = load_csv(theoretical_path)
    timestamps, act_x_raw, act_y_raw = load_csv(actual_path)

    # Get interpolated velocity at 0.1s intervals
    t_uniform, velocity = compute_velocity_savgol(timestamps, act_x_raw, act_y_raw)

    # Optionally average this x/y for raceline
    act_x, act_y = lap_average(act_x_raw, act_y_raw)

    # act_x, act_y = lap_average(act_x_raw, act_y_raw)


    # Compute position errors
    errors, match_indices = compute_errors(theo_x, theo_y, act_x, act_y)
    matched_act_v = velocity[match_indices]
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

    theo_interp = interp1d(np.linspace(0, 1, len(theo_v)), theo_v, kind='linear', fill_value="extrapolate")
    actual_interp = interp1d(np.linspace(0, 1, len(velocity)), matched_act_v, kind='linear', fill_value="extrapolate")
    N = min(len(theo_v), len(velocity))
    idx = np.linspace(0, 1, N)
    theo_v_resampled = theo_interp(idx)
    actual_v_resampled = actual_interp(idx)

    scaled_theo_v = theo_v_resampled * velocity_scale
    v_error = actual_v_resampled - scaled_theo_v

    print(f"\nVelocity Errors (after applying velocity_scale={velocity_scale}):")
    print(f"  Max    : {np.max(v_error):.4f} m/s")
    print(f"  Min    : {np.min(v_error):.4f} m/s")
    print(f"  Mean   : {np.mean(v_error):.4f} m/s")
    print(f"  Median : {np.median(v_error):.4f} m/s")
    print(f"  Std Dev: {np.std(v_error):.4f} m/s")

    # Plot
    # Position error plot
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

    # Velocity comparison plot
    plt.figure()
    plt.plot(idx, actual_v_resampled, label='Actual Velocity')
    plt.plot(idx, scaled_theo_v, label=f'Scaled Theoretical Velocity (×{velocity_scale})')
    plt.plot(idx, v_error, label='Velocity Error', linestyle='--')
    plt.xlabel('Normalized Path Index')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Comparison')
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == '__main__':
    main()
