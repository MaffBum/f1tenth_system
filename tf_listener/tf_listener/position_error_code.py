import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.signal import find_peaks
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from matplotlib.lines import Line2D
import csv




def load_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        first_row = next(reader)

    # Check if first row is header (non-numeric values)
    try:
        [float(x) for x in first_row]
        skip = 0  # It's numeric
    except ValueError:
        skip = 1  # Header detected

    data = np.loadtxt(filepath, delimiter=',', skiprows=skip)
    return data[:, 0], data[:, 1], data[:, 2]

def compute_actual_lap_times(timestamps, x, y, start_x=0.6, start_y=0.25, distance_threshold=1.0):
    # dists = np.sqrt((x - start_x)**2 + (y - start_y)**2)
    # Define start/finish line as the first point
    # x0, y0 = start_x, start_y
    x0, y0 = x[0], y[0]
    dists = np.sqrt((x - x0)**2 + (y - y0)**2)

    # Find local minima where car returns near start
    # Invert distance to find valleys (i.e., close to start)
    peaks, _ = find_peaks(-dists, distance=20, prominence=0.1)

    # Filter only those actually within threshold
    crossings = peaks[dists[peaks] < distance_threshold]
    
    # Need at least two to define one full lap
    if len(crossings) < 2:
        print("Not enough laps found!")
        return None, None

    lap_times = np.diff(timestamps[crossings])
    average_lap_time = np.mean(lap_times)

    return average_lap_time, lap_times


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


def compute_velocity_savgol(timestamps, x, y, window=11, poly=3):
    dt = np.mean(np.diff(timestamps))  # Approximate dt

    x_smooth = savgol_filter(x, window, poly)
    y_smooth = savgol_filter(y, window, poly)
    dx = savgol_filter(x, window, poly, deriv=1, delta=dt)
    dy = savgol_filter(y, window, poly, deriv=1, delta=dt)

    speed = np.sqrt(dx**2 + dy**2)
    return timestamps, speed

def compute_lap_time_from_velocity(x, y, v):
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    avg_v = (v[:-1] + v[1:]) / 2  # average velocity per segment
    dt = ds / avg_v
    return np.sum(dt)



def main():
    # Paths for the csvs for raceline and the actual car locations.
    # run from f1tenth_system directory
    velocity_scale = 0.9

    theoretical_path = './pure_pursuit/racelines/may18.csv' # raceline from raceline optimisation code
    actual_path = './particle_filter/csv/E1 copy.csv' # true position from tf_listener

    # Load both racelines
    theo_x, theo_y, theo_v = load_csv(theoretical_path)
    timestamps, act_x_raw, act_y_raw = load_csv(actual_path)

    # Get interpolated velocity at 0.1s intervals
    # t_uniform, velocity = compute_velocity_savgol(timestamps, act_x_raw, act_y_raw)
    t_uniform, x_uniform, y_uniform, velocity = compute_velocity(timestamps, act_x_raw, act_y_raw)

    # Optionally average this x/y for raceline
    act_x, act_y = lap_average(x_uniform, y_uniform)

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
    actual_interp = interp1d(np.linspace(0, 1, len(matched_act_v)), matched_act_v, kind='linear', fill_value="extrapolate")
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

    # Compute actual lap time
    actual_lap_time, all_lap_times = compute_actual_lap_times(timestamps, act_x_raw, act_y_raw)

    # Compute theoretical lap time
    theoretical_lap_time = compute_lap_time_from_velocity(theo_x, theo_y, theo_v*velocity_scale)

    print(f"\nLap Time Comparison:")
    if actual_lap_time is not None:
        print(f"  Actual Avg Lap Time     : {actual_lap_time:.2f} s over {len(all_lap_times)} laps")
    else:
        print("  Actual Lap Time         : Not enough laps detected")

    print(f"  Theoretical Lap Time    : {theoretical_lap_time:.2f} s")
    if actual_lap_time is not None:
        print(f"  Difference               : {actual_lap_time - theoretical_lap_time:.2f} s")

    # Plot
    # Position error plot
    fig, ax = plt.subplots()
    sc = ax.scatter(theo_x, theo_y, c=errors, cmap='viridis', s=10, label='Theoretical Raceline')
    ax.plot(theo_x[0], theo_y[0], 'ro', markersize=10, label='Start Point', zorder=5)
    # ax.annotate('', xy=(theo_x[5], theo_y[5]), xytext=(theo_x[0], theo_y[0]),
    #         arrowprops=dict(arrowstyle='-|>', color='red', lw=2), zorder=5)
    # ax.plot(act_x, act_y, 'r-', label='Actual Raceline (Averaged)')
    ax.plot(act_x, act_y, 'r-', label='Simulation Raceline')

    cbar = plt.colorbar(sc, ax=ax)
    cbar.ax.tick_params(labelsize=25)
    cbar.set_label('Position Error (m)', size=25)
    ax.set_title('Raceline Comparison with Color-Coded Errors', size=25)
    ax.set_xlabel('x', size=25)
    ax.set_ylabel('y', size=25)
    # ax.legend(prop={'size': 15})
    # ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1),
    #       fancybox=True, ncol=2, prop={'size':25})

    # Create custom legend handles
    legend_items_row1 = [
        Line2D([0], [0], marker='o', color='w', label='Theoretical Raceline',
            markerfacecolor='C0', markersize=6),
        Line2D([0], [0], color='r', lw=2, label='Simulation Raceline')
    ]
    legend_items_row2 = [
        Line2D([0], [0], marker='o', color='w', label='Start Point',
            markerfacecolor='red', markersize=10)
    ]

    # First row legend
    legend1 = ax.legend(handles=legend_items_row1, loc='upper center',
                        bbox_to_anchor=(0.5, -0.1), ncol=2,
                        fancybox=True, prop={'size': 25})
    ax.add_artist(legend1)

    # Second row legend (centered)
    ax.legend(handles=legend_items_row2, loc='upper center',
            bbox_to_anchor=(0.5, -0.25), ncol=1,
            fancybox=True, prop={'size': 25})
    
    ax.axis('equal')
    fig.set_facecolor('#f2f1ec')
    plt.grid(True)

    # Velocity comparison plot
    # ax1 = plt.figure()
    # plt.plot(idx, actual_v_resampled, label='Simulation Velocity')
    # # plt.plot(idx, scaled_theo_v, label=f'Scaled Theoretical Velocity (×{velocity_scale})')
    # plt.plot(idx, scaled_theo_v, label=f'Theoretical Velocity')
    # plt.plot(idx, v_error, label='Velocity Error', linestyle='--')
    # plt.xlabel('Normalized Path Index', size=25)
    # plt.ylabel('Velocity (m/s)', size=25)
    # plt.title('Velocity Comparison', size=25)
    # # plt.legend(prop={'size': 15})
    # plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1),
    #       fancybox=True, ncol=2, prop={'size':25})
    # ax1.set_facecolor('#f2f1ec')
    # plt.grid(True)

    # Velocity comparison plot
    fig2, ax2 = plt.subplots()
    line1, = ax2.plot(idx, actual_v_resampled, label='Simulation Velocity', color='tab:red')
    line2, = ax2.plot(idx, scaled_theo_v, label='Theoretical Velocity', color='tab:blue')
    line3, = ax2.plot(idx, v_error, label='Velocity Error', linestyle='--', color='tab:green')

    ax2.set_xlabel('Normalized Path Index', size=25)
    ax2.set_ylabel('Velocity (m/s)', size=25)
    ax2.set_title('Velocity Comparison', size=25)

    # Custom legend handles
    legend_items_row1 = [
        Line2D([0], [0], color='tab:blue', label='Theoretical Velocity'),
        Line2D([0], [0], color='tab:red', label='Simulation Velocity')
    ]
    legend_items_row2 = [
        Line2D([0], [0], color='tab:green', linestyle='--', label='Velocity Error')
    ]

    # First row legend
    legend1 = ax2.legend(handles=legend_items_row1, loc='upper center',
                        bbox_to_anchor=(0.5, -0.1), ncol=2,
                        fancybox=True, prop={'size': 25})
    ax2.add_artist(legend1)

    # Second row legend (centered)
    ax2.legend(handles=legend_items_row2, loc='upper center',
            bbox_to_anchor=(0.5, -0.25), ncol=1,
            fancybox=True, prop={'size': 25})

    fig2.set_facecolor('#f2f1ec')
    ax2.grid(True)

    plt.show()

if __name__ == '__main__':
    main()
