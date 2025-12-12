import pandas as pd
import matplotlib.pyplot as plt
import argparse

def plot_log_file(file_path, last_seconds=None):
    """
    Reads a torque log CSV file and plots the raw vs. smoothed torque for each joint found.
    Optionally plots only the last N seconds of data.
    """
    try:
        df = pd.read_csv(file_path)
        print("Log file loaded successfully. Columns found:")
        print(df.columns.tolist())

    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
        return
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return

    if 'timestamp' not in df.columns:
        print("Error: 'timestamp' column not found in the log file.")
        return
        
    df['time_sec'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1e9

    plot_df = df
    if last_seconds is not None and not df.empty:
        print(f"Displaying only the last {last_seconds} seconds of the log.")
        end_time = df['time_sec'].iloc[-1]
        start_time = max(0, end_time - last_seconds)
        plot_df = df[df['time_sec'] >= start_time].copy()

    joint_indices = []
    for col in plot_df.columns:
        if 'raw_torque_j' in col:
            try:
                idx = int(col.split('raw_torque_j')[1])
                if idx not in joint_indices:
                    joint_indices.append(idx)
            except (IndexError, ValueError):
                continue
    joint_indices.sort()
    
    if not joint_indices:
        print("No columns matching 'raw_torque_jX' format found in the CSV file.")
        return
        
    num_joints = len(joint_indices)
    print(f"Found data for {num_joints} joints: {joint_indices}")

    fig, axes = plt.subplots(num_joints, 1, figsize=(15, 4 * num_joints), sharex=True)
    if num_joints == 1:
        axes = [axes]

    fig.suptitle('Torque Smoothing Analysis (from Log File)', fontsize=16, fontweight='bold')

    for i, joint_idx in enumerate(joint_indices):
        ax = axes[i]
        raw_col = f'raw_torque_j{joint_idx}'
        smooth_col = f'smoothed_torque_j{joint_idx}'

        if raw_col not in plot_df.columns or smooth_col not in plot_df.columns:
            print(f"Warning: Missing raw or smoothed column for joint {joint_idx}. Skipping.")
            continue

        ax.plot(plot_df['time_sec'], plot_df[raw_col], label='Raw Torque', color='cornflowerblue', alpha=0.8, linewidth=1.5)
        ax.plot(plot_df['time_sec'], plot_df[smooth_col], label='Smoothed Torque', color='darkorange', linewidth=2.5)
        
        ax.set_title(f'Joint {joint_idx} Torque')
        ax.set_ylabel('Torque (Nm)')
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.legend()
        ax.axhline(0, color='black', linewidth=0.5, linestyle='-')

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot torque data from a CSV log file.')
    parser.add_argument(
        'file', 
        type=str, 
        help='Path to the torque_log.csv file.'
    )
    parser.add_argument(
        '--last-seconds',
        type=float,
        default=None,
        help='Optional: Only plot the last N seconds of the log file.'
    )
    args = parser.parse_args()

    plot_log_file(args.file, args.last_seconds)