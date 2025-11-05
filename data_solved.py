import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

file_path = 'data.csv'

try:
    raw_df = pd.read_csv(file_path)
    df = pd.read_csv(file_path)

    altitude_raw = raw_df['altitude_m'].copy()
    battery_raw = raw_df['battery_voltage'].copy()

    df['battery_voltage'] = df['battery_voltage'].replace(0.0, np.nan)
    df['altitude_m'] = df['altitude_m'].replace(5000.0, np.nan)
    df['battery_voltage'] = df['battery_voltage'].replace(25.5, np.nan)
    df['altitude_m'] = df['altitude_m'].replace(-5.5, np.nan)

    df['flight_mode'] = df['flight_mode'].fillna('UNKNOWN')

    df['altitude_m'] = df['altitude_m'].interpolate(method='linear')
    df['latitude'] = df['latitude'].interpolate(method='linear')
    df['longitude'] = df['longitude'].interpolate(method='linear')
    df['battery_voltage'] = df['battery_voltage'].interpolate(method='linear')

    print("--- Statistical Analysis of Data Given ---")
    cleaned_stats = df[['altitude_m', 'battery_voltage', 'latitude', 'longitude']].describe()
    print(cleaned_stats)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 18))
    fig.suptitle('Drone Flight Log Analysis', fontsize=16)

    ax1.plot(raw_df['timestamp'], altitude_raw, 'r--', label='Raw Altitude', alpha=0.5)
    ax1.plot(df['timestamp'], df['altitude_m'], 'b-', marker='o', markersize=4, label='Cleaned Altitude')
    ax1.set_title('Altitude over Time')
    ax1.set_xlabel('Timestamp')
    ax1.set_ylabel('Altitude (m)')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(raw_df['timestamp'], battery_raw, 'r--', label='Raw Battery', alpha=0.5)
    ax2.plot(df['timestamp'], df['battery_voltage'], 'g-', marker='o', markersize=4, label='Cleaned Battery')
    ax2.set_title('Battery Voltage over Time')
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('Battery Voltage (V)')
    ax2.legend()
    ax2.grid(True)

    ax3.set_title('2D Flight Path by Flight Mode')
    ax3.set_xlabel('Longitude')
    ax3.set_ylabel('Latitude')
    
    modes = df['flight_mode'].unique()
    colors = plt.cm.jet(np.linspace(0, 1, len(modes)))

    for mode, color in zip(modes, colors):
        mode_data = df[df['flight_mode'] == mode]
        ax3.plot(mode_data['longitude'], mode_data['latitude'], marker='o', linestyle='-', label=mode, color=color, markersize=5)
    
    ax3.scatter(df.iloc[0]['longitude'], df.iloc[0]['latitude'], c='blue', s=100, label='Start', zorder=5, marker='X')
    ax3.scatter(df.iloc[-1]['longitude'], df.iloc[-1]['latitude'], c='red', s=100, label='End', zorder=5, marker='X')
    
    ax3.legend()
    ax3.grid(True)
    ax3.set_aspect('equal', adjustable='box')

    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.savefig('data_plot.png')
    
    print("Successfully generated plot image 'cleaned_flight_data_plots.png'.")

except Exception as e:
    print(f'An error occurred: {e}')