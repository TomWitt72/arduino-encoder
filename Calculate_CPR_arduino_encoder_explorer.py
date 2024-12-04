import threading
import queue
import time
import math
import serial #pip install pyserial
import matplotlib.pyplot as plt #pip install matplotlib

def read_serial_data(serial_port, baud_rate, data_queue, stop_event):
    """
    Reads data from the serial port and puts it into a queue.
    """
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            ser.flushInput()
            while not stop_event.is_set():
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        position_change = int(line)
                        data_queue.put((time.time(), position_change))
                        print(f"Received data: {position_change}")
                    except ValueError:
                        print(f"Non-integer data received: {line}")
                time.sleep(0.01)
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")

def main():

    # Serial port parameters
    serial_port = input("Enter the serial port (e.g., COM4 or /dev/ttyACM0): ")
    baud_rate = 57600  # Should match the Arduino's baud rate

    # Get user input for wheel diameter
    wheel_diameter = float(input("Enter wheel diameter (in meters): "))

    #Converting wheel diameter into circumference
    circumference = float(input(math.pi*(wheel_diameter/2)*2))

#Taking the cirumference to estimate the total distance traveled, should be able to backwork the crp rate
    total_distance_estimated=circumference 
    
    
    # Start the serial reading thread
    data_queue = queue.Queue()
    stop_event = threading.Event()
    serial_thread = threading.Thread(
        target=read_serial_data,
        args=(serial_port, baud_rate, data_queue, stop_event)
    )
    serial_thread.start()

    print("\nCollecting data... Press Ctrl+C to stop.\n")

    # Data collection variables
    times = []
    position_changes = []

    try:
        while True:
            try:
                timestamp, position_change = data_queue.get(timeout=1)
                times.append(timestamp)
                position_changes.append(position_change)
            except queue.Empty:
                pass  # No new data received in the last second
    except KeyboardInterrupt:
        print("\nData collection stopped by user.")
    finally:
        # Stop the serial thread
        stop_event.set()
        serial_thread.join()
        print("Serial thread stopped.")

    # Check if data was collected
    if not times:
        print("No data was collected. Exiting.")
        return

    # Convert timestamps to elapsed time
    start_time = times[0]
    times = [t - start_time for t in times]

    # Calculate sample intervals
    sample_intervals = [t2 - t1 for t1, t2 in zip(times[:-1], times[1:])]

    # Eestimate the CPR
    estimated_cpr = float(input("\nEstimate the Counts Per Revolution (CPR) of the encoder: "))

    # Calculate distance per count based on estimated CPR
    distance_per_count = (math.pi * wheel_diameter) / estimated_cpr

    # Calculate speeds and distances
    speeds = []
    distances = []
    cumulative_distance = 0.0  # Initialize cumulative distance

    for i in range(1, len(times)):
        delta_counts = position_changes[i]
        delta_time = times[i] - times[i - 1]

        # Skip if delta_time is too small
        if delta_time <= 0:
            continue

        # Calculate speed
        speed = (delta_counts * distance_per_count) / delta_time
        speeds.append((times[i], speed))

        # Calculate distance
        distance = delta_counts * distance_per_count
        cumulative_distance += distance
        distances.append((times[i], cumulative_distance))

       #estimated distance per count to calculate CPR
        estimated_distance_per_count = (total_distance_estimated / delta_counts) 

    # Display results
    print("\nCalculated Speeds and Cumulative Distance based on your estimated CPR:")
    print("Time (s)\tSpeed (m/s)\tCumulative Distance (m)")
    for (t_speed, speed), (t_dist, cum_dist) in zip(speeds, distances):
        print(f"{t_speed:.2f}\t\t{speed:.2f}\t\t{cum_dist:.2f}")

    # Calculate the total distance traveled
    total_distance = cumulative_distance
    print(f"\nTotal Distance Traveled: {total_distance:.2f} meters")

    # Calculate the average speed
    if speeds:
        average_speed = sum(s for _, s in speeds) / len(speeds)
        print(f"Average Calculated Speed: {average_speed:.2f} m/s")
    else:
        print("No speed data to calculate average.")

# Calculate the CPR
    calculated_cpr = ((math.pi*wheel_diameter)/estimated_distance_per_count)
    
    # Plot the speed over time
    if speeds:
        plot_times, plot_speeds = zip(*speeds)
        plt.figure(figsize=(10, 5))
        plt.plot(plot_times, plot_speeds, label='Calculated Speed')
        plt.title('Calculated Speed Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No speed data to plot.")

    # Plot the cumulative distance over time
    if distances:
        dist_times, cum_distances = zip(*distances)
        plt.figure(figsize=(10, 5))
        plt.plot(dist_times, cum_distances, label='Cumulative Distance', color='green')
        plt.title('Cumulative Distance Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No distance data to plot.")


    # Allow adjustment of CPR estimate
    while True:
        adjust_cpr = input("\nWould you like to adjust your CPR estimate? (yes/no): ").lower()
        if adjust_cpr == 'yes':
            estimated_cpr = float(input("Enter new CPR estimate: "))
            distance_per_count = (math.pi * wheel_diameter) / estimated_cpr

            # Recalculate speeds and distances
            speeds = []
            distances = []
            cumulative_distance = 0.0

            for i in range(1, len(times)):
                delta_counts = position_changes[i]
                delta_time = times[i] - times[i - 1]

                if delta_time <= 0:
                    continue

                speed = (delta_counts * distance_per_count) / delta_time
                speeds.append((times[i], speed))

                distance = delta_counts * distance_per_count
                cumulative_distance += distance
                distances.append((times[i], cumulative_distance))

            # Display results
            print("\nNew Calculated Speeds and Cumulative Distance:")
            print("Time (s)\tSpeed (m/s)\tCumulative Distance (m)")
            for (t_speed, speed), (t_dist, cum_dist) in zip(speeds, distances):
                print(f"{t_speed:.2f}\t\t{speed:.2f}\t\t{cum_dist:.2f}")
                
            # Display total distance
            total_distance = cumulative_distance
            print(f"\nTotal Distance Traveled: {total_distance:.2f} meters")

            # Calculate the average speed
            if speeds:
                average_speed = sum(s for _, s in speeds) / len(speeds)
                print(f"Average Calculated Speed: {average_speed:.2f} m/s")
            else:
                print("No speed data to calculate average.")

            # Display calculated CPR
            print(f"\nCalculated CPR: {calculated_cpr}")
            
            # Plot the speed over time
            if speeds:
                plot_times, plot_speeds = zip(*speeds)
                plt.figure(figsize=(10, 5))
                plt.plot(plot_times, plot_speeds, label='Calculated Speed')
                plt.title('Calculated Speed Over Time')
                plt.xlabel('Time (s)')
                plt.ylabel('Speed (m/s)')
                plt.legend()
                plt.grid(True)
                plt.show()
            else:
                print("No speed data to plot.")

            # Plot the cumulative distance over time
            if distances:
                dist_times, cum_distances = zip(*distances)
                plt.figure(figsize=(10, 5))
                plt.plot(dist_times, cum_distances, label='Cumulative Distance', color='green')
                plt.title('Cumulative Distance Over Time')
                plt.xlabel('Time (s)')
                plt.ylabel('Distance (m)')
                plt.legend()
                plt.grid(True)
                plt.show()
            else:
                print("No distance data to plot.")

        elif adjust_cpr == 'no':
            break
        else:
            print("Please enter 'yes' or 'no'.")


if __name__ == "__main__":
    main()
