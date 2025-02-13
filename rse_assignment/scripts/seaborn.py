#!/usr/bin/env python3

import requests
import time
import matplotlib.pyplot as plt
import numpy as np

# Function to fetch data from Script B's API
def fetch_data_from_script_b():
    try:
        response = requests.get("http://localhost:8080/get_wheel_data")  # Update the URL if needed
        if response.status_code == 200:
            return response.json()  # Return the fetched JSON data
        else:
            print("Error fetching data:", response.status_code)
            return None
    except Exception as e:
        print(f"Exception occurred: {e}")
        return None

left_rpm_values = []
right_rpm_values = []
time_values = []

def process_and_plot_data(left_rpm, right_rpm):
    left_rpm_values.append(left_rpm)
    right_rpm_values.append(right_rpm)
    time_values.append(len(left_rpm_values))
    
    np_time_values = np.array(time_values, dtype=np.float64)
    np_left_rpm_values = np.array(left_rpm_values, dtype=np.float64)
    np_right_rpm_values = np.array(right_rpm_values, dtype=np.float64)

    np_time_values = np_time_values.flatten()
    np_left_rpm_values = np_left_rpm_values.flatten()
    np_right_rpm_values = np_right_rpm_values.flatten()

    plt.clf()

    # Plot the RPM values with matplotlib
    try:
        plt.plot(np_time_values, np_left_rpm_values, marker="o", label="Left RPM", color="blue")
        plt.plot(np_time_values, np_right_rpm_values, marker="o", label="Right RPM", color="orange")
        
        plt.xlabel("Time (iterations)")
        plt.ylabel("RPM")
        plt.title("Wheel RPMs Over Time")
        plt.legend()
        plt.pause(0.1)
        plt.draw()
    except Exception as e:
        print(f"Error during plotting: {e}")

if __name__ == "__main__":
    plt.ion()  # Turn on interactive mode
    plt.figure(figsize=(10, 6))

    while True:
        data = fetch_data_from_script_b()
        
        if data:
            left_rpm = data.get("left_rpm", 0)
            right_rpm = data.get("right_rpm", 0)

            print(f"Left RPM: {left_rpm}, Right RPM: {right_rpm}")

            # Process and plot the data
            process_and_plot_data(left_rpm, right_rpm)

        # Sleep for 1 second (to simulate 1Hz loop)
        time.sleep(1)
