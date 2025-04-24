import numpy as np
import matplotlib.pyplot as plt

def plot_samples(file_path):
    # Read data from file
    with open(file_path, 'r') as file:
        data_str = file.read().strip()
    
    # Convert string to list of floating-point numbers, ignoring empty entries
    samples = np.array([float(value) for value in data_str.split(',') if value.strip()])

    num_samples = 50000
    sample_interval = 0.6e-3
    time_axis = np.arange(num_samples) * sample_interval
    
    # Ensure we have 30,000 samples
    if len(samples) != num_samples:
        print(f"Warning: Expected 50,000 samples but got {len(samples)}.")
    
    # Plot data with lines connecting points
    plt.figure(figsize=(20, 10))
    plt.plot(time_axis, samples, marker='.', linestyle='-', markersize=1, alpha=0.8)
    plt.xlabel("Sample Index")
    plt.ylabel("Value")
    plt.title("Current Measurements")
    plt.grid(True)
    plt.xticks(np.linspace(0, max(time_axis), num=7))

    png_path = file_path.rsplit(".", 1)[0] + ".png"
    
    # Save the plot instead of showing it
    plt.savefig(png_path)
    plt.close()

    voltage = 5

    wifi_curr_th = 0.03

    cpu_curr = np.average(samples[samples <= wifi_curr_th])

    total_energy = np.sum(samples) * sample_interval * voltage
    wifi_energy = np.sum((samples[samples > wifi_curr_th]-cpu_curr)) * sample_interval * voltage
    
    print("Average Operating Current: ", cpu_curr, "A")
    print("Total energy consumption: ", total_energy, "J")
    print("Wifi energy consumption: ", wifi_energy, "J") # include both listen and transmit

# Example usage
file_path = "random_4-2.csv"  # Replace with actual file path
plot_samples(file_path)