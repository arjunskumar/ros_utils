import matplotlib.pyplot as plt

# Read data from the text file
file_path = 'i7_plot.txt'  # replace with the actual path to your file
skip_rows = 25
max_rows_to_plot = 13000

with open(file_path, 'r') as file:
    all_lines = file.readlines()
num_rows_to_plot = min(max_rows_to_plot, len(all_lines) - 1)

# Extract Elapsed Time, Real (MB), and CPU% values
elapsed_time = []
real_mb = []
cpu_percent = []

for line in all_lines[1:num_rows_to_plot + 1:skip_rows]:  # Skip the header line
    data = line.strip().split()
    elapsed_time.append(float(data[0]))
    real_mb.append(float(data[2]))
    cpu_percent.append(float(data[1]))

# Plotting
fig, ax1 = plt.subplots(figsize=(10, 5))



# Create a second y-axis to the left for CPU%
color = 'tab:red'
ax1.set_xlabel('Elapsed Time')
ax1.set_ylabel('CPU%', color=color)
ax1.plot(elapsed_time, cpu_percent,  label='CPU%', color=color)
ax1.tick_params(axis='y', labelcolor=color)


# Plot Elapsed Time vs Real (MB) on the right y-axis
color = 'darkblue'
ax2 = ax1.twinx()
#ax2.set_xlabel('Elapsed Time')
ax2.set_ylabel('Real Memory (MB)', color=color)
ax2.plot(elapsed_time, real_mb, label='Real Memory (MB)', color=color)
ax2.tick_params(axis='y', labelcolor=color)



fig.tight_layout()  # Adjust layout to prevent overlapping
plt.title('Elapsed Time vs Real Memory (MB) and CPU%')
plt.grid(False)

plt.show()
