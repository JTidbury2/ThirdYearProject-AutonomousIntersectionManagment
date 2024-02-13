import subprocess
import re

def run_simulation(simulation_cmd):
    # Run the simulation command and capture the output
    result = subprocess.run(simulation_cmd, stdout=subprocess.PIPE, text=True, shell=True)
    output = result.stdout

    # Initialize metrics
    metrics = {
        'crash_list_length': 0,
        'avg_delay': 0,
        'max_delay': 0,
        'actual_total_flow': 0,
    }

    # Extract metrics from the output
    crash_list_pattern = re.compile(r'longest_crash_list = (\[.*?\])')
    avg_delay_pattern = re.compile(r'avg_delay = ([\d\.]+)')
    max_delay_pattern = re.compile(r'max_delay = ([\d\.]+)')
    actual_total_flow_pattern = re.compile(r'actual_total_flow = ([\d\.]+)')

    # Process patterns
    for pattern, key in [(crash_list_pattern, 'crash_list_length'),
                         (avg_delay_pattern, 'avg_delay'),
                         (max_delay_pattern, 'max_delay'),
                         (actual_total_flow_pattern, 'actual_total_flow')]:
        match = pattern.search(output)
        if match:
            if key == 'crash_list_length':
                crash_list = eval(match.group(1))  # Convert string representation of list to actual list
                metrics[key] = len(crash_list)
            else:
                metrics[key] = float(match.group(1))

    return metrics

def main(simulation_cmd, num_runs):
    # Initialize total metrics
    total_metrics = {
        'crash_list_length': 0,
        'avg_delay': 0,
        'max_delay': 0,
        'actual_total_flow': 0,
    }

    for _ in range(num_runs):
        metrics = run_simulation(simulation_cmd)
        for key in total_metrics:
            total_metrics[key] += metrics[key]

    # Calculate and print averages
    for key, total in total_metrics.items():
        average = total / num_runs
        print(f'Average {key.replace("_", " ")} over {num_runs} runs: {average}')

if __name__ == '__main__':
    simulation_command = 'python main.py Dresner 10000'
    number_of_runs = 10  # Specify the number of times you want to run the simulation

    main(simulation_command, number_of_runs)
