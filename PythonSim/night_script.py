import subprocess
import re
import datetime

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

def main(num_runs, num_scenarios):
    all_results = {}

    for scenario in num_scenarios:
        simulation_command = f'python main.py Dresner {scenario} 0'
        print(f"Running simulation for scenario: {scenario}")

        # Initialize total metrics
        total_metrics = {
            'crash_list_length': 0,
            'avg_delay': 0,
            'max_delay': 0,
            'actual_total_flow': 0,
        }

        for _ in range(num_runs):
            metrics = run_simulation(simulation_command)
            for key in total_metrics:
                total_metrics[key] += metrics[key]

        # Calculate and store averages
        scenario_results = {}
        for key, total in total_metrics.items():
            average = total / num_runs
            scenario_results[key] = average

        all_results[scenario] = scenario_results

    # Get the current datetime
    now = datetime.datetime.now()
    file_name = f"night_result_{now.strftime('%Y%m%d_%H%M%S')}.txt"

    # Print and save results in a nice format
    with open(file_name, 'w') as f:
        for scenario, results in all_results.items():
            output = f"Results for scenario: {scenario}\n"
            f.write(output)
            print(output)
            for metric, value in results.items():
                output = f"Average {metric.replace('_', ' ')}: {value}\n"
                f.write(output)
                print(output)
            f.write("\n")

if __name__ == '__main__':
    number_of_runs = 1  # Specify the number of times you want to run the simulation
    num_scenarios = [500,1000, 2500]

    main(number_of_runs, num_scenarios)