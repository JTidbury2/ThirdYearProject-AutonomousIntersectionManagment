import subprocess
import re

def run_simulation(simulation_cmd):
    # Run the simulation command and capture the output
    result = subprocess.run(simulation_cmd, stdout=subprocess.PIPE, text=True, shell=True)
    output = result.stdout

    # Extract metrics from the output
    crash_list_pattern = re.compile(r'longest_crash_list = (\[.*?\])')
    crash_list_match = crash_list_pattern.search(output)
    if crash_list_match:
        crash_list_str = crash_list_match.group(1)
        crash_list = eval(crash_list_str)  # Convert string representation of list to actual list
        crash_list_length = len(crash_list)
    else:
        crash_list_length = 0

    return {
        'crash_list_length': crash_list_length,
        # Add other metrics extraction as needed
    }

def main(simulation_cmd, num_runs):
    total_crash_list_length = 0
    # Initialize variables for other metrics as needed

    for _ in range(num_runs):
        metrics = run_simulation(simulation_cmd)
        total_crash_list_length += metrics['crash_list_length']
        # Update totals for other metrics as needed

    # Calculate averages
    avg_crash_list_length = total_crash_list_length / num_runs
    # Calculate averages for other metrics as needed

    print(f'Average crash list length over {num_runs} runs: {avg_crash_list_length}')
    # Print averages for other metrics as needed

if __name__ == '__main__':
    simulation_command = 'python main.py Dresner 10000'
    number_of_runs = 10  # Specify the number of times you want to run the simulation

    main(simulation_command, number_of_runs)
