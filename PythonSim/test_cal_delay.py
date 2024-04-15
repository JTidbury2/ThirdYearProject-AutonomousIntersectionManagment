import pytest
from unittest.mock import mock_open, patch
from cal_delay import cal_metrics  # make sure your module is accessible

# Mock CSV data to be used across different tests
@pytest.fixture(scope="module")
def mock_csv_data():
    data = [
        "time,veh_id,zone,x,y,turn_dir,type",
        "100,1,ap,10,20,l,0",
        "200,1,ju,20,30,l,0",
        "300,1,ex,50,60,l,0",
        "100,2,ap,15,25,r,1",
        "200,2,ju,25,35,r,1",
        "250,2,ex,55,65,r,1",
        "crashed_Vehicle_ID: [1],2"
    ]
    return '\n'.join(data)

# Test for handling an empty file
# Test with normal data
def mock_csv_data():
    return """time,veh_id,zone,x,y,turn_dir,type
100,1,ap,10,20,l,0
200,1,ju,20,30,l,0
300,1,ex,50,60,l,0
100,2,ap,15,25,r,1
200,2,ju,25,35,r,1
250,2,ex,55,65,r,1
crashed_Vehicle_ID: [1],2
"""

def test_normal_operation(mock_csv_data):
    with patch("builtins.open", mock_open(read_data=mock_csv_data)) as mocked_file:
        metrics = cal_metrics("dummy_filename.csv")
        assert len(metrics['longest_crash_list']) == 2
        assert metrics['avg_delay'] >= 0
        assert metrics['max_delay'] >= 0

# Test actual total flow calculation
def test_actual_flow(mock_csv_data):
    with patch("builtins.open", mock_open(read_data=mock_csv_data)) as mocked_file:
        metrics = cal_metrics("dummy_filename.csv")
        expected_flow = 2 / ((300 - 200) * 0.1) * 3600  # Adjust based on your `veh_dt` value
        assert metrics['actual_total_flow'] == pytest.approx(expected_flow)

