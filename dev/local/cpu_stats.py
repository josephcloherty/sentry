import subprocess
import psutil

def get_cpu_stats():
   # Get CPU temperature 
    temp_result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
    temp_str = temp_result.stdout.strip()
    temp_celsius = float(temp_str.split('=')[1].split("'")[0])
   # Get CPU Usage
    cpu_usage = psutil.cpu_percent(interval=None)
    return temp_celsius, cpu_usage