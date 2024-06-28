import time
from multiprocessing import shared_memory
import numpy as np

# Create a new shared memory block
shm_name = 'test_data'
shm_size = 1024  # Adjust size as needed
shm = shared_memory.SharedMemory(create=True, name=shm_name, size=shm_size)

i = 0
data = np.zeros(2)

try:
    # Simulate continuous data acquisition
    while True:
        # Example data; replace with actual data acquisition logic
        data[0] = np.sin(100*i/360)
        data[1] = np.cos(100*i/360)

        i+=1

        data_bytes = data.tobytes()

        # Write data to shared memory
        shm.buf[:len(data_bytes)] = data_bytes
        print("data written")

        # Wait for a bit before next write
        time.sleep(0.05)
finally:
    # Clean up shared memory on exit
    shm.close()
    shm.unlink()
