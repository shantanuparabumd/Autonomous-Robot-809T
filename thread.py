import threading
import time
# Define a function for the thread
def my_thread_func(name, delay):
    count = 0
    while count < 5:
        print("Thread {} slept for {} seconds".format(name, delay))
        count += 1
        time.sleep(delay)

# Create two threads as an example
t1 = threading.Thread(target=my_thread_func, args=("Thread 1", 1))
t2 = threading.Thread(target=my_thread_func, args=("Thread 2", 2))

# Start the threads
t1.start()
t2.start()

# Wait for the threads to finish
t1.join()
t2.join()

print("Done.")
