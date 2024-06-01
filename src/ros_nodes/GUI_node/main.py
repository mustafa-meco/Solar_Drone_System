import tkinter as tk
from tkinter import messagebox
import threading
import rospy


# Thread dictionary to store running threads
threads = {}

# Importing main functions from other files
from ros_nodes.camera_adapter_node import bootstrap as camera_adapter_bootstrap
from ros_nodes.camera_reader_node import bootstrap as camera_reader_bootstrap
from ros_nodes.frames_saving_node import bootstrap as frames_saving_bootstrap

def start_node(node_name, bootstrap_function):
    if node_name in threads and threads[node_name].is_alive():
        messagebox.showinfo("Info", f"{node_name} is already running")
        return

    rospy.init_node(f"{node_name}_node", anonymous=True)
    thread = threading.Thread(target=bootstrap_function)
    thread.start()
    threads[node_name] = thread
    messagebox.showinfo("Execution", f"{node_name} started")

def stop_node(node_name):
    if node_name in threads and threads[node_name].is_alive():
        rospy.signal_shutdown(f"{node_name} shutdown")
        threads[node_name].join()
        messagebox.showinfo("Execution", f"{node_name} stopped")
    else:
        messagebox.showinfo("Info", f"{node_name} is not running")

def main():
    global threads
    threads = {}

    # Creating the main window
    root = tk.Tk()
    root.title("ROS Node Manager")

    # Creating buttons to start and stop ROS nodes
    button1_start = tk.Button(root, text="Start Camera Reader Node", command=lambda: start_node("camera_reader", camera_reader_bootstrap.bootstrap))
    button1_stop = tk.Button(root, text="Stop Camera Reader Node", command=lambda: stop_node("camera_reader"))
    button2_start = tk.Button(root, text="Start Camera Adapter Node", command=lambda: start_node("camera_adapter", camera_adapter_bootstrap.bootstrap))
    button2_stop = tk.Button(root, text="Stop Camera Adapter Node", command=lambda: stop_node("camera_adapter"))
    button3_start = tk.Button(root, text="Start Frames Saving Node", command=lambda: start_node("frames_saving", frames_saving_bootstrap.bootstrap))
    button3_stop = tk.Button(root, text="Stop Frames Saving Node", command=lambda: stop_node("frames_saving"))

    # Placing buttons on the window
    button1_start.pack(pady=5)
    button1_stop.pack(pady=5)
    button2_start.pack(pady=5)
    button2_stop.pack(pady=5)
    button3_start.pack(pady=5)
    button3_stop.pack(pady=5)

    # Running the main loop
    root.mainloop()