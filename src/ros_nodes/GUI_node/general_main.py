import tkinter as tk
from tkinter import messagebox
import threading
import rospy
import paramiko
import os
import time
import stat  # Import the stat module


# Importing main functions from other files
from ros_nodes.camera_adapter_node import bootstrap as camera_adapter_bootstrap
from ros_nodes.camera_reader_node import bootstrap as camera_reader_bootstrap
from ros_nodes.frames_saving_node import bootstrap as frames_saving_bootstrap

from lib.settings import get_settings
import logging
settings = get_settings()
# SSH configuration
DRONE_HOST = settings["networking"]["raspi"]
DRONE_PORT = 22
DRONE_USER = settings['security']["raspi"]["username"]
DRONE_PASSWORD = settings['security']["raspi"]["password"]  # Password for SSH

os.environ['ROS_MASTER_URI'] = f"http://{DRONE_HOST}:11311"
SYS_IP = os.popen('hostname -I').read().split()[0]
os.environ['ROS_IP'] = SYS_IP
os.environ['ROS_HOSTNAME'] = SYS_IP

# Function to transfer files via SFTP
def sftp_download_dir(remote_dir, local_dir):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(DRONE_HOST, DRONE_PORT, username=DRONE_USER, password=DRONE_PASSWORD)
        sftp = ssh.open_sftp()

        if not os.path.exists(local_dir):
            os.makedirs(local_dir)

        def download_directory(remote_directory, local_directory):
            os.makedirs(local_directory, exist_ok=True)
            for item in sftp.listdir_attr(remote_directory):
                remote_path = os.path.join(remote_directory, item.filename).replace('\\', '/')
                local_path = os.path.join(local_directory, item.filename).replace('\\', '/')
                if stat.S_ISDIR(item.st_mode):
                    download_directory(remote_path, local_path)
                else:
                    sftp.get(remote_path, local_path)

        download_directory(remote_dir, local_dir)

        sftp.close()
        ssh.close()
        messagebox.showinfo("Execution", "Files downloaded successfully")
    except Exception as e:
        messagebox.showerror("Error", f"Exception: {str(e)}")

def sftp_download_dir_thread(remote_dir, local_dir):
    threading.Thread(target=sftp_download_dir, args=(remote_dir, local_dir)).start()

def execute_ssh_command(host, port, user, password, command):
    def run_command():
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, port, username=user, password=password)

            full_command = (
                "source ~/.bashrc && "
                "source /opt/ros/noetic/setup.bash && "
                f"export ROS_MASTER_URI=http://{DRONE_HOST}:11311 && "
                f"export ROS_IP={DRONE_HOST} && "
                "cd ~/catkin_ws/src/Solar_Drone_System/src/ && "
                f"{command}"
            )

            print(f"Executing command: {full_command}")
            stdin, stdout, stderr = ssh.exec_command(full_command, get_pty=True)

            result = stdout.read().decode()
            error = stderr.read().decode()

            ssh.close()

            if error:
                print(f"Error: {error}")
                messagebox.showerror("Error", f"Error executing command on {host}: {error}")
            else:
                print("Command executed successfully")
                messagebox.showinfo("Execution", f"Command executed successfully on {host}")

        except Exception as e:
            messagebox.showerror("Error", f"Exception: {str(e)}")

    threading.Thread(target=run_command).start()

def start_remote_mavros():
    execute_ssh_command(DRONE_HOST, DRONE_PORT, DRONE_USER, DRONE_PASSWORD, "roslaunch mavros px4.launch")

def start_remote_master():
    execute_ssh_command(DRONE_HOST, DRONE_PORT, DRONE_USER, DRONE_PASSWORD, "roscore")


# Local node management functions
def start_local_node(node_name, bootstrap_function):
    if node_name in threads and threads[node_name].is_alive():
        messagebox.showinfo("Info", f"{node_name} is already running")
        return

    rospy.init_node(f"{node_name}_node", anonymous=True)
    thread = threading.Thread(target=bootstrap_function)
    thread.start()
    threads[node_name] = thread
    messagebox.showinfo("Execution", f"{node_name} started")

def stop_local_node(node_name):
    if node_name in threads and threads[node_name].is_alive():
        rospy.signal_shutdown(f"{node_name} shutdown")
        threads[node_name].join()
        messagebox.showinfo("Execution", f"{node_name} stopped")
    else:
        messagebox.showinfo("Info", f"{node_name} is not running")

# Remote node management functions
def start_remote_node(node_name, command):
    result = execute_ssh_command(DRONE_HOST, DRONE_PORT, DRONE_USER, DRONE_PASSWORD, command)
    messagebox.showinfo("Execution", f"Started {node_name} on drone:\n{result}")

def stop_remote_node(node_name, command):
    result = execute_ssh_command(DRONE_HOST, DRONE_PORT, DRONE_USER, DRONE_PASSWORD, command)
    messagebox.showinfo("Execution", f"Stopped {node_name} on drone:\n{result}")

def main():
    global threads
    threads = {}

    # Creating the main window
    root = tk.Tk()
    root.title("ROS Node Manager")

    # Thread dictionary to store running threads
    threads = {}

    # Local node buttons
    button1_start = tk.Button(root, text="Start Local Camera Reader Node", command=lambda: start_local_node("camera_reader", camera_reader_bootstrap.bootstrap))
    button1_stop = tk.Button(root, text="Stop Local Camera Reader Node", command=lambda: stop_local_node("camera_reader"))
    
    button3_start = tk.Button(root, text="Start Local Frames Saving Node", command=lambda: start_local_node("frames_saving", frames_saving_bootstrap.bootstrap))
    button3_stop = tk.Button(root, text="Stop Local Frames Saving Node", command=lambda: stop_local_node("frames_saving"))

    # Remote node buttons
    button5_start = tk.Button(root, text="Start Drone Camera Adapter Node", command=lambda: start_remote_node("drone_camera_adapter", "python3 ros_nodes/camera_adapter_node/bootstrap.py"))
    button5_stop = tk.Button(root, text="Stop Drone Camera Adapter Node", command=lambda: stop_remote_node("drone_camera_adapter", "pkill -f ros_nodes/camera_adapter_node/bootstrap.py"))
    button6_start = tk.Button(root, text="Start Drone Frames Saving Node", command=lambda: start_remote_node("drone_frames_saving", "python3 ros_nodes/frames_saving_node/bootstrap.py"))
    button6_stop = tk.Button(root, text="Stop Drone Frames Saving Node", command=lambda: stop_remote_node("drone_frames_saving", "pkill -f ros_nodes/frames_saving_node/bootstrap.py"))

    button7_start = tk.Button(root, text="Start MAVROS on Drone", command=lambda: start_remote_mavros())
    button8_start = tk.Button(root, text="Start ROS Master on Drone", command=lambda: start_remote_master())

    # add a text box to execute commands on the drone
    command_entry = tk.Entry(root, width=50)
    command_entry.pack(pady=5)
    button9_start = tk.Button(root, text="Execute command on Drone", command=lambda: execute_ssh_command(DRONE_HOST, DRONE_PORT, DRONE_USER, DRONE_PASSWORD, command_entry.get()))
    button9_start.pack(pady=5)
    
    # SFTP download button
    button_download = tk.Button(root, text="Download Files from Drone", command=lambda: sftp_download_dir_thread("/home/pi/frames/", "./frames/"))


    # Placing buttons on the window
    button1_start.pack(pady=5)
    button1_stop.pack(pady=5)
    button3_start.pack(pady=5)
    button3_stop.pack(pady=5)
    button5_start.pack(pady=5)
    button5_stop.pack(pady=5)
    button6_start.pack(pady=5)
    button6_stop.pack(pady=5)
    button7_start.pack(pady=5)
    button8_start.pack(pady=5)

    # show drone IP 
    drone_ip_label = tk.Label(root, text=f"Drone IP: {DRONE_HOST}")
    drone_ip_label.pack(pady=5)

    button_download.pack(pady=5)    
    
    
    # Running the main loop
    root.mainloop()
