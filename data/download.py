import os
import gdown
import subprocess

# Get the directory of the script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Change the working directory to the script's directory
os.chdir(script_dir)

# Download the folder from Google Drive
gdown.download_folder("https://drive.google.com/drive/folders/1JYbWg0DKEVRHOrldbf5HeOtMCLxwBZXk")

# Move all .npz files from the downloaded folder to the script's directory
subprocess.run("mv 2023-05-28-07-18-08_npz/*.npz .", shell=True)

# Remove the downloaded folder after moving its contents
subprocess.run("rm -rf 2023-05-28-07-18-08_npz", shell=True)

