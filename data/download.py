import gdown
import subprocess

gdown.download_folder("https://drive.google.com/drive/folders/1JYbWg0DKEVRHOrldbf5HeOtMCLxwBZXk")
subprocess.run("mv 2023-05-28-07-18-08_npz/*.npz .", shell=True)
subprocess.run("rm -rf 2023-05-28-07-18-08_npz", shell=True)
