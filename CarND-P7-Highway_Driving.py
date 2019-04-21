#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Date: 04/XX/2019
	Programmer: John A. Betancourt G.
	Mail: john.betancourt93@gmail.com
    Web: www.linkedin.com/in/jhon-alberto-betancourt-gonzalez-345557129

Description: Project 7 - Udacity - self driving cars Nanodegree
    Highway Driving

Tested on: 
    python 3.5
    UBUNTU 16.04
"""

# =============================================================================
# LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPEN
# =============================================================================
import os

from multiprocessing import Pool 

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import subprocess
import csv

# =============================================================================
def run_process(process):                                                             
    os.system('{}'.format(process)) 

def getClipboardData():
    p = subprocess.Popen(['xclip','-selection', 'clipboard', '-o'], 
        stdout=subprocess.PIPE)
    retcode = p.wait()
    data = p.stdout.read()
    return data

def setClipboardData(data):
    p = subprocess.Popen(['xclip','-selection','clipboard'], 
        stdin=subprocess.PIPE)
    p.stdin.write(data)
    p.stdin.close()
    retcode = p.wait()

def main():

    # Copy command to clipboard to Create Video from desktop
    video_name = "path_planing_test_{}.mp4".format(1)
    command = "ffmpeg -video_size {}x{} -framerate {} -f x11grab -i :0.0+100,200 {}".format(
        1200, 850, 30, os.path.join(os.getcwd(), "video_results", video_name))
    setClipboardData(command.encode())

    # Run subprocess
    try:
        if not os.path.isdir("build"):
            os.mkdir("build")
            os.system('cd build && cmake .. && make') 
        else:
            os.system('clear && cd build && make') 

        processes = (
            "{}".format(os.path.join(os.getcwd(), "term3_sim_linux", "term3_sim.x86_64")),
            "{}".format(os.path.join(os.getcwd(), "build", "path_planning"))
        )
        # Run simulator and socket
        pool = Pool(processes=len(processes))                                                        
        pool.map(run_process, processes)
    except Exception as e: 
        print(str(e))

    print("Process has finished")

# =============================================================================
if __name__=="__main__":

    # Run all stuff
    main()

# =============================================================================