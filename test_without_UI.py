import requests
import re
import pandas as pd
import datetime
from datetime import timezone
import os
import cv2
import glob
from natsort import natsorted
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
import numpy as np

# ______________________________________Correct here______________________________________
# parameters
filename = "src/test.csv"
out_path = "out/"
fig_path = "fig/"
save_video_flag = True

UPD_TIME = 500  # ms
D = 30  # mm

plot_time_step = 60  # s
plot_time_minor = 6
plot_speed_step = 100  # mm/s
plot_speed_minor = 5
# ________________________________________________________________________________________

rpm2mmps = D / 2 * 2 * np.pi / 60
mmps2rpm = 1 / rpm2mmps

# create output folders
try:
    os.makedirs(out_path, exist_ok=True)
except OSError as error:
    print("out_path folder error")
if save_video_flag:
    try:
        os.makedirs(fig_path, exist_ok=True)
    except OSError as error:
        print("fig_path folder error")


# loop function
def real_time_data(i, t, m1, m2):
    global counter, start_time, log_file_name, t_max, rpm2mmps, mmps2rpm
    connection_error_flag = False

    d = datetime.datetime.now(timezone.utc)
    current_time = d.hour * 3600 + d.minute * 60 + d.second + d.microsecond / 10 ** 6 - start_time

    # get rpm
    try:
        response = requests.get("http://192.168.4.2").content
    except requests.exceptions.ConnectionError as e:
        err = str(e)
        if "<title>Motor driver" in err:
            result = re.search('M1current: (.*) RPM </p><p> M1target:', err)
            m1_current = int(result.group(1)) * rpm2mmps
            m1.append(m1_current)
            result = re.search('M2current: (.*) RPM </p><p> M2target:', err)
            m2_current = int(result.group(1)) * rpm2mmps
            m2.append(m2_current)

            t.append(current_time)
            print("{} -> M1 = {} mm/s, M2 = {} mm/s".format(current_time, m1_current, m2_current))
            plt.plot(t, m1, color="#006400", linestyle='solid', linewidth=1)
            plt.plot(t, m2, color="#dc143c", linestyle='solid', linewidth=1)
            plt.legend(["M1 target", "M2 target", "M1 current", "M2 current"], loc=1)
            plt.xlabel('Time, s', fontsize=10)
            plt.ylabel('Belt speed, mm/s', fontsize=10)

            if save_video_flag:
                global name_counter
                plt.savefig("fig/" + str(name_counter) + ".png")
                name_counter += 1

        else:
            print("Failed")

    # set rpm
    d = datetime.datetime.now(timezone.utc)
    current_time = d.hour * 3600 + d.minute * 60 + d.second + d.microsecond / 10 ** 6 - start_time
    buff_size = len(raw_data[:, 0]) - 1

    if current_time - raw_data[counter, 0] >= -0.5:

        set_string = "M1_{:04d}_M2_{:04d}".format(int(raw_data[counter, 1]*mmps2rpm),
                                                  int(raw_data[counter, 2]*mmps2rpm))
        print(set_string)
        flag = False
        try:
            response = requests.post('http://192.168.4.2/setspeed', data={"setspeed": set_string})
        except requests.exceptions.ConnectionError as e:
            err = str(e)
            if "Wrong speed value" in err:
                print("Wrong input value")
            elif "Speed has been setted" in err:
                print("Speed has been changed")
                flag = True
            else:
                print("Connection error:")
                print(err)
                connection_error_flag = True

        if counter < buff_size and flag:
            counter += 1

    # exit condition
    if connection_error_flag:
        print("ABORTED")
        ani.event_source.stop()
        plt.savefig(out_path + log_file_name + ".png")
        plt.close()
    elif current_time >= t_max:
        print("DONE")
        ani.event_source.stop()
        plt.savefig(out_path + log_file_name + ".png")
        plt.close()


# read .csv file and plot target function
df = pd.read_csv(filename, header=0, low_memory=False)
raw_data = df.to_numpy()
t_max = max(raw_data[:, 0]) + plot_time_step / plot_time_minor

fig = plt.figure("Driver")
ax = fig.add_subplot(1, 1, 1)
ax.xaxis.set_major_locator(MultipleLocator(plot_time_step))
ax.xaxis.set_minor_locator(AutoMinorLocator(plot_time_minor))
ax.yaxis.set_major_locator(MultipleLocator(plot_speed_step))
ax.yaxis.set_minor_locator(AutoMinorLocator(plot_speed_minor))
ax.grid(which='major', color='#CCCCCC', linestyle='-')
ax.grid(which='minor', color='#CCCCCC', linestyle=':', alpha=0.2)
plt.xlim((-1) * plot_time_step / plot_time_minor, t_max)
plt.ylim((-1) * plot_speed_step / plot_speed_minor, 350)

plt.step(raw_data[:, 0], raw_data[:, 1], where='post', color='#2f4f4f', linestyle='dotted', alpha=0.8)
plt.step(raw_data[:, 0], raw_data[:, 2], where='post', color='#ff7f50', linestyle='dotted', alpha=0.8)

# call loop function
now = datetime.datetime.now(timezone.utc)
start_time = now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 10 ** 6

M1_RPM = []
M2_RPM = []
time = []
counter = 0
name_counter = 0
log_file_name = "{}_{}_{}_{}_log".format(now.year, now.month, now.day, now.hour * 3600 + now.minute * 60 + now.second)

ani = animation.FuncAnimation(fig, real_time_data, fargs=(time, M1_RPM, M2_RPM), interval=UPD_TIME)

plt.legend(["M1 target", "M2 target", "M1 current", "M2 current"], loc=1)
plt.xlabel('Time, s', fontsize=10)
plt.ylabel('Belt speed, mm/s', fontsize=10)
plt.show()

# Save logfile
data = "Time,M1,M2\n"
for i in range(0, len(time)):
    data = data + str(time[i]) + "," + str(M1_RPM[i]) + "," + str(M2_RPM[i]) + "\n"
f = open(out_path + log_file_name + ".csv", "w")
f.write(data)
f.close()

if save_video_flag:
    img_array = []
    glob_arr = natsorted(glob.glob(fig_path + '*.png'))
    print("Create " + log_file_name + ".avi")
    for i in glob_arr:
        img = cv2.imread(i)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)

    out = cv2.VideoWriter(out_path + log_file_name + ".avi", cv2.VideoWriter_fourcc(*'DIVX'), 30, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

    # remove pictures
    for i in glob_arr:
        try:
            os.remove(i)
        except OSError as error:
            print("Error while deleting file : ", i)
