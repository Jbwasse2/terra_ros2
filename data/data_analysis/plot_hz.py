import matplotlib.pyplot as plt
import numpy as np


def create_graphic(file_name, topic, fig_name):
    plt.cla()
    plt.clf()
    with open(file_name) as f:
        content = f.readlines()
    content = [float(x.strip()) for x in content]
    expected = [30 for i in range(len(content))]
    plt.title('Frequency of ' + str(topic) + ' over time')
    plt.xlabel('time (s)')
    plt.ylabel('Frequency (Hz)')
    plt.plot(content, label='actual')
    plt.plot(expected, label='expected')
    plt.legend()
    plt.savefig(fig_name, bbox_inches='tight')

create_graphic('cam_num.txt', 'Camera', 'cam.png')
create_graphic('twist_num.txt', 'Twist', 'twist.png')
