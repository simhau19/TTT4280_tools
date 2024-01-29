from remoteADC import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

HOST = "192.168.137.10"
PORT = 4280

ADC_RESOLUTION = 2**12
SUPPLY_VOLTAGE = 3.3
SAMPLE_COUNT = 5000
CHANNEL_COUNT = 3

with remoteADC() as adc:
    print(f"Connecting to {HOST}:{PORT}")
    adc.connect((HOST, PORT))
    ts , data = adc.read(SAMPLE_COUNT, CHANNEL_COUNT)
    t = np.linspace(0, ts*len(data), len(data))

    fig, ax = plt.subplots(1,1)

    def update(i):
        ax.clear()
        _ , data = adc.read(SAMPLE_COUNT, CHANNEL_COUNT)

        #remove bias
        data -= ADC_RESOLUTION/2

        #convert to voltage
        data *= SUPPLY_VOLTAGE/ADC_RESOLUTION

        #fix y-axis
        ax.set_ylim(-SUPPLY_VOLTAGE/2, SUPPLY_VOLTAGE/2)

        ln = ax.plot(t[1:], data[1:,:])
        return ln

    ani = FuncAnimation(fig, update,  interval = 200, blit=True)
    plt.show()



