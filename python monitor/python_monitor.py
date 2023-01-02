import time
import matplotlib.pyplot as plt
import serial


def create_serial():
    ser = serial.Serial('COM3', 9600, timeout=1)
    return ser


s = create_serial()


def create_plot():
    """
    Creates a plot window which will be updated in real time
    and reads the data from the serial port
    """
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    xs = []
    ys = []
    while True:
        try:
            data = s.readline()
            data = data.decode('utf-8')
            data = data.strip()
            data = data.split(',')
            data = [int(i) for i in data]
            x = data[0]
            y = data[1]
            xs.append(x)
            ys.append(y)
            ax.clear()
            ax.plot(xs, ys)
            fig.canvas.draw()
            plt.pause(0.0001)
        except KeyboardInterrupt:
            print('exiting')
            break


i = 0
plt.axis([0, 10, 0, 50])
a = [0.0] * 100
b = [0.0] * 100
c = [0.0] * 100
t = []

for s1 in range(100):
    t.append(s1 * 0.1)

while True:
    res = s.readline()
    a1, b1, c1 = str(res[:-2].decode("utf-8")).split(';')

    a.append(float(a1))
    a.pop(0)
    b.append(float(b1))
    b.pop(0)
    c.append(float(c1))
    c.pop(0)

    print(f'COM: {a1}, {b1}, {c1}; {i}')
    # time.sleep(0.1)
    i += 1
    plt.clf()
    plt.plot(t, a)
    plt.plot(t, b)
    plt.plot(t, c)
    # plt.draw()
    plt.pause(0.01)

# plt.show()
