import matplotlib.pyplot as plt
import csv
from math import pi

def plotter(nameCSV_IN, nameFigure_IN):
	y = [] # [rad]
	y_dot = [] # [rad/s]
	acceleration = [] # [rad/s^2]

	t = [] # [s]

	with open(nameCSV_IN) as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			y.append(float(row[1]))
			y_dot.append(float(row[2]))
			acceleration.append(float(row[3]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title(nameFigure_IN)

	plt.subplot(311)
	plt.plot(y,y_dot, 'r')
	plt.xlabel('Position [m]')
	plt.ylabel('Speed [m/s]')
	plt.grid()

	plt.subplot(312)
	plt.plot(t,y, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Position [m]')
	plt.grid()

	plt.subplot(313)
	plt.plot(t,y_dot, 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Speed [m/s]')
	plt.grid()

	plt.show()	

def main():
	plotter("../hopper_vertical.csv", "Vertical Hopper")

if __name__ == "__main__":
    main()