import matplotlib.pyplot as plt
import csv
from math import pi

def plotter(nameCSV_IN, nameFigure_IN):
	theta = [] # [rad]
	theta_dot = [] # [rad/s]
	angular_acceleration_input = [] # [rad/s^2]

	t = [] # [s]

	with open(nameCSV_IN) as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			theta.append(float(row[1])*(180/pi))
			theta_dot.append(float(row[2])*(180/pi))
			angular_acceleration_input.append(float(row[3]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title(nameFigure_IN)

	plt.subplot(311)
	plt.plot(theta,theta_dot, 'r')
	plt.xlabel('Angle [deg]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()

	plt.subplot(312)
	plt.plot(t,theta, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Angle [deg]')
	plt.grid()

	plt.subplot(313)
	plt.plot(t,theta_dot, 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()

	plt.show()	

def main():
	plotter("../pendulum_pid.csv", "Pendulum - Angle, speed & phase")

if __name__ == "__main__":
    main()