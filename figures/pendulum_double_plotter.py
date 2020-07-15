import matplotlib.pyplot as plt
import csv
from math import pi

def plotter(nameCSV_IN, nameFigure_IN):
	theta1 = [] # [rad]
	theta2 = []
	theta1_dot = [] # [rad/s]
	theta2_dot = []
	angular_acceleration1_input = [] # [rad/s^2]
	angular_acceleration2_input = []

	t = [] # [s]

	with open(nameCSV_IN) as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			theta1.append(float(row[1])*(180/pi))
			theta1_dot.append(float(row[2])*(180/pi))
			theta2.append(float(row[3])*(180/pi))
			theta2_dot.append(float(row[4])*(180/pi))
			angular_acceleration1_input.append(float(row[5]))
			angular_acceleration2_input.append(float(row[6]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title(nameFigure_IN)

	# theta1
	plt.subplot(321)
	plt.plot(theta1,theta1_dot, 'r')
	plt.xlabel('Angle [deg]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()

	plt.subplot(323)
	plt.plot(t,theta1, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Angle [deg]')
	plt.grid()

	plt.subplot(325)
	plt.plot(t,theta2_dot, 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()

	# theta2
	plt.subplot(322)
	plt.plot(theta2,theta2_dot, 'r')
	plt.xlabel('Angle [deg]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()

	plt.subplot(324)
	plt.plot(t,theta2, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Angle [deg]')
	plt.grid()

	plt.subplot(326)
	plt.plot(t,theta2_dot, 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Angular speed [deg/s]')
	plt.grid()


	plt.show()	

def main():
	plotter("../pendulum_double.csv", "Pendulum - Angle, speed & phase")

if __name__ == "__main__":
    main()