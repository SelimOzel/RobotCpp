import matplotlib.pyplot as plt
import csv

def plotter(nameCSV_IN, nameFigure_IN):
	theta = [] # [rad]
	theta_dot = [] # [rad/s]
	angular_acceleration_input = [] # [rad/s^2]

	t = [] # [s]

	with open(nameCSV_IN) as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			theta.append(float(row[1]))
			theta_dot.append(float(row[2]))
			angular_acceleration_input.append(float(row[3]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title(nameFigure_IN)

	plt.subplot(311)
	plt.plot(theta,theta_dot, 'r')
	plt.xlabel('Angle [rad]')
	plt.ylabel('Angular speed [rad/s]')
	plt.grid()

	plt.subplot(312)
	plt.plot(t,theta, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Angle [rad]')
	plt.grid()

	plt.subplot(313)
	plt.plot(t,theta_dot , 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Angular speed [rad/s]')
	plt.grid()

	plt.show()	

def main():
	plotter("../_bin/pendulum_pid.csv", "Pendulum - Angle, speed & phase")

if __name__ == "__main__":
    main()