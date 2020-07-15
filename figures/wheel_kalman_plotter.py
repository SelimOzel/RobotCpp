import matplotlib.pyplot as plt
import csv

def plotter(nameCSV_IN, nameFigure_IN):
	alpha = [] # [rad]
	alpha_dot = [] # [rad/s]

	xPos = [] # [m]
	yPos = [] # [m]
	angle = [] # [rad]

	torque = [] # [Nm]
	w = [] # [rad/s]

	t = [] # [s]

	with open(nameCSV_IN) as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			alpha.append(float(row[1]))
			alpha_dot.append(float(row[2]))
			xPos.append(float(row[3]))
			yPos.append(float(row[4]))
			angle.append(float(row[5]))
			torque.append(float(row[6]))
			w.append(float(row[7]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title(nameFigure_IN)

	plt.subplot(321)
	plt.plot(xPos,yPos, 'r')
	plt.xlabel('xPos [m]')
	plt.ylabel('yPos [m]')
	plt.grid()

	plt.subplot(322)
	plt.plot(t,alpha, 'b')	
	plt.xlabel('Time [s]')
	plt.ylabel('Wheel Angle [rad]')
	plt.grid()

	plt.subplot(323)
	plt.plot(t,angle , 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Wheel Angle Global Frame [rad]')
	plt.grid()

	plt.subplot(324)
	plt.plot(t,alpha_dot, 'g')	
	plt.xlabel('Time [s]')
	plt.ylabel('Wheel Angular Speed [rad/s]')
	plt.grid()

	plt.subplot(325)
	plt.plot(t,w , 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Wheel Angular Speed Global Frame [rad/s]')
	plt.grid()	

	plt.subplot(326)
	plt.plot(t,torque, 'k')
	plt.xlabel('Time [s]')
	plt.ylabel('Torque [Nm]')
	plt.grid()	

	plt.show()	

def main():
	plotter("../wheel_1kg_kalman_constant_torque.csv", "Advanced Wheel 1 kg with Kalman - Constant Torque")
	plotter("../wheel_2kg_kalman_constant_torque.csv", "Advanced Wheel 2 kg with Kalman - Constant Torque")

if __name__ == "__main__":
    main()