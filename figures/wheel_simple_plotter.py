import matplotlib.pyplot as plt
import csv

def main():
	print("Simple Wheel Plotter")

	xPos = [] # [m]
	yPos = [] # [m]
	angle = [] # [rad]

	v = [] # [m/s]
	w = [] # [rad/s]

	t = [] # [s]

	with open('../_bin/wheel_output.csv') as csvfile:
		spamreader = csv.reader(csvfile)
		for row in spamreader:
			t.append(float(row[0])) 
			xPos.append(float(row[1]))
			yPos.append(float(row[2]))
			angle.append(float(row[3]))
			v.append(float(row[4]))
			w.append(float(row[5]))

	fig, ax = plt.subplots(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
	fig.canvas.set_window_title('Simple Wheel - Movement in X Axis')

	plt.subplot(221)
	plt.plot(xPos,yPos, 'r')
	plt.xlabel('xPos [m]')
	plt.ylabel('yPos [m]')
	plt.grid()

	plt.subplot(222)
	plt.plot(t,angle, 'b')
	plt.xlabel('Time [s]')
	plt.ylabel('Robot Angle [rad]')
	plt.grid()

	plt.subplot(223)
	plt.plot(t,v , 'g')
	plt.xlabel('Time [s]')
	plt.ylabel('Speed [m/s]')
	plt.grid()

	plt.subplot(224)
	plt.plot(t,w, 'k')	
	plt.xlabel('Time [s]')
	plt.ylabel('Angular Speed [rad/s]')
	plt.grid()

	plt.show()

if __name__ == "__main__":
    main()