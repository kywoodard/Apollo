#!/usr/bin/python

import sys
import numpy as np
import csv
import time
from matplotlib import pyplot as plt
from matplotlib import animation

def main():
	global lines, dt, time_text, t_arr, y, z, dy, dz, theta, dtheta, m, ax
	global thrust, torque, rocket_y_offset, rocket_z_offset, scale_thrust
	global thrust_y_offset, thrust_z_offset
	y_platform = np.array([-5,5,5,-5,-5])
	z_platform = np.array([0,0,-1,-1,0])

	# line_platform.set_data(y_platform,z_platform)

	plotcolors = ["black","red"]
	linewidth = [2,3]
	lines = []

	path = 'rocket_ODE_output.txt'
	with open(path,'rb') as f:
		reader = csv.reader(f,delimiter='\t')
		row_count = 0
		for line in reader:
			row_count = row_count + 1
		col_count = len(line)-1

	t_arr = np.empty(shape=(row_count,1))
	X_arr = np.empty(shape=(row_count,col_count))

	with open(path,'rb') as f:
		incr= 0;
		reader = csv.reader(f,delimiter='\t')
		for line in reader:
			t_arr[incr] = float(line[0])
			for i in range(1,col_count+1):
				X_arr[incr][i-1] = float(line[i])
			incr = incr + 1;

	dt = np.round((t_arr[1]-t_arr[0])*1000.)	#Delta time in milliseconds 
	num_frames = int(np.floor(row_count/10))

	y = np.zeros(shape=(row_count,1))
	z = np.zeros(shape=(row_count,1))
	dy = np.zeros(shape=(row_count,1))
	dz = np.zeros(shape=(row_count,1))
	theta = np.zeros(shape=(row_count,1))
	dtheta = np.zeros(shape=(row_count,1))
	m = np.zeros(shape=(row_count,1))
	thrust = np.zeros(shape=(row_count,1))
	torque = np.zeros(shape=(row_count,1))
	points_flag = False;
	for j in range(0,row_count):
		y[j] = X_arr[j][0]
		z[j] = X_arr[j][1]
		theta[j] = X_arr[j][3]
		dy[j] = X_arr[j][4]
		dz[j] = X_arr[j][5]
		dtheta[j] = X_arr[j][6]
		m[j] = X_arr[j][8]
		thrust[j] = X_arr[j][9]
		torque[j] = X_arr[j][10]
		if all([any([z[j] <= 0,m[j] <= 25.0]),not points_flag]):
			points_flag = True;
			points = [0.0]
			trans_speed = np.sqrt(np.square(dy[j])+np.square(dz[j]))
			if not any([np.fabs(y[j]) > 20., z[j] > 10.,
						np.fabs(theta[j]) > np.pi/6,trans_speed < 5.0,
						np.fabs(dtheta[j])>1.0]):
				points = points + 10 * (m[j]-25.)/150.
				points = points + 30 * (20.-np.fabs(y[j]))/20.
				points = points + 20 * (10.-z[j])/10.
				points = points + 20 * (np.pi/6. 
										- np.fabs(theta[j]))/(np.pi/6.)
				points = points + 10 * (5. - trans_speed)/5.
				points = points + 10 * (1.0 - np.fabs(dtheta[j]))/1.

		if j == row_count-1:
			if not points_flag:
				points = [0.0]
			print 'Points:',points[0]

	rocket_y_offset = np.array([0.,-1.,-1.,0.5,-0.5,-1,1,0.5,1.,1.,0.])
	rocket_z_offset = np.array([10.,8.,-10.,-10.,-10.,-13.,-13.,-10.,-10.,8.,
								10.])

	scale_thrust = 1.
	thrust_y_offset = np.array([0.,0.])

	try:
		if 'sim_on' == sys.argv[1]:
			fig = plt.figure()
			axes_lim_x = (-20,20)
			axes_lim_y = (900,1100)
			ax = plt.axes(xlim=axes_lim_x, ylim=axes_lim_y)
			plt.grid()
			line_platform, = ax.plot(y_platform, z_platform, lw=2)
			time_text = ax.text(0.95, 0.01,[],
		        verticalalignment='bottom', horizontalalignment='right',
		        transform=ax.transAxes,
		        color='green', fontsize=15)
			for index in range(2):
				lobj = ax.plot([],[],lw=linewidth[index],
					color=plotcolors[index])[0]
				lines.append(lobj)
			anim = animation.FuncAnimation(fig, animate, init_func=init,
								frames=num_frames, interval=dt)
			plt.show()
	except:
		pass

def init():
	for line in lines:
		line.set_data([],[])
	return lines

def animate(i):
	t1 = time.time()
	it = int(np.floor(i*10))
	time_text.set_text(str(60.0-t_arr[it]))
	max_axes = 100
	min_axes = 30
	alpha = min(max(z[it]/1000.,0.),1.)
	curr_axes = (1-alpha)*min_axes + alpha*max_axes

	y_rocket = (rocket_y_offset*np.cos(theta[it])
				- rocket_z_offset*np.sin(theta[it]))
	z_rocket = (rocket_y_offset*np.sin(theta[it])
				+ rocket_z_offset*np.cos(theta[it]))

	thrust_z_offset = np.array([-13.5,-13.5-scale_thrust*thrust[it]])
	
	y_thrust = (thrust_y_offset*np.cos(theta[it])
				- thrust_z_offset*np.sin(theta[it]))
	z_thrust = (thrust_y_offset*np.sin(theta[it])
				+ thrust_z_offset*np.cos(theta[it]))
	animate_data = [[y_rocket,z_rocket],[y_thrust,z_thrust]]

	axes_lim_x = (y[it]-curr_axes,y[it]+curr_axes)
	axes_lim_y = (z[it]-curr_axes,z[it]+curr_axes)
	ax.set_xlim(axes_lim_x[0], axes_lim_x[1])
	ax.set_ylim(axes_lim_y[0], axes_lim_y[1])
	for lnum,line in enumerate(lines):
		line.set_data(y[it] + animate_data[lnum][0],
					  z[it] + animate_data[lnum][1])
	t2 = time.time()
	time.sleep(max(0,dt/1000.*10.-(t2-t1)))
	return lines

if __name__ == "__main__":
	main()
