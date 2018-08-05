#!/usr/bin/env python3
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib import markers as mk
import numpy as np
import math
from scipy.signal import lfilter

values_test_1 = [(22.0, 1533423578.4893882), (22.0, 1533423578.5243108), (22.0, 1533423578.5509465), (22.0, 1533423578.5985653), (22.0, 1533423578.6320543), (22.25, 1533423578.6657016), (22.5, 1533423578.702649), (22.75, 1533423578.7370794), (23.0, 1533423578.7705715), (22.75, 1533423578.804226), (22.5, 1533423578.8436701), (22.25, 1533423578.8772607), (22.0, 1533423578.9107177), (22.0, 1533423578.9456232), (22.0, 1533423578.9793105), (22.0, 1533423579.012896), (22.0, 1533423579.0577383), (22.0, 1533423579.0927415), (22.25, 1533423579.1262949), (22.75, 1533423579.1599264), (23.25, 1533423579.1948402), (23.5, 1533423579.2495172), (23.5, 1533423579.2844448), (23.0, 1533423579.3182337), (22.5, 1533423579.3516338), (22.25, 1533423579.3856337), (22.25, 1533423579.42034), (22.75, 1533423579.4779289), (24.0, 1533423579.5308754), (25.5, 1533423579.5659423), (27.0, 1533423579.5995305), (28.5, 1533423579.6373236), (29.25, 1533423579.6710186), (29.75, 1533423579.706057), (29.75, 1533423579.7393773), (29.75, 1533423579.7731774), (29.75, 1533423579.8082838), (30.0, 1533423579.8417363), (30.5, 1533423579.8751268), (31.0, 1533423579.9090526), (31.5, 1533423579.943794), (31.75, 1533423579.977367), (31.25, 1533423580.0336306), (30.25, 1533423580.066892), (29.0, 1533423580.1006718), (27.5, 1533423580.1355042), (26.75, 1533423580.169223), (26.0, 1533423580.2029228), (25.75, 1533423580.236653), (25.75, 1533423580.2715075), (25.75, 1533423580.3090525), (26.0, 1533423580.342724), (26.0, 1533423580.3762798), (26.0, 1533423580.4099367), (26.0, 1533423580.4435954), (26.25, 1533423580.4787216), (26.75, 1533423580.5120687), (27.25, 1533423580.5457795), (27.75, 1533423580.580727), (28.0, 1533423580.6072645), (28.5, 1533423580.6632924), (29.25, 1533423580.6968393), (30.0, 1533423580.730545), (30.75, 1533423580.7654605), (30.25, 1533423580.7992141), (29.25, 1533423580.8328235), (27.75, 1533423580.8663418), (26.0, 1533423580.9013329), (25.0, 1533423580.9347417), (24.25, 1533423580.969216), (24.0, 1533423581.003748), (24.0, 1533423581.0300248), (24.0, 1533423581.056647), (24.75, 1533423581.0902464), (25.75, 1533423581.128013), (26.75, 1533423581.1616774), (27.75, 1533423581.1953654), (27.75, 1533423581.2288942), (27.25, 1533423581.2637722), (26.75, 1533423581.2973857), (26.5, 1533423581.330986), (26.5, 1533423581.365929), (27.0, 1533423581.39952), (27.75, 1533423581.4332347), (28.5, 1533423581.466904), (29.5, 1533423581.5017936), (30.25, 1533423581.5354302), (30.75, 1533423581.5689454), (30.75, 1533423581.6027362), (30.0, 1533423581.6376913), (29.25, 1533423581.6713107), (28.25, 1533423581.7049448), (27.5, 1533423581.738585), (27.25, 1533423581.7763262), (27.0, 1533423581.81002), (27.25, 1533423581.8437257), (27.5, 1533423581.8788211), (28.25, 1533423581.934485), (29.25, 1533423581.9699702), (30.0, 1533423582.0049326), (30.75, 1533423582.0381396), (31.0, 1533423582.0746455), (31.0, 1533423582.1095452), (31.0, 1533423582.1432207), (31.0, 1533423582.1767757), (31.0, 1533423582.210378), (30.75, 1533423582.2440448), (30.5, 1533423582.2706382), (30.25, 1533423582.2971802), (29.5, 1533423582.344694), (29.0, 1533423582.3786442), (28.75, 1533423582.4120412), (28.5, 1533423582.4455943), (29.25, 1533423582.4819405), (29.75, 1533423582.5155473), (29.5, 1533423582.550424), (29.0, 1533423582.5843103), (27.75, 1533423582.617728), (26.5, 1533423582.6514053), (25.75, 1533423582.6869047), (25.0, 1533423582.7200708), (24.25, 1533423582.7535045), (23.75, 1533423582.7899623), (23.0, 1533423582.8248737), (22.5, 1533423582.8587158), (22.25, 1533423582.9145334), (22.0, 1533423582.948174), (22.25, 1533423582.9817336), (22.5, 1533423583.016698), (23.0, 1533423583.0502405), (23.75, 1533423583.083762), (24.5, 1533423583.1174934), (25.0, 1533423583.152435), (25.25, 1533423583.185911), (25.0, 1533423583.2196379), (24.5, 1533423583.2533023), (24.25, 1533423583.288249), (24.0, 1533423583.322271), (24.0, 1533423583.3555102), (24.25, 1533423583.39322), (24.5, 1533423583.4268951), (25.0, 1533423583.4607215), (26.0, 1533423583.494153), (27.0, 1533423583.5289834), (28.5, 1533423583.5626822), (30.0, 1533423583.5963113), (30.75, 1533423583.6299813), (31.25, 1533423583.6647234), (30.5, 1533423583.6997278), (29.5, 1533423583.7334526), (28.5, 1533423583.7671578), (27.25, 1533423583.80219), (26.5, 1533423583.856574), (25.75, 1533423583.8924522), (25.25, 1533423583.9487736), (25.0, 1533423584.0109675), (25.0, 1533423584.0613387), (25.0, 1533423584.1155164), (25.0, 1533423584.1490128), (25.0, 1533423584.182749), (25.0, 1533423584.2165518), (25.0, 1533423584.250157), (25.0, 1533423584.2880044), (25.0, 1533423584.3228972), (25.0, 1533423584.3788354), (25.0, 1533423584.412443), (25.0, 1533423584.4683967), (25.0, 1533423584.5019352), (25.0, 1533423584.535453), (25.25, 1533423584.5708013), (25.75, 1533423584.6053), (26.25, 1533423584.6390283), (26.75, 1533423584.6728954), (27.25, 1533423584.713634), (27.5, 1533423584.7469428), (27.75, 1533423584.7833579), (28.0, 1533423584.8154507), (27.75, 1533423584.8490663), (27.5, 1533423584.8840108), (27.0, 1533423584.9177096), (26.5, 1533423584.9512248), (26.25, 1533423584.9848778), (26.25, 1533423585.0412958), (26.75, 1533423585.0758698), (27.25, 1533423585.109572), (28.0, 1533423585.1460235), (28.5, 1533423585.1809263), (29.0, 1533423585.214542), (29.75, 1533423585.2483876), (30.25, 1533423585.3040907), (30.75, 1533423585.3382435), (31.25, 1533423585.37274), (31.5, 1533423585.406234), (32.25, 1533423585.4397748), (33.0, 1533423585.4735422), (33.5, 1533423585.5084834), (34.0, 1533423585.5350516), (33.0, 1533423585.5908756), (32.0, 1533423585.624553), (31.0, 1533423585.659322), (30.0, 1533423585.6929846), (30.0, 1533423585.7377334), (30.25, 1533423585.7713363), (30.5, 1533423585.8062432), (31.0, 1533423585.839873), (31.25, 1533423585.8958433), (31.0, 1533423585.9297707), (30.5, 1533423585.9855928), (29.75, 1533423586.0191529), (29.75, 1533423586.0528882), (30.25, 1533423586.0877788), (31.75, 1533423586.1212904), (34.5, 1533423586.1548855), (37.25, 1533423586.188992), (39.5, 1533423586.2237306), (40.25, 1533423586.2571492), (39.5, 1533423586.3132806), (38.0, 1533423586.3468032), (36.75, 1533423586.3804886), (36.0, 1533423586.4141724), (35.5, 1533423586.4489326), (35.5, 1533423586.485289), (35.25, 1533423586.5190077), (35.25, 1533423586.5541525), (35.25, 1533423586.5874746), (35.25, 1533423586.6213434), (35.25, 1533423586.6548707), (35.5, 1533423586.6897812), (35.75, 1533423586.716641), (36.0, 1533423586.7568421), (36.5, 1533423586.7946324), (36.75, 1533423586.8288248), (37.25, 1533423586.86332), (37.5, 1533423586.8968565), (37.75, 1533423586.9305923), (38.0, 1533423586.9667602), (38.0, 1533423587.0214818), (38.0, 1533423587.0550096), (38.0, 1533423587.088728), (38.0, 1533423587.1224606), (38.25, 1533423587.157285), (38.75, 1533423587.1908286), (39.5, 1533423587.2245133), (40.25, 1533423587.2595136), (41.0, 1533423587.2929912), (41.75, 1533423587.3280225), (42.5, 1533423587.361753), (44.0, 1533423587.3993206), (45.5, 1533423587.432994), (45.75, 1533423587.489136), (45.0, 1533423587.5449529), (43.25, 1533423587.5787983), (41.75, 1533423587.612076), (41.25, 1533423587.6458626), (41.5, 1533423587.672275), (42.25, 1533423587.7059612), (41.75, 1533423587.741189), (41.25, 1533423587.7745075), (42.25, 1533423587.81376), (43.5, 1533423587.8473272), (45.0, 1533423587.8813288), (46.5, 1533423587.917348), (46.5, 1533423587.950944), (45.75, 1533423587.9845455), (45.5, 1533423588.0183692), (45.0, 1533423588.0530682), (44.0, 1533423588.0866988), (42.5, 1533423588.1205032), (42.75, 1533423588.17625), (43.25, 1533423588.2029731), (44.25, 1533423588.2364933), (46.0, 1533423588.2701457), (46.0, 1533423588.3038766), (46.0, 1533423588.3372734), (46.0, 1533423588.3697257), (46.0, 1533423588.4033527), (46.0, 1533423588.436731), (46.0, 1533423588.470221), (46.0, 1533423588.5026662)]

start_time = 1533423578.4893882

filtered_data = []
list_vals = [values_test_1[1]]*10
values_test_2 = list_vals+values_test_1

def connectpoints(x,y,p1,p2):
    x1, x2 = x, p1
    y1, y2 = y, p2
    plt.plot([x1,x2],[y1,y2],'k-')

def gauss():
	# for val in values_test_2:
	# 	print(val[1]-start_time)
	last_vals = values_test_2[:10]

	def function(x, var, mu):
		return (1/(var*math.sqrt(2*math.pi)))*math.pow(math.e, (-math.pow((x-mu), 2))/(2*math.pow(var, 2)))

	def variance_func(val_list, avg):
		value = 0
		for val in val_list:
			value = value + math.pow((val[0]-avg), 2)
		value = value/(len(val_list)-1)
		return value

	for val in values_test_2:
		avg = sum(last_vals[-10:][0])/10
		variance = variance_func(last_vals[-10:], avg)
		# print(type(last_vals[-4:]))
		# print(last_vals[-4:])
		value = function(val[0], variance, avg)
		count = 0
		if value < 1.5269627206783045e-17:
			# print(val[1])
			count = count+1
			val_tuple = (val[0], val[1])
			filtered_data.append(val_tuple)
		print(count)
		last_vals.append(val)
		# print(val[0])
		# print(value)
	# print(filtered_data)

	# 2.4431398748369085e-18
	# 2.443139873359419e-18

def main():
	# for val in values_test_1:
	# 	print(val[1]-start_time)
	# print('-----+++++++++--------')
	gauss()
	# print('-----+++++++++--------')
	# for val in filtered_data:
	# 	print(val[1]-start_time)
	# print('-----+++++++++--------')
	rssi_min = 50
	rssi_max = 0
	for value in values_test_1:
		if value[0] < rssi_min:
			rssi_min = value[0]
		if value[0] > rssi_max:
			rssi_max = value[0]

	time_max = values_test_1[-1][1]
	time_min = values_test_1[0][0]

	plt.figure(figsize=(15, 6))
	plt.title("Distance Approximation without Gaussian Filtering")
	plt.xlim(0,10)
	plt.ylim(rssi_min-0.5, rssi_max+0.5)
	plt.xlabel('Distance (m)')
	plt.ylabel('RSSI')

	old_index = 0
	for val in values_test_1[1:]:
		# NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
		time_val = val[1]-start_time
		past_time_val = values_test_1[old_index][1]-start_time
		past_val = values_test_1[old_index][0]
		plt.scatter(time_val, val[0], marker='.', color='b')
		# plt.scatter(time_val, filtered_data[old_index+1][0])
		connectpoints(time_val, val[0], past_time_val, past_val)
		old_index = old_index +1

	plt.legend()
	plt.show()

	# plt.title("Distance Approximation without Gaussian Filtering")
	# plt.xlim(0,10)

	# plt.xlim(0,10)
	# plt.ylim(rssi_min-0.5, rssi_max+0.5)
	old_index = 0
	print('----------------')
	for val in filtered_data[1:]:
		time_val = val[1]-start_time
		past_time_val = filtered_data[old_index][1]-start_time
		past_val = filtered_data[old_index][0]
		print(time_val)
		plt.scatter(time_val, val[0], marker='.', color='r')
		connectpoints(time_val, val[0], past_time_val, past_val)
		old_index = old_index +1

	# plt.legend()
	plt.show()

if __name__ == '__main__':
	main()