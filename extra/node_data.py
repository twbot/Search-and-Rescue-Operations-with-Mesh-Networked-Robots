#!/usr/bin/env python3
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib import markers as mk
import numpy as np
import math
from scipy.signal import lfilter
import pandas as pd
import argparse
import csv
import os

start = 1533423578.4893882

filtered_data = []

data_frame = []
model_values = []
a = 1
b = 1
c = 0

def collect_data():
	cur_dir = os.getcwd()
	file_path = os.path.join(cur_dir, 'extra.csv')
	global data_frame
	data_frame = pd.read_csv(file_path)

def connectpoints(x,y,p1,p2, color, linetype, labelname=None):
    x1, x2 = x, p1
    y1, y2 = y, p2
    if labelname:
    	plt.plot([x1,x2],[y1,y2],color, linestyle=linetype, label=labelname)
    else:
    	plt.plot([x1,x2],[y1,y2],color, linestyle=linetype)

def gauss(args):
	val = data_frame.iloc[0]
	for i in range(0, 10):
		data_frame.loc[0] = [val[0], val[1]]  # adding a row
		data_frame.index = data_frame.index + 1  # shifting index
	data_frame.sort_index(inplace=True) 
	last_vals = data_frame[:10]

	gauss_vals = []
	def function(x, var, mu):
		if var == 0:
			var = 1
		return (1/(var*math.sqrt(2*math.pi)))*math.pow(math.e, (-math.pow((x-mu), 2))/(2*math.pow(var, 2)))

	def variance_func(val_list, avg):
		value = 0
		for val in val_list:
			value = value + math.pow((val-avg), 2)
		value = value/(len(val_list)-1)
		return value

	index = 0
	for index, row in data_frame.iterrows():
		avg = sum(last_vals[-args.window:]['rssi'])/args.window
		variance = variance_func(last_vals[-args.window:]['rssi'], avg)
		value = function(row['rssi'], variance, avg)
		if index == 0:
			value_list = [value]*10
			gauss_vals = gauss_vals + value_list
		gauss_vals.append(value)
		avg_gauss_vals = sum(gauss_vals[-args.gauss_val:])/args.gauss_val
		count = 0
		if value > avg_gauss_vals:
			count = count+1
			val_tuple = (row['rssi'], row['distance'])
			filtered_data.append(val_tuple)
		last_vals.append(row)
		index = index+1

def original_model(x):
	return a*math.pow(math.e, (b*x+c))

def determine_model():
	ind = np.arange(1,10, .1)
	for val in ind:
		point = (val, original_model(val))
		print(original_model(val))
		plt.scatter(val, original_model(val))
	for val in values_test_1[5:]:
		plt.plot(original_model)
	plt.show()

def find_parameters():
	# R^2 = 1 - (R*pnt)/(R*avg)
	cost = 1000
	while not (cost < 0.5):
		for val in model:
			r_pnt = (r_pnt,r_avg)
			cost = 1 - (r_pnt/r_avg)

def main(args):
	collect_data()
	gauss(args)
	rssi_min = 50
	rssi_max = 0
	for index, row in data_frame.iterrows():
		if row['rssi'] < rssi_min:
			rssi_min = row['rssi']
		if row['rssi'] > rssi_max:
			rssi_max =row['rssi']

	time_max = data_frame.iloc[-1]['distance']
	time_min = data_frame.iloc[0]['distance']

	plt.figure(figsize=(15, 6))
	plt.title("Distance Approximation with Gaussian Filtering")
	plt.xlim(0,10)
	plt.ylim(rssi_min-0.5, rssi_max+0.5)
	plt.xlabel('Distance (m)')
	plt.ylabel('RSSI')

	old_index = 0
	for index, row in data_frame[1:].iterrows():
		time_val = row['distance']-start
		past_time_val = data_frame.iloc[old_index]['distance']-start
		past_val = data_frame.iloc[old_index]['rssi']
		if old_index == 0:
			plt.scatter(time_val, row['rssi'], marker='.', color='b', label="Unfiltered Data")
			connectpoints(time_val, row['rssi'], past_time_val, past_val, 'k', 'solid', "Unfiltered Model")
		else:
			plt.scatter(time_val, row['rssi'], marker='.', color='b')
			connectpoints(time_val, row['rssi'], past_time_val, past_val, 'k', 'solid')
		
		old_index = old_index +1

	# plt.show()

	old_index = 0
	print('----------------')
	for val in filtered_data[1:]:
		time_val = val[1]-start
		past_time_val = filtered_data[old_index][1]-start
		past_val = filtered_data[old_index][0]
		# print(time_val)
		if old_index == 0:
			plt.scatter(time_val, val[0], marker='o', color='r', label="Filtered Data")
			connectpoints(time_val, val[0], past_time_val, past_val, 'b', 'dashed', "Filtered Model")
		else:
			plt.scatter(time_val, val[0], marker='o', color='r')
			connectpoints(time_val, val[0], past_time_val, past_val, 'b', 'dashed')
		old_index = old_index +1

	plt.legend()
	plt.show()

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser()
	parser.add_argument('gauss_val', help='Gauss Val', type=int,  default=9, nargs='?')
	parser.add_argument('window', help='Number of history data points to analyze', nargs='?', type=int, default=5)
	args = parser.parse_args()
	main(args)