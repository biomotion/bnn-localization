
import csv
from decimal import *
import math
from math import *
import sys
import numpy
from numpy import *

def main():

	# Ground Truth and Measurement Path
	file_gt = sys.argv[1]
	file_meas = sys.argv[2]

	gt_list = read_csv(file_gt)
	meas_list = read_csv(file_meas)

	print ("\n***Calculation Starts***\n")
	score, yaw_score, pitch_score, roll_score = calscore(gt_list, meas_list)
	print "\nNumber of your frames: %d" %(len(meas_list))


	print "Total score: %s %s %s %s" %(score, yaw_score, pitch_score, roll_score)
	print "final score: %s\n" %(0.4*score + 0.4* yaw_score + 0.1*pitch_score + 0.1*roll_score)
	print ("***Calculation Finished***")

def min_angle(gt, meas):
    _list = [gt-2*math.pi - meas, gt - meas, gt + 2 * math.pi - meas]
    return min(_list)

def calscore(_gt, _meas):

	_dist_score = []
	_yaw_score = []
	_pitch_score = []
	_roll_score = []
	_total_yaw_score = 0
	_total_dist_score = 0
	_total_pitch_score = 0
	_total_roll_score = 0

	for i in range(len(_gt)):
		Notfound = True
		for m in range(len(_meas)):
			meas = _meas[m]
			if( _gt[i][0] == meas[0] ):
				gt = numpy.array(_gt[i][1:3])
				mm = numpy.array(meas[1:3])
				dist = numpy.linalg.norm(gt-mm)
				_dist_score.append(grading(dist, float(sys.argv[3])))
				_yaw_score.append(grading(min_angle(_gt[i][4], meas[4]), float(sys.argv[4])))
				_pitch_score.append(grading(min_angle(_gt[i][5], meas[5]), float(sys.argv[5])))
				_roll_score.append(grading(min_angle(_gt[i][6], meas[6]), float(sys.argv[6])))
				Notfound = False
        if Notfound:     
			_dist_score.append(0)
			_yaw_score.append(0)
			_pitch_score.append(0)
			_roll_score.append(0)               
	_total_dist_score = sqrt(mean(square(_dist_score)))
	_total_yaw_score = sqrt(mean(square(_yaw_score)))
	_total_pitch_score = sqrt(mean(square(_pitch_score)))
	_total_roll_score = sqrt(mean(square(_roll_score)))

	return _total_dist_score, _total_yaw_score, _total_pitch_score, _total_roll_score
	
def grading(distance, sigma):

	if distance < sigma :
		return 1
	elif distance > sigma and distance < 2 * sigma:
		return 0.7
	elif distance > 2 * sigma and distance < 3 * sigma:
		return 0.3
	else:
		return 0



def read_csv(_csv_file):

	_list = []

	f= open(_csv_file, 'r')
	data = csv.reader(f)

	for row in data:
		if row:
			_list.append(row)

	f.close()

	_list = check_data_type(_list)

	return _list


def check_data_type(_list):

	for m in range(len(_list)):
		_list[m][0]=str(_list[m][0])
		_list[m][1]=float(_list[m][1]) 
		_list[m][2]=float(_list[m][2]) 
		_list[m][3]=float(_list[m][3])
		_list[m][4]=float(_list[m][4])
		_list[m][5]=float(_list[m][5])
		_list[m][6]=float(_list[m][6])

	return _list

if __name__ == '__main__':
	main()
