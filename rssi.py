import subprocess
import sys, os, copy, json
from subprocess import Popen, PIPE
import string
import numpy as np
import math

#PartA: variable definition

#insert "iwconfig" on terminal and determine this variable
interface = 'wlan0'

#define BS
BS_database = []
BS_data1 = {'ssid':'MD402', 'f2.4':'8C:3B:AD:22:02:66', 'f5':'8C:3B:AD:22:02:68', 'locx':0.61, 'locy':7.04, 'locz':2.9, 'uuid':1}
BS_database.append(BS_data1)
BS_data2 = {'ssid':'MD402', 'f2.4':'00:11:32:9D:2B:30', 'f5':'00:11:32:9D:2B:31', 'locx':13.29, 'locy':1.05, 'locz':2.9, 'uuid':2}
BS_database.append(BS_data2)
BS_data3 = {'ssid':'MD402-1', 'f2.4':'00:11:32:9D:30:3A', 'f5':'00:11:32:9D:30:3B', 'locx':0.82, 'locy':-0.15, 'locz':0, 'uuid':3}
BS_database.append(BS_data3)
BS_data4 = {'ssid':'MD402-1', 'f2.4':'8C:3B:AD:21:FF:66', 'f5':'8C:3B:AD:21:FF:68', 'locx':14.24, 'locy':7.78, 'locz':0, 'uuid':4}
BS_database.append(BS_data4)

#PartB: RSSI value catching
class RSSI_Scan(object):
    # Allows us to declare a network interface externally.
    def __init__(self, interface):
        self.interface = interface

    
    def getRawNetworkScan(self, sudo=False):
        # Scan command 'iwlist interface scan' needs to be fed as an array.
        if sudo:
            scan_command = ['sudo','iwlist',self.interface,'scan']
        else:
            scan_command = ['iwlist',self.interface,'scan']
        # Open a subprocess running the scan command.
        scan_process = Popen(scan_command, stdout=PIPE, stderr=PIPE)
        # Returns the 'success' and 'error' output.
        (raw_output, raw_error) = scan_process.communicate() 
        # Block all execution, until the scanning completes.
        scan_process.wait()
        # Returns all output in a dictionary for easy retrieval.
        return {'output':raw_output,'error':raw_error}

    @staticmethod
    def getSSID(raw_cell):
        ssid = raw_cell.split('ESSID:"')[1]
        ssid = ssid.split('"')[0]
        return ssid

    
    @staticmethod
    def getQuality(raw_cell):
        quality = raw_cell.split('Quality=')[1]
        quality = quality.split(' ')[0]
        return quality

    @staticmethod
    def getAddress(raw_cell):
    	address = raw_cell.split('Address: ')[1]
    	address = address.split('\\')[0]
    	return address

    @staticmethod
    def getSignalLevel(raw_cell):
        signal = raw_cell.split('Signal level=')[1]
        signal = int(signal.split(' ')[0])
        return signal

    
    def parseCell(self, raw_cell):
        cell = {
            'ssid': self.getSSID(raw_cell),
            'address': self.getAddress(raw_cell),
            'signal': self.getSignalLevel(raw_cell)
        }
        return cell

    
    def formatCells(self, raw_cell_string):
        raw_cells = raw_cell_string.split('Cell') # Divide raw string into raw cells.
        raw_cells.pop(0) # Remove unneccesary "Scan Completed" message.
        if(len(raw_cells) > 0): # Continue execution, if atleast one network is detected.
            # Iterate through raw cells for parsing.
            # Array will hold all parsed cells as dictionaries.
            formatted_cells = [self.parseCell(cell) for cell in raw_cells]
            # Return array of dictionaries, containing cells.
            return formatted_cells
        else:
            print("Networks not detected.")
            return False
    
    @staticmethod
    def filterAccessPoints(all_access_points, network_names):
        focus_points = [] # Array holding the access-points of concern.
        # Iterate throguh all access-points found.
        for point in all_access_points:
            # Check if current AP is in our desired list.
            if point['ssid'] in network_names:
                focus_points.append(point)
        return focus_points
        
    def getAPinfo(self, networks=False, sudo=False):
        # TODO implement error callback if error is raise in subprocess
        # Unparsed access-point listing. AccessPoints are strings.
        raw_scan_output = self.getRawNetworkScan(sudo)['output'] 
        raw_scan_str = str(raw_scan_output)
        raw_scan_result = raw_scan_str[2:]
        # Parsed access-point listing. Access-points are dictionaries.
        all_access_points = self.formatCells(raw_scan_result)
        # Checks if access-points were found.
        if all_access_points:
            # Checks if specific networks were declared.
            if networks:
                # Return specific access-points found.
                return self.filterAccessPoints(all_access_points, networks)
            else:
                # Return ALL access-points found.
                return all_access_points
        else:
            # No access-points were found. 
            return False

#PartC: Matching
Basis = []
def power(index):
	return index['signal']

def num(index):
	return index[6]

def rssiRange(rssi, param):
	#d=10^((ABS(RSSI)-A)/(10*n))
	n = param[0]
	A = param[1]
	iRssi = abs(rssi)
	power = float(( iRssi - A ) / ( 10 * n ))
	result = pow(10 , power)
	return result

def matching(ap_info, BS_database, Basis):
	for cell in ap_info:
		for cmp_bs in BS_database:
			if cell['address'] == cmp_bs['f5']:
				data = []
				data.append(cmp_bs['locx'])
				data.append(cmp_bs['locy'])
				data.append(cmp_bs['locz'])
				data.append(cell['signal'])
				data.append(cell['ssid'])
				data.append(cell['address'])
				data.append(cmp_bs['uuid'])
				Basis.append(data)
				break
		#if len(Basis) >= 4:
			#break

#PartD: Locating
def triposition(Basis):
	if len(Basis) != 4:
		print("Wrong! ", len(Basis))
		return -1
	else:
		ld1 = pow(Basis[1][3], 2)-pow(Basis[1][0], 2)-pow(Basis[1][1], 2)-pow(Basis[1][2], 2)+pow(Basis[0][0], 2)+pow(Basis[0][1], 2)+pow(Basis[0][2], 2)-pow(Basis[0][3], 2)
		ld2 = pow(Basis[2][3], 2)-pow(Basis[2][0], 2)-pow(Basis[2][1], 2)-pow(Basis[2][2], 2)+pow(Basis[0][0], 2)+pow(Basis[0][1], 2)+pow(Basis[0][2], 2)-pow(Basis[0][3], 2)
		ld3 = pow(Basis[3][3], 2)-pow(Basis[3][0], 2)-pow(Basis[3][1], 2)-pow(Basis[3][2], 2)+pow(Basis[0][0], 2)+pow(Basis[0][1], 2)+pow(Basis[0][2], 2)-pow(Basis[0][3], 2)
		ldm = np.array([ld1, ld2, ld3])
		A = np.array([
			[2*(Basis[0][0]-Basis[1][0]), 2*(Basis[0][1]-Basis[1][1]), 2*(Basis[0][2]-Basis[1][2])],
			[2*(Basis[0][0]-Basis[2][0]), 2*(Basis[0][1]-Basis[2][1]), 2*(Basis[0][2]-Basis[2][2])],
			[2*(Basis[0][0]-Basis[3][0]), 2*(Basis[0][1]-Basis[3][1]), 2*(Basis[0][2]-Basis[3][2])]
			])
		ans = np.linalg.solve(A, ldm)
		return ans

def check(x, y, z):
	distance_dict = dict()
	r1 = math.sqrt(pow(x-0.61, 2)+pow(y-7.04, 2)+pow(z-2.9, 2)) 	#'f2.4':'00:11:32:9D:2B:30', 'f5':'00:11:32:9D:2B:31'
	r2 = math.sqrt(pow(x-13.29, 2)+pow(y-1.05, 2)+pow(z-2.9, 2)) 	#'f2.4':'8C:3B:AD:22:02:66', 'f5':'8C:3B:AD:22:02:68'
	r3 = math.sqrt(pow(x-0.82, 2)+pow(y+0.15, 2)+pow(z, 2))			#'f2.4':'00:11:32:9D:30:3A', 'f5':'00:11:32:9D:30:3B'
	r4 = math.sqrt(pow(x-14.24, 2)+pow(y-7.78, 2)+pow(z, 2))		#'f2.4':'8C:3B:AD:21:FF:66', 'f5':'8C:3B:AD:21:FF:68'
	par = [x, y, z]
	distance_dict = {"A": r1, "B": r2, "C": r3, "D": r4}
	return distance_dict

def get_rssi(rssi_db, distance_dict):
	resA = []
	resB = []
	resC = []
	resD = []
	res = []
	dis = []
	res.append(resA)
	res.append(resB)
	res.append(resC)
	res.append(resD)

	for i in range(10):
		print(i)
		Basis = []
		rssi_scanner = RSSI_Scan(interface)
		ap_info = rssi_scanner.getAPinfo(sudo=True)
		ap_info.sort(key=power, reverse=True)
		matching(ap_info, BS_database, Basis)
		Basis.sort(key=num)
		
		
		cnt = 0
		for ma in Basis:
			res[cnt].append(ma[3])
			cnt = cnt +1 

	for j in range(4):
		print(res[j])

	avg_dict = dict()
	avg_dict["A"] = np.mean(resA)
	avg_dict["B"] = np.mean(resB)
	avg_dict["C"] = np.mean(resC)
	avg_dict["D"] = np.mean(resD)

	for key in avg_dict.keys():
		if key not in rssi_db.keys():
			temp = dict()
			temp[distance_dict[key]] = [avg_dict[key]]
			rssi_db[key] = copy.deepcopy(temp)
		else:
			if distance_dict[key] not in rssi_db[key].keys():
				rssi_db[key][distance_dict[key]] = []
			rssi_db[key][distance_dict[key]].append(avg_dict[key])
	return rssi_db

	# dis.append(avgA)
	# dis.append(avgB)
	# dis.append(avgC)
	# dis.append(avgD)
	
def predict_rssi():
	resA = []
	resB = []
	resC = []
	resD = []
	res = []
	dis = []
	res.append(resA)
	res.append(resB)
	res.append(resC)
	res.append(resD)

	param_list = []
	param_list.append([6.17651989505506, -10.980833895238225])
	param_list.append([6.050075300423728, -13.777222841930119])
	param_list.append([3.624689527748884, 14.081706101821961])
	param_list.append([4.522151333850268, 3.5916879722896433])

	for i in range(10):
		print(i)
		Basis = []
		rssi_scanner = RSSI_Scan(interface)
		ap_info = rssi_scanner.getAPinfo(sudo=True)
		ap_info.sort(key=power, reverse=True)
		matching(ap_info, BS_database, Basis)
		Basis.sort(key=num)
		
		
		cnt = 0
		for ma in Basis:
			# res[cnt].append(ma[3])
			res[cnt].append(rssiRange(ma[3], param_list[cnt]))
			cnt = cnt +1 

	for j in range(4):
		print(res[j])

	avg_dict = dict()
	avg_dict["A"] = np.mean(resA)
	avg_dict["B"] = np.mean(resB)
	avg_dict["C"] = np.mean(resC)
	avg_dict["D"] = np.mean(resD)

	for key in avg_dict.keys():
		print(key, avg_dict[key])

	# testA = [0.61, 7.04, 2.9, rssiRange(avg_dict["A"], param_list[0])]
	# testB = [13.29, 1.05, 2.9, rssiRange(avg_dict["B"], param_list[1])]
	# testC = [0.82, -0.15, 0, rssiRange(avg_dict["C"], param_list[2])]
	# testD = [14.24, 7.78, 0, rssiRange(avg_dict["D"], param_list[3])]

	testA = [0.61, 7.04, 2.9, avg_dict["A"]]
	testB = [13.29, 1.05, 2.9, avg_dict["B"]]
	testC = [0.82, -0.15, 0, avg_dict["C"]]
	testD = [14.24, 7.78, 0, avg_dict["D"]]

	meas = []
	meas.append(testA)
	meas.append(testB)
	meas.append(testC)
	meas.append(testD)

	loc = triposition(meas)
	return loc

# xx = 3.6
# yy = 3.0
# zz = 0.8
# distance_dict = check(xx, yy, zz)

# DB_NAME = '/rssi_db.json'
# FILEPATH = os.path.dirname(os.path.abspath(__file__))
# DB_PATH = FILEPATH + DB_NAME
# rssi_db = dict()
# if os.path.isfile(DB_PATH):
#     with open(DB_PATH) as infile:
#         rssi_db = json.load(infile)
#     infile.close()

# for i in range(10):
# 	rssi_db = get_rssi(rssi_db, distance_dict)
# with open(DB_PATH, 'w') as outfile:
#     json.dump(rssi_db, outfile)
# outfile.close()

# for cnt in range(4):
# 	group = (dis[cnt], rssiRange(dis[cnt]), test[cnt])
# 	print(group)
# print("----------------------------------------")

# avgA = rssiRange(avgA)
# avgB = rssiRange(avgB)
# avgC = rssiRange(avgC)
# avgD = rssiRange(avgD)

# #location
# meas = []
# dis = []

# #dis = [10.473614466839994, 4.69507188443372, 4.865685974248646, 12.57720159653967]
# #testA = [0.61, 7.04, 2.9, dis[0]]
# #testB = [13.29, 1.05, 2.9, dis[1]]
# #testC = [0.82, -0.15, 0, dis[2]]
# #testD = [14.24, 7.78, 0, dis[3]]

# testA = [0.61, 7.04, 2.9, avgA]
# testB = [13.29, 1.05, 2.9, avgB]
# testC = [0.82, -0.15, 0, avgC]
# testD = [14.24, 7.78, 0, avgD]

# meas.append(testA)
# meas.append(testB)
# meas.append(testC)
# meas.append(testD)

# loc = triposition(meas)
# print(loc)
