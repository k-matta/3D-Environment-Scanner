# Hallway Scan Companion Application
#
# Purpose:	This application is used as the PC companion app used with the microcontroller hallway scanner
#			The application collects scans, performs basic data processing, and outputs a 3D, interactive
#			visualization of the collected data. Raw data is also exported as a CSV file
#
#   Special notes:
#	   1. Open3D only works with Pythons 3.6-3.11.  It does not work with 3.12 or 3.13
#	   2. For this eample you should run it in IDLE.  Anaconda/Conda/Jupyter
#	   require different Open3D graphing methods (these methods are poorly documented)
#	   3. Under Windows 10 you may need to install the MS Visual C++ Redistributable bundle
#		   https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170
#	   4. VirtualBox does not support OpenGL. If you're running Windows under Virtualbox or
#	   your system doesn't support OpenGL (very rare), then you can install an OpenGL emulator dll
#		   https://fdossena.com/?p=mesa/index.frag (unzip and copy opengl32.dll into Python dir)
#
#   T. Doyle
#   March 18, 2022 (Updated 2020 example)
#   Template updated to application by Karl Matta on April 5th 2026

# Install dependencies
import math
import numpy as np
import open3d as o3d
import serial

# Define important constants
SENSOR_HEIGHT = 210
SHIFT = 0
SCAN_DISTANCE = 300
MEASURMEMENTS_PER_CYCLE = 64
BASE_FILE_NAME = "scan"
COM = "COM3"

# Turn a received string into a dictionary of measured values
def parse_data(str_array: str, splitter: str = "\n") -> dict[str, int|float]:
	data_array = str_array.split(splitter)
	data_dict = {}
	if (len(data_array) == 8):
		keys = ['distance', 'angle', 'signal', 'ambiant', 'spad_num', 'range_status', 'status', 'invalid']
		for datum in range(len(data_array)):
			data_dict[keys[datum]] = int(data_array[datum].strip())
		data_dict["angle"] = data_dict["angle"]/1024
	else:
		keys = ['slice', 'distance', 'angle_raw', 'angle', 'x', 'y', 'z', 'signal', 'ambiant', 'spad_num', 'range_status', 'status', 'invalid']
		for datum in range(len(keys)):
			if ("." in data_array[datum]):
				data_dict[keys[datum]] = float(data_array[datum].strip())
			else:
				data_dict[keys[datum]] = int(data_array[datum].strip())
		data_dict["angle"] = data_dict["angle_raw"]/1024
	return data_dict

# Take an array of strings and convert it into an array of dictionary values
def load_data(array):
	for line in range(len(array)):
		array[line] = parse_data(array[line], ", ")
	return array

# Convert an angle and distance into an x-y coordinate
def get_coords(point: dict):
	angle = point["angle"]*math.pi
	a = math.cos(angle)*point["distance"]
	b = math.sin(angle)*point["distance"]
	return (a, b)

# Get the distance of a point from the origin
def get_distance_from_origin(a: float, b: float):
	return math.sqrt(b**2 + a**2)

# Get the slope of a line given its angle
def get_slope(angle: float):
	a = math.cos(angle*math.pi)
	b = math.sin(angle*math.pi)
	return b/a

# Get the slope of a line given two points
def get_point_slope(point1: dict, point2: dict):
	(a1, b1) = get_coords(point1)
	(a2, b2) = get_coords(point2)

	return (b2-b1)/(a2-a1)

# Get the intersection between two lines 
def get_intersection(m1: float, point1: dict, point2: dict):
	(c2, d2) = get_coords(point2)
	m2 = get_point_slope(point1, point2)
	b2 = d2 - m2*c2 # Get y-intercept of the second line (rearranged from y = mx + b)

	if (not m1): # Line 2 will always intersect at y=0 if m1=0 since m1 passes through the origin.
		y_int = 0.0
		x_int = (y_int-b2)/m2 # Solving for the x-coordinate of the interseciton pioint from y=mx+b
	else:
		y_int = b2/(1-m2/m1) # Solving line 1 for x and then substituting into line 2 yields this result (knowing that m1 has the form y = mx + 0)
		x_int = y_int/m1 # Solving for the x-coordinate of the intersection point

	# Return the coordinates and the distance from the origin
	return(x_int, y_int + SENSOR_HEIGHT, get_distance_from_origin(x_int, y_int)) # Since the sensor is raised SENSOR_HEIGHT mm above the floor, this number is added to y

# Find invalid points and set them to the maximum displacement as an error result
def fix_jagged(point1: list, point2: list, point3: list):
	if (abs(point1[2] - point2[2]) > 400 and (abs(point2[2] - point3[2]) > 400 or abs(point1[2] - point3[2]) > 800)): # Check for unnaturally large displacement in a group of 3 points
		if (abs(point1[1] - point2[1]) > 150 or abs(point2[1] - point3[1]) > 150 or abs(point1[1] - point3[1]) > 300):
			if ((point1[2] + point2[2] + point3[2])/5 > 1000): # Set all points' z-value (left/right) to maximum if they have a high value
				point1[2] = 3500
				point2[2] = 3500
				point3[2] = 3500
			elif ((point1[2] + point2[2] + point3[2])/5 < -1000):
				point1[2] = -3500
				point2[2] = -3500
				point3[2] = -3500

# Generate a list of data from a .xyz file
def generate_from_raw(xyz: list[list], raw_data: list[dict[str, int|float]]) -> None:
	index = len(xyz)

	# Until the current xyz array has caught up to the data in the file
	while (index < len(raw_data)):
		cycle = []
		failed_measurements = []

		# Get the data, one circular scan at a time
		for item in range(MEASURMEMENTS_PER_CYCLE):
			cycle.append(raw_data[index].copy())

			# Track failed measurements for correction
			if (raw_data[index]["invalid"]):
				failed_measurements.append(item)
			index += 1

		while (len(failed_measurements)):
			measurement = failed_measurements[0]

			# Find the previous valid measurement
			prev_valid = measurement
			while prev_valid in failed_measurements:
				prev_valid = (prev_valid + MEASURMEMENTS_PER_CYCLE - 1) % MEASURMEMENTS_PER_CYCLE

			# Find the next valid measurement
			next_valid = measurement
			while (next_valid in failed_measurements):
				next_valid = (next_valid + 1) % MEASURMEMENTS_PER_CYCLE

			# Adjust index if the previous valid measurement looped through the array to the end
			if (prev_valid > next_valid):
				prev_index = prev_valid - MEASURMEMENTS_PER_CYCLE
			else:
				prev_index = prev_valid

			# Try to interpolate a line between the two valid measurements and plot the invalid measurement along that line
			for fail in range(prev_index+1, next_valid):
				slope = get_slope(cycle[fail]["angle"])
				(z, y, distance) = get_intersection(slope, cycle[prev_valid], cycle[next_valid])
				cycle[fail]["distance"] = distance
				cycle[fail]["coords"] = (z, y)
				failed_measurements.remove((fail + MEASURMEMENTS_PER_CYCLE) % MEASURMEMENTS_PER_CYCLE)

		# Clip measurements on the z-axis to a maximum distance of 3.5m
		for data in cycle:
			y = data["y"]
			z = data["z"]

			if (z < 0): z = max(-3500, z)
			else: z = min(3500, z)

			# Since the floor is 0, y cannot be negative
			xyz.append([data["x"], max(y, 0.0), z + SHIFT]) # The SHIFT property is applied in case the sensor was moved to a different location

# Track important data
failed = False
scan_data: list[list|tuple] = []
scan_data_raw = []
choice = ""

# Determine what file the user would like to use
while (choice.lower() != "n" and choice.lower() != "a" and choice.lower() != "r"):
	choice = input("Please indicate whether you would like to create a new scan (n), append to an old one (a), or read an existing scan (r).\n> ")
choice = choice.lower()

if (choice != "n"):
	print("Retrieving existing data...")

	# Try to open the previous file
	try:
		with open(BASE_FILE_NAME + "_raw.csv", "r") as r:
			raw_temp = r.readlines()
			for line in range(len(raw_temp)):
				raw_temp[line] = raw_temp[line].strip()

			# Parse the data from the previous scan
			for row in range(1, len(raw_temp)):
				scan_data_raw.append(parse_data(raw_temp[row], ", "))

	# Handle errors
	except FileNotFoundError:
		print(f"Could not find file with name '{BASE_FILE_NAME}_raw.csv'.")
		if (choice == "a"):
			print("A new file with this name will be created at the end of this program.")
		else:
			failed = True
		print("\nIf this is a mistake, please change the BASE_FILE_NAME variable at the top of this file.\n")

	if (not failed):
		try:

			# Parse the previous raw measurements
			with open(BASE_FILE_NAME + ".xyz", "r") as t:
				scan_text = t.readlines()
				for line in range(len(scan_text)):
					scan_text[line] = scan_text[line].strip()

				for row in range(len(scan_data)):
					# Retrieve data and ensure it complies with digital and logical limitations
					new_row = []
					[x, y, z] = scan_text[row].split(" ")
					new_row.append(float(x))
					new_row.append(max(float(y), 0.0))
					z = float(z)
					if (z < 0): z = max(-3500, z)
					else: z = min(3500, z)
					new_row.append(z)
					scan_data.append(new_row)

		# Handle errors
		except FileNotFoundError:
			print(f"Could not find file with name '{BASE_FILE_NAME}.xyz'.")
			print("A new file with this name will be generated based on raw data.")
			# Generate missing data from the raw file if necessary
			generate_from_raw(scan_data, scan_data_raw) # type: ignore

		# Generate missing data from the raw file if necessary
		if (len(scan_data) < len(scan_data_raw)):
			generate_from_raw(scan_data, scan_data_raw) # type: ignore

		# Set far, jagged points to the maximum z-value to mark them as potentially invalid/unreliable
		for row in range(len(scan_data)):
			if (row > 1):
				if (scan_data[row][0] == scan_data[row-1][0] and scan_data[row-1][0] == scan_data[row-2][0]):
					fix_jagged(scan_data[row-2], scan_data[row-1], scan_data[row]) # type: ignore

		# Convert to tuple since the data shouldn't be changed
		for row in range(len(scan_data)):
			scan_data[row] = tuple(scan_data[row])

s = serial.Serial()

# Prepare for data collection from the device over UART
if (choice != "r" and not failed):
	try:
		s.port = COM
		s.baudrate = 115200
		s.timeout = 2
		print("Opening: " + s.name) # type: ignore
		s.open()
	except Exception as e:
		print(f"Error opening port {COM}.\nError: {e}")
		failed = True

# Parse the received data
if (not failed):
	x = 0
	if (choice == "a"):
		while True:
			# Allow the user to insert a scan at any location (in case one was skipped or erased in the middle)
			try:
				x = input("Please indicate the slice number you would like to insert:\n> ")
				x = int(x)*SCAN_DISTANCE
				break
			except TypeError:
				print("Invalid input. Please enter a number using digits.\n")

	if (choice != "r"):

		# reset the buffers of the UART port to delete the remaining data in the buffers
		s.reset_output_buffer()
		s.reset_input_buffer()

		# Waiting for transmission from the MCU
		print("Starting data collection...")

		try:
			done = False
			while not done:
				scanned = 0
				current_scan = []
				failed_measurements = []

				while(scanned < MEASURMEMENTS_PER_CYCLE):
					data = ""
					# Keep checking for data until a full set is received or the terminal signal is read
					while (len(data.split("\n")) < 8 and data != 'T'):
						raw_data = s.read_until(b"\n\r\n\r").strip() # Read until the end sequence
						data += raw_data.decode("ascii", "ignore")
					s.reset_input_buffer()
					print("Decoded Data:", data)
					if (data == 'T'):
						done = True
						break
					else:
						# Extract important info about the received data
						data = parse_data(data)
						data['angle_raw'] = data['angle']*1024
						data['slice'] = x//SCAN_DISTANCE
						data['x'] = x
						(z,y) = get_coords(data)
						data['y'] = y + SENSOR_HEIGHT
						data['z'] = z + SHIFT
						scan_data_raw.append(data)
						data = data.copy()
						current_scan.append(data)

						# Keep track of invalid scans
						if (data["invalid"]): failed_measurements.append(scanned)
						scanned += 1
						print()

				# Fix failed measurements before starting the next set of scans
				while (len(failed_measurements)):
					measurement = failed_measurements[0]

					# Find previous valid measurement
					prev_valid = measurement
					while prev_valid in failed_measurements:
						prev_valid = (prev_valid + MEASURMEMENTS_PER_CYCLE - 1) % MEASURMEMENTS_PER_CYCLE

					# Find next valid measurement
					next_valid = measurement
					while (next_valid in failed_measurements):
						next_valid = (next_valid + 1) % MEASURMEMENTS_PER_CYCLE

					# Account for index wraparound
					if (prev_valid > next_valid):
						prev_index = prev_valid - MEASURMEMENTS_PER_CYCLE
					else:
						prev_index = prev_valid

					# Try to plot the invalid points along a linen between the known valid points
					for fail in range(prev_index+1, next_valid):
						slope = get_slope(current_scan[fail]["angle"])
						(z, y, distance) = get_intersection(slope, current_scan[prev_valid], current_scan[next_valid])
						current_scan[fail]["distance"] = distance
						current_scan[fail]["coords"] = (z, y)
						if (prev_index != prev_valid):
							failed_measurements.remove((fail + MEASURMEMENTS_PER_CYCLE) % MEASURMEMENTS_PER_CYCLE)
						else:
							failed_measurements.remove(fail)

				# Extract finalized coordinates from data
				for data in current_scan:
					if ("coords" in data.keys()):
						z = data["coords"][0]
						y = data["coords"][1]
					else:
						y = math.sin(math.pi*data["angle"])*data["distance"] + SENSOR_HEIGHT
						z = math.cos(math.pi*data["angle"])*data["distance"]

					if (z < 0): z = max(-3500, z)
					else: z = min(3500, z)

					scan_data.append([x, max(y, 0.0), z + SHIFT])

				# Reset for the next set of measurements
				print("Finished current slice.\nRestarting data collection procedure...\n")
				x += SCAN_DISTANCE

		# Handle errors
		except KeyboardInterrupt:
			failed = True
			print("Beginning Emergency Shutdown...")
		except Exception as e:
			failed = True
			print(f"Error has triggered a shutdown...\nError: {e}")

		# Fix jagged points
		for row in range(len(scan_data)):
			if (row > 1):
				if (scan_data[row][0] == scan_data[row-1][0] and scan_data[row-1][0] == scan_data[row-2][0]):
					fix_jagged(scan_data[row-2], scan_data[row-1], scan_data[row]) # type: ignore

if (failed):
	if (choice != "r"):
		print("Scanning Failed!")
else:
	if (choice == "a"):
		# Sort the data so that the slices are in the correct order
		scan_data.sort(key=lambda x: x[0])
		scan_data_raw.sort(key=lambda x: x['slice'])

	# Write the coordinates to an XYZ file for future viewing
	f = open(BASE_FILE_NAME + ".xyz", "w")
	for (x, y, z) in scan_data:
		f.write(f"{x} {y} {z}\n")
	f.close()

	# Write the full suite of collected data to a CSV file for future analysis
	with open(BASE_FILE_NAME + "_raw.csv", "w") as r:
		r.write("Slice, Distance, Raw Angle, Angle, X, Y, Z, Signal, Ambiant, SPAD Number, RangeStatus, Function Status, Invalid\n")
		for item in scan_data_raw:
			r.write(f"{item['slice']}, {item['distance']}, {item['angle_raw']}, {item['angle']}, {item['x']}, {item['y']}, {item['z']}, {item['signal']}, {item['ambiant']}, {item['spad_num']}, {item['range_status']}, {item['status']}, {item['invalid']}\n")
	print("Scanning Complete!")

	# Generate a point point cloud from the XYZ file's generated data
	pcd = o3d.io.read_point_cloud(BASE_FILE_NAME + ".xyz", format="xyz")

	# Visualize point cloud data
	o3d.visualization.draw_geometries([pcd]) # type: ignore

	# Give each vertex a unique number
	yz_slice_vertex = []
	for vertex in range(len(scan_data)):
		yz_slice_vertex.append([vertex])

	# Define coordinates to connect lines in each yz slice
	lines = []  
	for vertex in range(0, len(scan_data), MEASURMEMENTS_PER_CYCLE):
		for i in range(MEASURMEMENTS_PER_CYCLE):
			lines.append([yz_slice_vertex[vertex+i], yz_slice_vertex[vertex+(i+1)%MEASURMEMENTS_PER_CYCLE]])

	# Define coordinates to connect lines between current and next yz slice
	for vertex in range(0, len(scan_data)-MEASURMEMENTS_PER_CYCLE-1, MEASURMEMENTS_PER_CYCLE):
		for i in range(MEASURMEMENTS_PER_CYCLE):
			lines.append([yz_slice_vertex[vertex+i], yz_slice_vertex[vertex+i+MEASURMEMENTS_PER_CYCLE]])

	# This line maps the lines to the 3d coordinate vertices
	line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

	# Visualize wireframe model
	o3d.visualization.draw_geometries([line_set]) # type: ignore


if (choice != "r"):
	#close the port
	print("Closing: " + s.name) # type: ignore
	s.close()

print("Program shutdown successfully.")