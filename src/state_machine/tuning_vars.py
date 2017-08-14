
import numpy as np

class TUNING_VARS:
	dank_pot_KX = 0.0005 #0.0005
	dank_pot_KY = 0 #1.0/10000 kinda?
	dank_pot_SPEED_CAP = 2.75 #3.5 #3
	dank_pot_TURNING_SLOW_FACTOR = 5 #6  #8
	#dank_pot_TURNING_SLOW_FACTOR_EXPERIMENTAL = 28
	#dank_pot_KX = 0.0005 #experimental, delete if doesnt work
        
        overpass_KX = 0.0001
        overpass_speed_boost = 2

	#find_yellow_line_THRESH_YELLOW = [np.array([18, 216, 128]), np.array([103,255,255])]
	find_yellow_line_THRESH_YELLOW = [np.array([10, 170, 128]), np.array([103,255,255])]
	find_yellow_line_THRESH_ORANGE = [np.array([132, 106, 167]), np.array([38, 255, 255])]
	find_yellow_line_COUTOUR_AREA = 500
	find_yellow_line_CUT_SIZE = 2.0/3 #2.0/3

	line_follow_KP = 0.0018
 	line_follow_KD = 0.01
	line_follow_SPEED = 2.3
	line_follow_TURNING_SLOW_FACTOR = 2 #2.5 #3 #4

	mesh_control_EXCLUDE_RANGE = 180
	mesh_control_SEARCH_RANGE = 360

	wall_controller_D_DES = .7
	wall_controller_RIGHT_OFFSET = .2
	wall_controller_SPEED_DES = 3.5 #3
	wall_controller_KP = 0.07 #0.15 #0.3
	wall_controller_SCAN_LOW_RIGHT = 172
	wall_controller_SCAN_HIGH_RIGHT = 300
	wall_controller_SCAN_LOW_LEFT = 780
	wall_controller_SCAN_HIGH_LEFT = 908
