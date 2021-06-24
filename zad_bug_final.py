#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan

import math
import argparse

angle_precision = math.pi / 90 # dozwolony błąd kąta(2 stopni)
distance_precision = 0.1 # dozwolony błąd pozycji
min_distance_to_wall = 1 # dystans, który utrzymuje robot, śledząc ścianę
pose = Pose()
pub = None

"""
Go to point states:
0 - fix yaw
1 - go straight
2 - point achieved
"""
go_to_point_state = 0 
go_to_point_activator = None # zmienna, aktywująca tryb bezpośredniego przemieszczenia się do zadanego punktu if True


"""
Follow wall states:
0 - find wall
1 - turn left
2 - move along the wall
"""
wall_follower_state = 0
follow_wall_activator = None # zmienna, aktywująca tryb śledzenia ściany i omijania jej CW if True

"""
Follow wall 2 states:
0 - find wall
1 - turn right
2 - move along the wall
"""
wall_follower_state_2 = 0
follow_wall_activator_2 = None # zmienna, aktywująca tryb śledzenia ściany i omijania jej CCW if True

bug1_state = 0
bug1_activator = None # zmienna, aktywująca algorytm bug1 if True
bug1_state_desc = ["Go to point", "Circumnavigate the obstacle", "Go to closest point"]
bug1_count_state_time = 0 # zmienna, kontrolująca czas trwania stanu
bug1_count_loop = 0
bug1_circ_closest_pt = Pose() 
bug1_circ_starting_pt = Pose()

alg1_state = 0
alg1_activator = None # if True - zmienna, aktywująca algorytm alg1
alg1_state_desc = ["Go to point", "Circumnavigate the obstacle CW", "Circumnavigate the obstacle CCW"]
alg1_count_state_time = 0
alg1_count_loop = 0
alg1_hit_point1 = Pose()
alg1_hit_point2 = Pose()

rev1_state = 0
rev1_activator = None # if True - zmienna, aktywująca algorytm rev1
rev1_state_desc = ["Go to point", "Circumnavigate the obstacle CW", "Circumnavigate the obstacle CCW"]
rev1_count_state_time = 0
rev1_count_loop = 0
rev1_hit_point = Pose()
rev1_hit_point_library = {} # słownik, key = punkt spotkania przeszkody, value = kierunek zwrotu(0 or 1)
rev1_turn_decider = 0 # zmienna, decydująca w którą stronę skręca robot(if % 2 == 0 - CCW)


initial_pose = Pose() # początkowa pozycja robota
initial_pose.x = rospy.get_param('x_pos')
initial_pose.y = rospy.get_param('y_pos')

algorithm = ""

# obszary skanowania
regions_ = {
	'right': 0,
	'fright': 0,
	'front': 0,
	'fleft': 0,
	'left': 0,
}

# funkcja, wczytująca parametry z wiersza poleceń
def get_data():
	parser = argparse.ArgumentParser()
	parser.add_argument("-x" , type = float)
	parser.add_argument("-y", type = float)
	parser.add_argument("-a" , type = str)
	args = parser.parse_args()
	desired_point = Pose()
	desired_point.x = args.x
	desired_point.y = args.y
	algorithm = args.a
	return desired_point, algorithm

# funkcja, aktywująca żądany algorytm
def set_algorithm(desired_algorithm):
	global bug1_activator, alg1_activator, rev1_activator
	if desired_algorithm == "bug1":
		bug1_activator = True
		alg1_activator = False
		rev1_activator = False
	elif desired_algorithm == "alg1":
		bug1_activator = False
		alg1_activator = True
		rev1_activator = False
	elif desired_algorithm == "rev1":
		bug1_activator = False
		alg1_activator = False
		rev1_activator = True

# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	global regions_

	regions_ = {
		'left' : min(min(scan.ranges[80:109]),10),
		'fleft' : min(min(scan.ranges[26:79]),10),
		'front' : min(min(scan.ranges[335:359]),min(scan.ranges[0:25]),10),
		'fright' : min(min(scan.ranges[280:334]),10),
		'right' : min(min(scan.ranges[250:279]),10),
	}

# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global new_vel, pose
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	
# funkcja normalizująca kąt i kierunek obrotu
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# funkcja wyliczająca dystans między punktami
def distance_between_points(current_pose, desired_pose):
    distance = math.sqrt(math.pow((desired_pose.x - current_pose.x), 2) + math.pow((desired_pose.y - current_pose.y), 2))
    return distance

# funkcja wyliczająca dystans do M-linii
def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_pose, desired_point
    p1 = initial_pose
    p2 = desired_point
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq
    
    return distance

# funkcja zmieniająca stan w trybie go_to_point
def change_go_to_point_state(state):
	global go_to_point_state
	go_to_point_state = state
	print '"""GO_TO_POINT""" State has been changed to [%s]' % state


# funkcja obracająca robota w trybie go_to_point
def fix_angle(des_point):
    global pose, angle_precision, go_to_point_state, pub
    desired_angle = math.atan2(des_point.y - pose.y, des_point.x - pose.x)
    err_angle = normalize_angle(pose.theta - desired_angle)


    twist_msg = Twist()
    if math.fabs(err_angle) > angle_precision:
        twist_msg.angular.z = 0.7 if err_angle > 0 else -0.7
    
    pub.publish(twist_msg)

    if math.fabs(err_angle) <= angle_precision:
        print '"""GO_TO_POINT""" Angle error: [%s]' % err_angle
        change_go_to_point_state(1)

# funkcja, kirująca robota w kierunku prostym w trybie go_to_point
def go_straight(des_point):
    global pose, angle_precision, go_to_point_state, pub
    desired_angle = math.atan2(des_point.y - pose.y, des_point.x - pose.x)
    err_angle = normalize_angle(pose.theta - desired_angle)

    err_pos = math.sqrt(pow(des_point.y - pose.y, 2) + pow(des_point.x - pose.x, 2))
    
    if err_pos > distance_precision:
    	twist_msg = Twist()
    	twist_msg.linear.x = 0.5
    	twist_msg.angular.z = 0.2 if err_angle > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print '"""GO_TO_POINT""" Position error: [%s]' % err_pos
        change_go_to_point_state(2)
    
    if math.fabs(err_angle) > angle_precision * 10:
        print '"""GO_TO_POINT""" Angle error: [%s]' % err_angle
        change_go_to_point_state(0)


# funkcja wywoływana przy dotarciu do docelowego punktu w trybie go_to_point
def point_achieved():
	global pub, go_to_point_activator
	twist_msg = Twist()
	twist_msg.linear.x = 0.0
	twist_msg.angular.z = 0.0
	pub.publish(twist_msg)
	print(' """GO_TO_POINT""" Point achieved')
	print(pose.x)
	print(pose.y)
	go_to_point_activator = False

# funkcja dla uruchomienia trybu go_to_point
def go_to_point(des_point):
	global go_to_point_activator
	go_to_point_activator = True

# funkcja zmieniająca stan w trybie follow_wall
def change_follow_wall_state(state):
	global wall_follower_state
	wall_follower_state = state
	print'"""WALL_FOLLOWING""" State has been changed to [%s] ' %state

# funkcja zmieniająca stan w trybie follow_wall_2
def change_follow_wall_state_2(state):
	global wall_follower_state_2
	wall_follower_state_2 = state
	print'"""WALL_FOLLOWING_2""" State has been changed to [%s] ' %state

# funkcja określająca zachowanie robota w trybie follow_wall
def follow_wall_take_action():
	global regions_, pub
	regions = regions_
	d = min_distance_to_wall

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		change_follow_wall_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		change_follow_wall_state(1)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		change_follow_wall_state(2)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		change_follow_wall_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		change_follow_wall_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		change_follow_wall_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		change_follow_wall_state(1)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		change_follow_wall_state(0)
	else:
		rospy.loginfo(regions)

# funkcja określająca zachowanie robota w trybie follow_wall_2
def follow_wall_take_action_2():
	global regions_, pub
	regions = regions_
	d = min_distance_to_wall
	state_description = ""

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		change_follow_wall_state_2(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		change_follow_wall_state_2(1)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		change_follow_wall_state_2(0)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		change_follow_wall_state_2(2)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		change_follow_wall_state_2(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		change_follow_wall_state_2(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		change_follow_wall_state_2(1)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		change_follow_wall_state_2(0)
	else:
		rospy.loginfo(regions)

def turn_left():
	global pub
	msg = Twist()
	msg.angular.z = -0.3
	pub.publish(msg)

def turn_right():
	global pub
	msg = Twist()
	msg.angular.z = 0.3
	pub.publish(msg)

def find_wall():
	global pub
	msg = Twist()
	msg.linear.x = 0.1
	msg.angular.z = 0.3
	pub.publish(msg)

def find_wall_2():
	global pub
	msg = Twist()
	msg.linear.x = 0.1
	msg.angular.z = -0.3
	pub.publish(msg)

def follow_the_wall():
	global regions_, pub

	msg = Twist()
	msg.linear.x = 0.5
	pub.publish(msg)

# funkcja zmieniająca stan i dokonująca potrzebnych ustawień w trybie bug_1
def bug1_change_state(state):
	global bug1_state, bug1_state_desc
	global go_to_point_activator, follow_wall_activator
	global count_state_time
	bug1_count_state_time = 0
	bug1_state = state
	log = "state changed: %s" %bug1_state_desc[state]
	rospy.loginfo(log)
	if bug1_state == 0:
		go_to_point_activator = True
		follow_wall_activator = False
	if bug1_state == 1:
		go_to_point_activator = False
		follow_wall_activator = True
	if bug1_state == 2:
		go_to_point_activator = False
		follow_wall_activator = True

# funkcja zmieniająca stan i dokonująca potrzebnych ustawień w trybie alg_1
def alg1_change_state(state):
	global alg1_state, alg1_state_desc
	global go_to_point_activator, follow_wall_activator, follow_wall_activator_2
	global alg1_count_state_time
	alg1_count_state_time = 0
	alg1_state = state
	log = "State has been changed: %s" %alg1_state_desc[state]
	rospy.loginfo(log)
	if alg1_state == 0:
		go_to_point_activator = True
		follow_wall_activator = False
		follow_wall_activator_2 = False
	if alg1_state == 1:
		go_to_point_activator = False
		follow_wall_activator = True
		follow_wall_activator_2 = False
	if alg1_state == 2:
		go_to_point_activator = False
		follow_wall_activator = False
		follow_wall_activator_2 = True

# funkcja zmieniająca stan i dokonująca potrzebnych ustawień w trybie rev_1
def rev1_change_state(state):
	global rev1_state, rev1_state_desc
	global go_to_point_activator, follow_wall_activator, follow_wall_activator_2
	global rev1_count_state_time
	rev1_count_state_time = 0
	rev1_state = state
	log = "State has been changed: %s" %rev1_state_desc[state]
	rospy.loginfo(log)
	if rev1_state == 0:
		go_to_point_activator = True
		follow_wall_activator = False
		follow_wall_activator_2 = False
	if rev1_state == 1:
		go_to_point_activator = False
		follow_wall_activator = True
		follow_wall_activator_2 = False
	if rev1_state == 2:
		go_to_point_activator = False
		follow_wall_activator = False
		follow_wall_activator_2 = True

if __name__== "__main__":
	global new_vel, desired_point, pub, angle_precision, pose
	global go_to_point_state, go_to_point_activator
	global regions_, wall_follower_state, follow_wall_activator
	global wall_found, min_distance_to_wall
	global bug1_activator, bug1_state, bug1_state_desc
	global bug1_circ_starting_pt, bug1_circ_closest_pt
	global bug1_count_loop, bug1_count_state_time
	global wall_follower_state_2, follow_wall_activator_2
	global initial_pose, alg1_hit_point1, alg1_hit_point2
	global alg1_count_loop, alg1_count_state_time, alg1_activator
	global rev1_state, rev1_state_desc, rev1_activator
	global rev1_count_state_time, rev1_count_loop, rev1_hit_point
	global rev1_hit_point_library, rev1_turn_decider, algorithm
	desired_point, algorithm = get_data() # wczytywanie parametrów z linii poleceń
	set_algorithm(algorithm)
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	rate=rospy.Rate(20) # 20Hz
	while not rospy.is_shutdown():
		# logika działania trybu go_to_point
		if go_to_point_activator == True:
			if go_to_point_state == 0:
				fix_angle(desired_point)
			elif go_to_point_state == 1:
				go_straight(desired_point)
			elif go_to_point_state == 2:
				point_achieved()
		
		# logika działania trybu follow_wall
		elif follow_wall_activator == True:
			if wall_follower_state == 0:
				find_wall()
			elif wall_follower_state == 1:
				turn_left()
			elif wall_follower_state == 2:
				follow_the_wall()

		# logika działania trybu follow_wall_2
		elif follow_wall_activator_2 == True:
			if wall_follower_state_2 == 0:
				find_wall_2()
			elif wall_follower_state_2 == 1:
				turn_right()
			elif wall_follower_state_2 == 2:
				follow_the_wall()		

		# logika działania algorytmu bug1
		if bug1_activator == True:
			if regions_ == None:
				continue
			if bug1_state == 0:
				go_to_point(desired_point)
				if 0.15 <regions_['front'] < 1 and bug1_circ_starting_pt.x == 0 and bug1_circ_starting_pt.y == 0:
					bug1_circ_starting_pt.x = pose.x
					bug1_circ_starting_pt.y = pose.y
					bug1_circ_closest_pt.x = pose.x
					bug1_circ_closest_pt.y = pose.y
					bug1_change_state(1)
			elif bug1_state == 1:
				follow_wall_take_action()
				if distance_between_points(pose, desired_point) < distance_between_points(bug1_circ_closest_pt, desired_point):
					bug1_circ_closest_pt.x = pose.x
					bug1_circ_closest_pt.y = pose.y
				# bug1_count_state_time jest potrzebna do tego, by robot miał czas odjechać od punktu napotkania przeszkody
				if bug1_count_state_time > 7 and distance_between_points(pose, bug1_circ_starting_pt) < 0.4:
					bug1_change_state(2)
			elif bug1_state == 2:
				follow_wall_take_action()
				if distance_between_points(pose, bug1_circ_closest_pt) < 0.4:
					bug1_change_state(0)
			
			bug1_count_loop +=1
			if bug1_count_loop == 70:
				bug1_count_state_time += 1
				bug1_count_loop = 0

		# logika działania algorytmu alg1
		if alg1_activator == True:
			if regions_ == None:
				continue
			dst_to_line = distance_to_line(pose)
			if alg1_state == 0:
				go_to_point(desired_point)
				if alg1_count_state_time >= 1 and regions_['front'] < 0.9:
					if alg1_hit_point1.x == 0.0 and alg1_hit_point1.y == 0.0:
						alg1_hit_point1.x = pose.x
						alg1_hit_point1.y = pose.y
					elif alg1_hit_point2.x == 0.0 and alg1_hit_point2.x == 0.0:
						alg1_hit_point2.x = pose.x
						alg1_hit_point2.y = pose.y
					alg1_change_state(1)
			elif alg1_state == 1:
				follow_wall_take_action()
				if alg1_count_state_time > 3 and dst_to_line < 0.1 and distance_between_points(pose, desired_point) < distance_between_points(alg1_hit_point1, desired_point) and \
				   distance_between_points(pose, initial_pose) < distance_between_points(desired_point, initial_pose):
					alg1_change_state(0)
				elif alg1_count_state_time > 5 and distance_between_points(pose, alg1_hit_point1) < 0.3:
					alg1_hit_point1.x == 0.0
					alg1_hit_point1.y = 0.0
					alg1_change_state(2)
			elif alg1_state == 2:
				if alg1_count_state_time < 2:
					msg = Twist()
					msg.linear.x = 0
					msg.angular.z = 0.3
					pub.publish(msg)
				else:
					follow_wall_take_action_2()
				if alg1_count_state_time > 5 and dst_to_line < 0.1 and distance_between_points(pose, desired_point) < distance_between_points(alg1_hit_point2, desired_point) - 0.5 and \
				   distance_between_points(pose, initial_pose) < distance_between_points(desired_point, initial_pose):
					alg1_hit_point2.x == 0.0
					alg1_hit_point2.y == 0.0
					alg1_change_state(0)


			alg1_count_loop +=1
			if alg1_count_loop == 70:
				alg1_count_state_time += 1
				alg1_count_loop = 0

		# logika działania algorytmu rev1
		if rev1_activator == True:
			if regions_ == None:
				continue

			dst_to_line = distance_to_line(pose)

			if rev1_state == 0:
				go_to_point(desired_point)
				if rev1_count_state_time >= 1 and regions_['front'] < 0.7:
					rev1_hit_point.x = pose.x
					rev1_hit_point.y = pose.y
					if rev1_turn_decider % 2 == 0:
						rev1_hit_point_library[rev1_hit_point] = 0
						rev1_turn_decider +=1
						rev1_change_state(1)
					elif rev1_turn_decider % 2 == 1:
						rev1_hit_point_library[rev1_hit_point] = 1
						rev1_turn_decider +=1
						rev1_change_state(2)

			elif rev1_state == 1:
				if rev1_count_state_time < 2:
					msg = Twist()
					msg.linear.x = 0
					msg.angular.z = -0.3
					pub.publish(msg)
				else:
					follow_wall_take_action()
				if rev1_count_state_time > 5 and dst_to_line < 0.1 and distance_between_points(pose, desired_point) < distance_between_points(rev1_hit_point, desired_point) and \
				   distance_between_points(pose, initial_pose) < distance_between_points(desired_point, initial_pose):
					rev1_change_state(0)
				elif rev1_count_state_time > 5:
					current_pose = Pose()
					current_pose.x = pose.x
					current_pose.y = pose.y
					for key in rev1_hit_point_library.keys():
						if distance_between_points(key, current_pose) < 0.3:
							if rev1_hit_point_library[key] == 0:
								rev1_hit_point_library[key] = 1
								rev1_change_state(2)
							elif rev1_hit_point_library[key] == 1:
								rev1_hit_point_library[key] = 0
								rev1_count_state_time = 0

			elif rev1_state == 2:
				if rev1_count_state_time < 2:
					msg = Twist()
					msg.linear.x = 0
					msg.angular.z = 0.3
					pub.publish(msg)
				else:
					follow_wall_take_action_2()
				if rev1_count_state_time > 5 and dst_to_line < 0.1 and distance_between_points(pose, desired_point) < distance_between_points(rev1_hit_point, desired_point) and \
				   distance_between_points(pose, initial_pose) < distance_between_points(desired_point, initial_pose):
					rev1_change_state(0)
				elif rev1_count_state_time > 5:
					current_pose = Pose()
					current_pose.x = pose.x
					current_pose.y = pose.y
					for key in rev1_hit_point_library.keys():
						if distance_between_points(key, current_pose) < 0.3:
							if rev1_hit_point_library[key] == 1:
								rev1_hit_point_library[key] = 0
								rev1_change_state(1)
							elif rev1_hit_point_library[key] == 0:
								rev1_hit_point_library[key] = 1
								rev1_count_state_time = 0
			rev1_count_loop +=1
			if rev1_count_loop == 70:
				rev1_count_state_time += 1
				rev1_count_loop = 0
		rate.sleep()
	print("END")
