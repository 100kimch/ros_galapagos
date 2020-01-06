#! /usr/bin/env python
#! Deprecated

import rospy
import numpy as np
import copy
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan

default_ladar_degree = 4
rate = rospy.Rate(10)

class raw:
    

######################################## functions ########################################
def get_average(_data):
	''' It makes several value's average
        * Input
            _data : several data values
        * Output
            average : avergae of data values
    '''
    average = sum(data)/(len(data)-data.count(0)+0.01) 
    return average


############################ main function of detecting blocking bar ############################
def detecting_tunnel_info(_ladar_data):
    ''' detecting blocking bar function main function of this py file
        * Input
            _ladar_data : ladar data from LaserScan directory that is published at ladar module
        * Output
            tunnel_infos : make tunnel informations by using ladar datas
    '''
    SensorData = data.ranges       # Get the data from Lidar

    # Seperate the data depend of direction of turtlebot
    front = SensorData[1:18]+SensorData[342:359]
	real_front = SensorData[1:5] + SensorData[354:359]
	front_left = SensorData[18:54]                   
	front_right = SensorData[306:342]

    left = SensorData[54:90]
	right = SensorData[270:306]	
	left_side = SensorData[85:95]
	right_side = SensorData[265:275]
	left_beside = SensorData[60:70]
	right_beside = SensorData[290:300]

	back = SensorData[135:225]
    
    # Each data has least 10 values including error values like coming up zero value
    # so if error occured, except error values and make the average of their data
	front_avg = avg(front)
	real_front_avg = avg(real_front) 
    front_left_avg = avg(front_left)
	front_right_avg = avg(front_right)

    left_avg = avg(left)				
	right_avg = avg(Right)							
	left_beside_avg = avg(left_beside)		
	right_beside_avg = avg(right_beside)
	left_side_avg = avg(left_side)
	right_side_avg = avg(right_side)
    
    back_avg = avg(back)
	
########################################################################################################################################
####  Save the process values in messege form  #########################################################################################
########################################################################################################################################

	raw.data = [Front_avg, Left_avg, Front_Left_avg, Front_Right_avg, Right_avg, Left_Beside_avg, Right_Beside_avg, Back_avg, real_Front_avg, Left_side_avg, Right_side_avg]	
	raw.sharp = [SensorData[40], SensorData[29], SensorData[20], SensorData[9], SensorData[350], SensorData[339], SensorData[329], SensorData[320]]	
	raw.front = [avg(SensorData[37:40]), avg(SensorData[33:35]), avg(SensorData[26:29]), avg(SensorData[23:25]), avg(SensorData[16:18]), avg(SensorData[13:15]), avg(SensorData[344:346]), avg(SensorData[338:340]), avg(SensorData[332:335]), avg(SensorData[330:334]), avg(SensorData[329:331]), avg(SensorData[321:324]) ]
	
########################################################################################################################################

	data2 = [Front_Left_avg, Front_avg , Front_Right_avg]
	copy_data = copy.copy(data2)           # data2 values are relocated by 'data2.sort()' So they have to be copied. 
	data2.sort(reverse=True)	       # Relocation values to large values to small values. 
	find_same(data2[0])		       # Give the large value to Function. Large value means far distance from turtlebot

	rospy.loginfo(raw.front)
	pub.publish(raw)

    



#! code for test start
def ladar_test(ladar_data):                                 
    print 'value at 0 degree'
    print ladar_data.range[0*default_ladar_degree]
    print 'value at 90 degree'
    print ladar_data.range[90*default_ladar_degree]
    print 'value at 180 degree'
    print ladar_data.range[180*default_ladar_degree]
#! code for test end

rospy.init_node('ladar_sensor', anonymous=True)
def scan_ladar():
	#rospy.Subscriber('/scan', LaserScan, detecting_blocking_bar)   
	rospy.Subscriber('/scan', LaserScan, ladar_test)                #! code for test   
	rospy.spin()			  
    
scan_ladar()                                                        #! code for test

if __name__ == '__main__':
	try:	
		scan_sensor()
	except rospy.ROSInterruptException:
		pass
