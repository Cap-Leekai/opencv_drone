#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import numpy as np
import rospy
import tf
import math
import dynamic_reconfigure.client
from opencv_drone.msg import DroneTrajectory

from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,LaserScan
from opencv_drone.msg import frame_detect
from std_msgs.msg import Float32
from drone_msgs.msg import Goal
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from drone_msgs.msg import DroneTrajectory

depth_image_topic = "/r200/depth/image_raw"
image_topic = "/r200/image_raw"
frame_detect_topic = "/frame_detector"
alt_topic = "/drone/alt"                                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"
drone_goal_pose = "/goal_pose"
lidar_topic='laser/scan'

marker_topic = "/marker_target_point"


marker_publisher = None
depth_frame = None
image_binary = None
rgb_image = None
corners = None

detect_frame=False
detect_line=False
detect_line_count=0

frame_detect_flag = frame_detect()
goal_pose = Goal()
drone_pose = PoseStamped()
drone_alt = Float32()
flag = True
window_detect_flag = False
use_unstable = False

# функция считывания текущего положения дрона
def drone_pose_cb(data):
    global drone_pose, quaternion, roll, pitch, yaw
    drone_pose = data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

def frame_detect_cb(data):
    global frame_detect_flag,detect_line_count
    frame_detect_flag.detect_frame=data.detect_frame
    frame_detect_flag.detect_line=data.detect_line
    if frame_detect_flag.detect_line==True:
        detect_line_count=detect_line_count+1
    else:
        detect_line_count=0

def scan_cb(msg):
    global min_dist_index,point_near,point_cloud
    point_cloud=msg.ranges
    #point_cloud_s=msg.ranges[90:-90]

    #print("PCL",len(point_cloud))
    #min_dist = min(point_cloud_s)
    min_dist = min(point_cloud)
    #min_dist_index_s = msg.ranges.index(min_dist)
    min_dist_index = msg.ranges.index(min_dist)
    #min_dist_index = min_dist_index_s+90
    point_near = [min_dist*math.cos(math.radians(min_dist_index-180)),min_dist*math.sin(math.radians(min_dist_index-180))]
    #point_near = [min_dist*math.cos(math.radians(min_dist_index-90)),min_dist*math.sin(math.radians(min_dist_index-90))]

    #print("point_near",point_near)


def callback(config):
    rospy.loginfo("Config set to {run}".format(**config))


# функция считывания текущей высоты
def drone_alt_cb(data):
    global drone_alt
    drone_alt = data.data


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    return X, Y


# callback считывания картинки с realsence в rgb
def rgb_image_cb(data):
    global rgb_image
    try:
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except:
        print "Error read rgb_image"
        rgb_image = None


# callback считывания карты глубины с realsence
def depth_image_cb(data):
    global image_binary, depth_frame
    try:
        bridge = CvBridge()

        # переводим фрейм из росовского сообщения в картинку opencv
        depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_frame = np.array(depth_image, dtype=np.float32)   # каждый элемент фрейма хранит значение типа float являющееся расстоянием в метрах до точки

        image_binary = np.zeros_like(depth_frame)
        # делаем маску из допустимых пикселей на основе условия
        image_binary[(depth_frame < 1.5) & (depth_frame > 0.5)] = 255

        image_binary = np.uint8(image_binary)

    except:
        print "Error read depth image"
        depth_image = None
        image_binary = None


# функция нахождения столба
def detector_of_pillar():
    global flag, rgb_image_copy, cv_image,image_binary,scan
    global min_dist_index, point_near, point_cloud,drone_pose,yaw,frame_detect_flag


    try:

        rgb_image_copy = rgb_image.copy()

        if rgb_image_copy is not None and 255 in image_binary:

            rgb_integrate = cv.bitwise_and(rgb_image, rgb_image, mask=image_binary)
            cv.imshow("BLET",rgb_integrate)
            half_bin = image_binary[: image_binary.shape[0] // 2, :]
            half_depth = depth_frame[: depth_frame.shape[0] // 2, :]

            # находим контуры
            contours, hierarchy = cv.findContours(half_bin, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

            # сортируем контуры
            contours = sorted(contours, key=cv.contourArea, reverse=True)

            if len(contours):

                cv.drawContours(rgb_integrate, contours[0], -1, (0, 255, 0), 5)

                cords_of_rect = cv.boundingRect(contours[0])
                cords_of_center = (
                (cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2))

                cv.rectangle(rgb_integrate, (cords_of_rect[0], cords_of_rect[1]),
                             (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]),
                             (255, 0, 0), 1)
                cv.circle(rgb_integrate,
                          ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)),
                          10, (0, 255, 0), -10)

                Q = -(cords_of_center[0] - (image_binary.shape[1] // 2)) * (1.3962 / image_binary.shape[1])  # 1.3962 - угол обзора камеры в радианах

                # print cords_of_center, Q
                dist_to_pillar = half_depth[cords_of_center[1]][cords_of_center[0]]

                x = math.cos(Q) * (dist_to_pillar + 2.0)
                y = math.sin(Q) * (dist_to_pillar + 1.5)
                print('pub marker')

                cv.imshow("pillar", rgb_integrate)
                # print cv.contourArea(contours[0]), dist_to_pillar

                ang_obs=math.fabs(math.atan2(point_near[1], point_near[0]))
                print("ANG OBS",ang_obs)
                print("YAW",yaw)
                #if cv.contourArea(contours[0]) > 10000. and dist_to_pillar < 2.5 and not frame_detect_topic.detect_window and ang_obs<math.pi/2:
                edge_rp = [0.0, 0.0]
                edge_lp = [0.0, 0.0]
                edge_dist=0.0
                edge_lp, edge_rp = take_edges_from_lidar(point_cloud, min_dist_index)
                edge_dist = math.sqrt((edge_lp[0] - edge_rp[0]) ** 2 + (edge_lp[1] - edge_rp[1]) ** 2)
                print("EDGE DIST",edge_dist)
                print(edge_lp,edge_rp)
                if cv.contourArea(contours[0]) > 10000. and dist_to_pillar < 2.5 and ang_obs < math.pi / 2 and 0.4<edge_dist and edge_dist<1.2:

                    print "detect pillar"
                    frame_detect_flag.detect_frame = True
                    detect_frame_publisher.publish(frame_detect_flag)

                    use_unstable = True
                    use_potential = False
                    client.update_configuration({"run": use_unstable})
                    client_pot.update_configuration({"run": use_potential})


                    # if flag:
                    goal_pose.pose.point.x, goal_pose.pose.point.y = transform_cord(yaw, (x, y, 0.0))  # + Q, (dist_to_pillar + 1.5, 0.0, 0.0)
                    goal_pose.pose.point.z = drone_alt
                    goal_pose.pose.course = yaw + Q
                    find_pillar_center(min_dist_index, point_cloud, drone_pose, yaw)

                else:
                    pass
                    # frame_detect_flag.detect_frame = False
                    # detect_frame_publisher.publish(frame_detect_flag)
    except:
        #find_pillar_center(min_dist_index, point_cloud, drone_pose, yaw)
        print "Pillar detector --> ERROR"

def make_marker(point):
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z

    return marker

def test_marker(point):
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z

    return marker

def find_pillar_center(index,scan,cur_pose,course):
    #Find pillar_center
    center_pillar = np.array([[0.0], [0.0]])
    cur_pose=np.array([[cur_pose.pose.position.x],[cur_pose.pose.position.y]])
    edge_rp=[0.0,0.0]
    edge_lp=[0.0,0.0]
    i=0
    print("RAR")
    #Check distance difference and define edges of pillar(near object)
    edge_lp,edge_rp=take_edges_from_lidar(scan,index)
    edge_dist=math.sqrt((edge_lp[0]+edge_rp[0])**2+(edge_lp[1]+edge_rp[1])**2)
    #Calculate pillar center by distance derrivative
    center_pillar[0]=float((edge_lp[0]+edge_rp[0])/2.0)
    center_pillar[1]=float((edge_lp[1]+edge_rp[1])/2.0)
    print("PILLAR CENTER LOC : ",center_pillar)

    find_trajectory_for_pillar_avoidance(center_pillar,drone_pose,course,1.0,0.8,20)

def take_edges_from_lidar(scan,index):
    #Function finds edges from lidar laser scan to calculate derrivatives between each point
    i=0
    edge_rp = [0.0, 0.0]
    edge_lp = [0.0, 0.0]
    for i in range(len(scan)):
        index_arr=[index+i,index-i,index+i+1,index-i-1]
        for j in range(len(index_arr)):
            if index_arr[j]>359:
                index_arr[j]=index_arr[j]-359
            if index_arr[j]<0:
                index_arr[j]=359+index_arr[j]
        if edge_rp==None or edge_rp==[0.0,0.0]:
            if (math.fabs(scan[index_arr[0]]-scan[index_arr[2]]))>0.1:
                edge_rp=[scan[index_arr[0]]*math.cos(math.radians(index_arr[0]-180)),scan[index_arr[0]]*math.sin(math.radians(index_arr[0]-180))]
        if edge_lp==None or edge_lp==[0.0,0.0]:
            if (math.fabs(scan[index_arr[1]]-scan[index_arr[3]]))>0.1:
                edge_lp=[scan[index_arr[1]]*math.cos(math.radians(index_arr[1]-180)),scan[index_arr[1]]*math.sin(math.radians(index_arr[1]-180))]
    return [edge_lp,edge_rp]

def find_trajectory_for_pillar_avoidance(center_pillar,cur_pose,course,safe_dist,pillar_diameter,step_curv):
    global pub_marker
    circle_coords=list()
    angle=list()
    cur_pose=np.array([[cur_pose.pose.position.x],[cur_pose.pose.position.y]])
    for i in range(step_curv):
        course_s=0
        M = np.array([[math.cos(course_s), -math.sin(course_s)],
                      [math.sin(course_s), math.cos(course_s)]])
        circle_x=(pillar_diameter*0.5+safe_dist)*math.cos(math.pi*i/(step_curv))
        circle_y=(pillar_diameter*0.5+safe_dist)*math.sin(math.pi*i/(step_curv))
        circle_coords.append([circle_x,circle_y])
        angle.append(-math.atan2(circle_coords[i][0],circle_coords[i][1]))
    print("calculated coords local",circle_coords[0][0])
    course=course

    i=0
    #print("COORD",circle_coords)
    for i in range(len(circle_coords)):
        M = np.array([[math.cos(course), -math.sin(course)],
                      [math.sin(course), math.cos(course)]])
        course_s = -math.pi/2
        Ma = np.array([[math.cos(angle[i]+course_s), -math.sin(angle[i]+course_s)],
                       [math.sin(angle[i]+course_s), math.cos(angle[i]+course_s)]])
        circle_point=np.array([[circle_coords[i][0]],[circle_coords[i][1]]])
        Ma=center_pillar+np.dot(Ma,circle_point)
        Mg=cur_pose+(np.dot(M,Ma))
        circle_coords[i]=Mg
    #print("new_circle_coords",circle_coords)
    xg=0.0
    yg=0.0
    for i in range(len(circle_coords)):
        xg=xg+circle_coords[i][0]/len(circle_coords)
        yg=yg+circle_coords[i][1]/len(circle_coords)
    print("G",xg,yg)
    print("CUR POSE",cur_pose)
    point = Point(x=xg, y=yg, z=1)
    marker = setup_marker(point)
    pub_marker.publish(marker)
    #print("CIRCLE REVERSE:",circle_coords)
    trajectory_publisher(circle_coords,1.56)

def trajectory_publisher(trajectory,height):
    global goal_pose_pub, drone_pose, detect_frame_publisher, frame_detect_flag,point_near,detect_line_count
    #flag_corrector_course = False
    if math.atan2(point_near[1],point_near[0])-yaw>0:
        trajectory=list(reversed(trajectory))
    start_point=drone_pose
    dist=0.0
    goal_pose.pose.course = yaw
    while True:
        dist=math.sqrt((start_point.pose.position.x-drone_pose.pose.position.x)**2+(start_point.pose.position.y-drone_pose.pose.position.y)**2)
        if len(trajectory) != 0:
            print("DIST",dist)
            print("DETECT LINE",frame_detect_flag.detect_line)

            x = trajectory[0][0]
            y = trajectory[0][1]
            z = height #trajectory[0][2]

            del trajectory[0]

            goal_pose.pose.point.x = x
            goal_pose.pose.point.y = y
            goal_pose.pose.point.z = z
           # ang_obs = yaw+(math.atan2(point_near[1], point_near[0]))#-((math.atan2(point_near[1], point_near[0]))/math.fabs((math.atan2(point_near[1], point_near[0]))))*math.pi/2
           # print("ang_obs",ang_obs)
           # goal_pose.pose.course = ang_obs

            # print z

            # if flag_corrector_course is not True:
            #     goal_pose.pose.course = yaw + yaw_error
            #     flag_corrector_course = True
            # else:
            #     goal_pose.pose.course = yaw

            while True:
                #print ("BRUH")
                dist_2=math.sqrt((goal_pose.pose.point.x-drone_pose.pose.position.x)**2+(goal_pose.pose.point.y-drone_pose.pose.position.y)**2)
                if dist_2>0.5 and dist_2<10: #and detect_line==True and dist<1) or (detect_line==False and dist>1):
                    goal_pose_pub.publish(goal_pose)
                    # frame_detect_flag.detect_pillar = True
                    # detect_frame_publisher.publish(frame_detect_flag)

                    #print(goal_pose)
                elif (dist>2.5 and frame_detect_flag.detect_line==True and detect_line_count>15):
                    trajectory=list()
                    dist=0
                    use_unstable = False
                    frame_detect_flag.detect_frame = False
                    client.update_configuration({"run": use_unstable})
                    break
                else:
                    break
        else:
            frame_detect_flag.detect_frame = False
            #frame_detect_flag.detect_line = True
            detect_frame_publisher.publish(frame_detect_flag)
            print("THATS ALL FOLKS")
            break

def setup_marker(pose):
    """
    Настройка маркера для отображения в rviz
    :type pose: Point
    :type free_flag: Bool
    :param pose:
    :param free_flag:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.id = 1
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.type = Marker.SPHERE
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.8
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = pose.z

    return marker



min_dist_index=0
point_cloud=LaserScan().ranges
yaw=0.0





def main():
    global depth_frame, image_binary, rgb_image, detect_frame_publisher, rgb_integrate, goal_pose_pub, marker_publisher, client,client_pot,pub_marker

    rospy.init_node("Pillar_detector_node")

    hz = rospy.Rate(10)

    # init subscribers
    rospy.Subscriber(depth_image_topic, Image, depth_image_cb)
    rospy.Subscriber(image_topic, Image, rgb_image_cb)
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)
    rospy.Subscriber(alt_topic, Float32, drone_alt_cb)
    rospy.Subscriber(lidar_topic, LaserScan, scan_cb)
    rospy.Subscriber(frame_detect_topic, frame_detect, frame_detect_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size=10)
    detect_frame_publisher = rospy.Publisher(frame_detect_topic, frame_detect, queue_size=10)
    marker_publisher = rospy.Publisher('window_target_marker', Marker)
    pub_marker = rospy.Publisher(marker_topic, Marker, queue_size=10)


    # init client dyhamic reconfigure
    client = dynamic_reconfigure.client.Client("unstable_planner_node", timeout=1, config_callback=callback)
    client_pot = dynamic_reconfigure.client.Client("pot_planner_node", timeout=1, config_callback=callback)


    while not rospy.is_shutdown():

        if depth_frame is not None and rgb_image is not None:
            pass
            detector_of_pillar()

            hz.sleep()
        else:
            print "Images not read"

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

# начало исполнения кода
if __name__ == "__main__":
    main()
