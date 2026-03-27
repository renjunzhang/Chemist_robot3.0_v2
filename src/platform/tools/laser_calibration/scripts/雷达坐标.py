#!/usr/bin/env python3
# coding=utf-8
import time

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_scipy
import cmath
from two_d_icp import GetTrans,ShowPointValue



def LaserMsgToPoint(scan_msg):
    r=scan_msg.ranges
    theta = np.arange(scan_msg.angle_min,scan_msg.angle_max,scan_msg.angle_increment)
    ls=[]
    for i in range(len(theta)):
        if np.isinf(r[i]):
            # tmp=np.array([0,0,0])
            continue
        else:
            tmp=np.array([r[i]*np.cos(theta[i]),r[i]*np.sin(theta[i]),0])

        ls.append(tmp)

    return np.array(ls)

def SetInitTrans(rz,tx,ty):
    """
    :param rz: 绕z轴的旋转，弧度制
    :param tx: x平移
    :param ty: y平移
    :return: 变换矩阵
    """
    euler=np.array([0,0,rz])#x,y,z
    T = np.array([[tx], [ty], [0]])#x,y,z
    rotation = R_scipy.from_euler('xyz', euler, degrees=False)
    rotation_matrix = rotation.as_matrix()

    Matrix_T = np.column_stack([rotation_matrix, T])
    Matrix_T = np.row_stack((Matrix_T, np.array([0, 0, 0, 1])))
    return Matrix_T

def TansToBaselink(theta,offset_x,offset_y,points):
    n,m=points.shape
    for i in range(n):
        x_=points[i][0]
        y_=points[i][1]
        points[i][0]=x_*np.cos(-theta)+y_*np.sin(-theta)+offset_x
        points[i][1]=y_*np.cos(-theta)-x_*np.sin(-theta)+offset_y



def Main():

    scan_msg_0=rospy.wait_for_message("/sick_front/scan",LaserScan)
    scan_msg_1=rospy.wait_for_message("/sick_rear/scan",LaserScan)

    scan_msg_3=rospy.wait_for_message("/scan_full_filtered",LaserScan)
    scan_3_points=LaserMsgToPoint(scan_msg_3)

    scan_0_points=LaserMsgToPoint(scan_msg_0)
    scan_1_points=LaserMsgToPoint(scan_msg_1)
    # ShowPointValue(scan_1_points)

    TansToBaselink(theta=-2.35619449012,offset_x=-0.39,offset_y=-0.317,points=scan_1_points)

    # n,m=scan_1_points.shape
    # pr=[]
    # ptheta=[]
    # for i in range(n):
    #     x=scan_1_points[i][0]
    #     y=scan_1_points[i][1]
    #     tmp_r,tmp_theta=cmath.polar(complex(x,y))
    #     pr.append(tmp_r)
    #     ptheta.append(tmp_theta)
    # plt.polar(ptheta,pr,color="red",marker=".",ms=1,ls='none')
    # plt.show()



    #把2点云转到base_link坐标系下
    init_trans=SetInitTrans(rz=0.810533,tx=0.394121,ty=0.320083)#0.78,0.33,0.175

    trnas=GetTrans(scan_0_points,scan_1_points,init_trans,th=0.025)

    print("角度=%f,x=%f,y=%f"%(np.arccos(trnas[0][0]),trnas[0][3],trnas[1][3]))




def ShowScanDataUpdate(scan_topic):
    """
    可视化实时的点云
    :param scan_topic:接收点云话题的名字
    :return: none
    """
    scan_msg_0=rospy.wait_for_message(scan_topic,LaserScan)

    datas=scan_msg_0.ranges
    theta = np.arange(scan_msg_0.angle_min,scan_msg_0.angle_max+scan_msg_0.angle_increment,scan_msg_0.angle_increment)
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    line,=ax.plot(theta,datas,".",markersize=1)
    while True:
        scan_msg_0=rospy.wait_for_message(scan_topic,LaserScan)
        datas=scan_msg_0.ranges
        theta = np.arange(scan_msg_0.angle_min,scan_msg_0.angle_max+scan_msg_0.angle_increment,scan_msg_0.angle_increment)
        line.set_xdata(theta)
        line.set_ydata(datas)
        fig.canvas.draw()

        # 控制动态更新图形的时间间隔
        plt.pause(0.01)



def GetClockAngle(v1, v2):
    # 2个向量模的乘积
    TheNorm = np.linalg.norm(v1)*np.linalg.norm(v2)
    # 叉乘
    rho =  np.rad2deg(np.arcsin(np.cross(v1, v2)/TheNorm))
    # 点乘
    theta = np.arccos(np.dot(v1,v2)/TheNorm)
    if rho < 0:
        return -theta
    else:
        return theta


def Calculate(point_base_1,point_base_2,point_scan_1,point_scan_2):
    """
    np类型的(x,y)
    :param point_base_1:
    :param point_base_2:
    :param point_scan_1:
    :param point_scan_2:
    :return:
    """
    vet_base=np.array([point_base_1[0]-point_base_2[0],point_base_1[1]-point_base_2[1]])
    vet_scan=np.array([point_scan_1[0]-point_scan_2[0],point_scan_1[1]-point_scan_2[1]])
    angle=GetClockAngle(vet_scan,vet_base)

    """
    x_=ysin$+xcos$
    y_=ycos$-xsin$
    """
    tmp_x=point_scan_1[1]*np.sin(angle)+point_scan_1[0]*np.cos(angle)
    tmp_y=point_scan_1[1]*np.cos(angle)-point_scan_1[0]*np.sin(angle)

    offset=point_base_1-np.array([tmp_x,tmp_y])

    print("偏角=%f,x偏移=%f，y偏移=%f"%(angle,tmp_x,tmp_y))
    return angle,offset

def ShowScanData(datas,angle_min,angle_max,angle_increment,color="red"):

    theta = np.arange(angle_min, angle_max+angle_increment, angle_increment)
    plt.polar(theta,datas,color=color,marker=".",ms=1,ls='none')







def Quaternion2Euler(quate):
    r = R_scipy.from_quat(quate)
    euler = r.as_euler('xyz', degrees=False)
    return euler


if __name__== "__main__":
    # 初始化节点
    rospy.init_node("lidar_node")
    time.sleep(2)

    #1观测雷达计算填入数据
    # ShowScanDataUpdate("/sick_s30b/laser/scan0")
    #
    # #2得到后雷达到base_link的转换
    # w=0
    # L=0
    # d=0
    # d1=0
    # d2=0
    # r_scan1=0
    # theta_scan_1=0
    # r_scan2=0
    # theta_scan_2=0
    #
    #
    # pb1=np.array([L/2+np.sqrt(d1**2-d**2),-(d+w/2)])
    # pb2=np.array([-(L/2+np.sqrt(d2**2-d**2)),-(d+w/2)])
    # ps1=np.array([r_scan1*np.cos(theta_scan_1),r_scan1*np.sin(theta_scan_1)])
    # ps2=np.array([r_scan2*np.cos(theta_scan_2),r_scan2*np.sin(theta_scan_2)])
    # Calculate(pb1,pb2,ps1,ps2)

    #3得到前雷达到base_link的转换
    Main()







