#!/usr/bin/python3 
import numpy as np 
import math
import ctypes
from ctypes import *
import io
import sys
import struct
import time

sys.stdout = io.TextIOWrapper(sys.stdout.buffer,encoding='utf8')

CDLL("./libhps3d64.so", mode=ctypes.RTLD_GLOBAL)
lib = cdll.LoadLibrary("./lidar.so")

##########################################编写有关Mavlink协议有关的代码#################################

# 设置MAVlink的几个字节的信息
MAV_system_id = 1
MAV_component_id = 1
packet_sequence = 0
MAV_OPTICAL_FLOW_message_id = 76
MAV_OPTICAL_FLOW_extra_crc = 152

# 编写计算校验位的函数
def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

#每个包41个字节，BbBbbbfffffffHBBBH，1+1+1+1+1+1+4*7+2+1+1+1+2=41
# Mavlink协议打包
def send_optical_flow_packet(x, y,flag,fd):
    global packet_sequence
    temp = struct.pack("<fffffffHBBB",x,y,flag,4,5,100,0,31011,0,0,0)#

    #print(len(temp))
    temp = struct.pack("<bBbbb33s",
                       33,
                       packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_OPTICAL_FLOW_message_id,
                       temp)

    #print(len(temp))
    temp = struct.pack("<B38sH",
                       0xFE,
                       temp,
                       checksum(temp, MAV_OPTICAL_FLOW_extra_crc))

    byte_len = len(temp)
    #print(byte_len)

    # print (struct.unpack("<BbBbbbfffffffHBBBH",temp))

    # print([hex(x) for x in temp])

    packet_sequence += 1
    ret = lib.uart_pack_send(temp,byte_len,fd)
    if ret != 1:
        print("uart error")
    return temp


INPUT = c_uint16 * 9600
data = INPUT()

lib.lidar_connect()
fd = lib.open_and_init_uart()


while True:
    
    start_time = time.time()

    lib.lidar_measure(data)

    m10 = []
    '''
    二值化
    '''
    #安全距离，单位毫米mm
    safety_distance = 2500
    i = 0
    for dist_value in range(9600):
        each_line=data[dist_value]
        #根据深度值是否在安全距离外，将其二值化
        if (each_line==0 or (each_line>safety_distance and each_line<65500)):#65500是饱和点，距离小于200mm；距离值0是无穷远点，绝对安全。
            each_line = 1
        else:
            each_line = 0
        m10.append(each_line)												
    Z=np.array(m10)

    #将9600个深度信息转换成60*160的矩阵
    Z = np.array(Z[0:9600])
    Z = Z.reshape(60,160)
   


    '''
    滑窗算法
    '''

    '''
    滑窗算法Step1 : 初始设置与控制参数
    '''

    #搜索区域限定
    row_min = 10
    row_max = 40

    #初始滑窗大小
    window_size = 20
    #初始检测区域边界
    row_start = row_min
    row_end = row_start + window_size
    column_start = 0
    column_end = column_start + window_size
    #生成20*20的滑窗
    window = np.ones((window_size,window_size))*1

    #控制参数
    #待检测块列步进值
    column_step = 10
    #待检测块行步进值
    row_step = 10
    #滑窗大小步进值
    window_size_step = 10

    #用于存储所有的安全滑窗边长
    safety_window_size = []
    #图像中心点
    centrl_point = []
    #最大安全滑窗下，所有能找到的此大小滑窗的中心点，safty_poit_window存储该中心点对应的滑窗边界
    safty_poit = []
    safty_poit_window = []

    '''
    滑窗算法Step2 : 在当前帧找出最优滑窗大小
    '''
    #当前检测区域
    while True:

        #当前检测块安全时为1
        find_window_success= 1 
        #当前帧检测完成是为1
        search_compeleted = 0  

        #更新待检测块
        now_window = Z[row_start:row_end,column_start:column_end]

        #检测当前的待检测块，按元素进行与运算，结果权威True则该块为安全区域，检测通过
        result = np.logical_and(window,now_window)

        #判断当前检测区域是否安全
        for wise in result.flat:#遍历二维数组的所有元素
            #当前块有一个与结果不是True，就不是安全区域
            if wise == False:
                # print("now window is not safety")
                #因为当前区域不是安全区域，所以待检测块右移column_step个像素
                column_start = column_start+column_step
                column_end = column_start + window_size
                #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                if column_end > 160:
                    column_start = 0
                    column_end = column_start + window_size
                    row_start = row_start+row_step
                    row_end = row_start + window_size
                    #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，search_compeleted置1
                    if row_end >row_max:
                        # print("Seach Compeleted")
                        search_compeleted = 1
                        break#本帧图像检测完，跳出该for循环
                #当前检测块不是安全区域，所以将find_window_success清零
                find_window_success = 0
                break#一旦确定该待检测块不是安全区域时，就跳出该for循环

        #本帧图像检测完成，跳出while循环
        if search_compeleted==1:
            break

        #如果当前检测块是安全区域，则在上述for循环中find_window_success不会被清零，就会执行下边if代码
        if find_window_success==1:
            # print("now window is safety")
            #记录本次安全区域对应的滑窗值
            safety_window_size.append(window_size)
            #原地向右下角方向将滑窗值增加window_size_step
            window_size = window_size+window_size_step
            #更新检测滑窗
            window = np.ones((window_size,window_size))*1
            #更新待检测块边界，与检测滑窗匹配
            row_end = row_start + window_size
            column_end = column_start + window_size
            #待检测块任意一边碰到边界，则认为上一个检测块大小就是最优滑窗大小，本帧图像搜索结束，跳出while循环
            if column_end > 160 or row_end > row_max:
                break

    #并输出历史安全滑窗
    # print("所有可行的安全滑窗值：")
    # print(safety_window_size)

    #如果当前帧存在大于20的安全滑窗则继续
    if len(safety_window_size) and max(safety_window_size)>=20:

        '''
        滑窗算法Step3 : 找出当前帧中所有符合最优滑窗大小的安全区域
        '''
        #已经发现了全部存在的可行滑窗大小，并存储在safety_window_size中，其中最后一个是最大的滑窗值，就是最优滑窗值，下面开始寻找整幅图中该滑窗值下的最优安全区域
        optimal_window_size = safety_window_size.pop()

        #初始检测区域边界，从左上角开始，检测区域边长固定为最优滑窗大小optimal_window_size
        row_start = row_min
        row_end = row_start + optimal_window_size
        column_start = 0
        column_end = column_start + optimal_window_size

        #生成最优大小的滑窗
        window = np.ones((optimal_window_size,optimal_window_size))*1

        #计数
        while True:

            #当前检测块安全时为1
            find_window_success= 1 
            #当前帧检测完成是为1
            search_compeleted = 0  

            #更新待检测块
            now_window = Z[row_start:row_end,column_start:column_end]

            #检测当前的待检测块，按元素进行与运算，结果权威True则该块为安全区域，检测通过
            result = np.logical_and(window,now_window)

            #判断当前检测区域是否安全
            for wise in result.flat:#遍历二维数组的所有元素
                #当前块有一个与结果不是True，就不是安全区域
                if wise == False:
                    # print("now window is not safety")
                    #因为当前区域不是安全区域，所以待检测块右移column_step个像素
                    column_start = column_start+column_step
                    column_end = column_start + optimal_window_size
                    #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                    if column_end > 160:
                        column_start = 0
                        column_end = column_start + optimal_window_size
                        row_start = row_start+row_step
                        row_end = row_start + optimal_window_size
                        #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，search_compeleted置1
                        if row_end >row_max:
                            # print("Seach Compeleted")
                            search_compeleted = 1
                            break#本帧图像检测完，跳出该for循环
                    #当前检测块不是安全区域，所以将find_window_success清零
                    find_window_success = 0
                    break#一旦确定该待检测块不是安全区域时，就跳出该for循环
            
            #本帧图像检测完成，跳出while循环
            if search_compeleted==1:
                break
            
            #如果当前检测块是安全区域，则在上述for循环中find_window_success不会被清零，就会执行下边if代码
            if find_window_success==1:
                # print("now window is safety")

                #记录待安全块的信息，包括中心点坐标X_point、Y_point，当前的检测起始边界
                X_point = (int)((column_start+column_end-1)/2)
                Y_point = (int)((row_start+row_end-1)/2)
                safty_poit.append([X_point,Y_point])
                safty_poit_window.append([row_start,column_start])

                #同时记录图像中心点，方便后期计算使用
                centrl_point.append([79,29])

                #当前区域是安全区域，待检测块右移column_step个像素
                column_start = column_start+column_step
                column_end = column_start + optimal_window_size

                #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                if column_end > 160:
                    column_start = 0
                    column_end = column_start + optimal_window_size
                    row_start = row_start+row_step
                    row_end = row_start + optimal_window_size
                    #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，跳出while循环
                    if row_end >row_max:
                        #print("Seach Compeleted")
                        break
            
        #输出本帧图像在最优滑窗大小下的所有安全点，以及与之对应的滑窗起始边界
        # print("safty_poit: ")
        # print(safty_poit)
        # print("safty_poit_window: ")
        # print(safty_poit_window)

        '''
        滑窗算法Step4 : 找出当前帧中安全区域
        '''
        #计算每个安全点到图像中心点的距离，距离为两点x、y坐标差的绝对值的和
        safty_point_err = np.subtract(safty_poit,centrl_point)
        safty_point_dist = abs(safty_point_err[:,0]) + abs(safty_point_err[:,1])

        # #输出距离
        # print("safty_point_dist: ")
        # print(safty_point_dist)

        #找出最小距离点对应的索引值optimal_index，并求出其对应的最优点optimal_point和对应的滑窗起始边界optimal_window_corresponding_to_optimal_point
        safty_point_dist = list(safty_point_dist)
        optimal_index = safty_point_dist.index(min(safty_point_dist))
        optimal_point = safty_poit[optimal_index]
        optimal_window_corresponding_to_optimal_point = safty_poit_window[optimal_index]

        #输出上述信息
        #print(("optimal_point: ",optimal_point),flush=True)
        # print("optimal_window_corresponding_to_optimal_point: ")
        # print(optimal_window_corresponding_to_optimal_point)
        # print("optimal_window_size:")
        # print(optimal_window_size)

        send_optical_flow_packet(optimal_point[0]-79,29-optimal_point[1],1,fd)
        print((optimal_point[0]-79,29-optimal_point[1],1),flush=True)

    else:
        #print(("当前帧没有可通行区域"),flush=True)
        send_optical_flow_packet(0,0,0,fd)
        print((0,0,0),flush=True)
    end_time = time.time()
    total_time = end_time - start_time
    fps = 1/total_time
    print(("FPS:{0}",format(fps)),flush=True)