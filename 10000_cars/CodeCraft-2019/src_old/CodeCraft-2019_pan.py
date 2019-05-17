# -*- coding: UTF-8 -*-

from numba import jit
import numpy as np
#from PIL import Image, ImageDraw, ImageFont
from Dijkstra import dijkstra
import copy
import logging
import sys

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')

# 道路限速与车辆速度不匹配
M_1 = 0.5
M_2 = 1.5
S_1 = 2

############################################准备工作#############################################
def read_txt(txt):
    messages = []
    with open(txt, 'r') as rf:
        header = rf.readline()
        for line in rf.readlines():
            message = line.strip().strip('()').split(',')
            for i in range(len(message)):
                message[i] = int(message[i].strip())
            messages.append(message)

    return messages


def read_data(car_path, road_path, cross_path):
    roads = read_txt(road_path)
    cars = read_txt(car_path)
    crosses = read_txt(cross_path)

    return roads, cars, crosses

# 建立邻接图
def build_graph(roads, crosses):

    cross_num = len(crosses)
    inf = float('inf')
    graph = [[inf for i in range(cross_num)] for i in range(cross_num)]

    for i in range(cross_num):
        cross = crosses[i]
        graph[cross[0]-1][cross[0]-1] = 0

        for j in range(1,5):
            if cross[j] != -1:
                road_id = cross[j]
                road = get_road(road_id, roads)
                distance = road[1]
                speed = road[2]
                start_cross = road[4]
                end_cross = road[5]
                #print('start:',start_cross,' end:',end_cross)
                if road[6] == 1:
                    graph[start_cross-1][end_cross-1] = graph[end_cross-1][start_cross-1] = distance
                else:
                    graph[start_cross-1][end_cross-1] = distance

    return graph

def get_road(road_id, roads):
    road_id_list = []
    for road in roads:
        road_id_list.append(road[0])

    road_index = road_id_list.index(road_id)

    return roads[road_index]
    

def trans_path(car_from, min_dist_path, roads):
    path = []
    start = car_from
    for i in range(len(min_dist_path)):
        end = min_dist_path[i]
        #print('start cross:',start,' end cross:',end)

        for road in roads:
            if (road[4] == start+1 and road[5] == end+1) or (road[5] == start+1 and road[4] == end+1):
                path.append(road[0])
        
        start = end

    return path


########################################查看路况##############################################

# 根据当前时间和图使用判题器获取当前时刻拥堵路况
def get_crowd_road(cars, roads, crosses, anwer_list, current_time):
    
    # 调用判题器返回道路路况

    return crowd_roads_list


########################################路径寻优##############################################

# 根据dijkstra算法寻找最优路径
def getroute(graph, car, source, target, roads, crowd_roads_list):

    # 根据道路限速和车辆速度初始化权值
    for i in range(len(roads)):
        if roads[i][6] == 1:
            if car[3] == roads[i][2]:
                graph[roads[i][4]-1][roads[i][5]-1] *= M_1
                graph[roads[i][5]-1][roads[i][4]-1] *= M_1
            elif (car[3] - roads[i][2] >= 2) or (roads[i][2] - car[3] >= 2):
                graph[roads[i][4]-1][roads[i][5]-1] *= M_2
                graph[roads[i][5]-1][roads[i][4]-1] *= M_2
                
        else:
            if car[3] == roads[i][2]:
                graph[roads[i][4]-1][roads[i][5]-1] *= M_1
            elif (car[3] - roads[i][2] >= 2) or (roads[i][2] - car[3] >= 2):
                graph[roads[i][4]-1][roads[i][5]-1] *= M_2
    
        # 根据道路拥挤状况调整图权值
        for i in range(len(crowd_roads_list)):
            road_from = crowd_roads_list[i].split('-')[0]
            road_to = crowd_roads_list[i].split('-')[1]
        
            # 当前拥挤道路的权值调整为inf
            graph[road_from-1][road_to-1] = float('inf')

    distance, path = dijkstra(graph, source)
    dist = {}
    for key, value in path.items():        
        pd = {}
        for k, v in value.items():
            pd[k] = {}
            pd[k]['path'] = v
        dist[key] = pd
    return dist[source][target]['path']

########################################发车规则##############################################

# 初始发车，初始时发几百辆车，道路中没有车，所以直接寻优
def init_schedule(distance_graph, cars, roads, answer_path):
    # 已出发车辆id列表
    init_go_cars = []
    init_cars = 400
    
    # answer列表，记录当前已发车辆路径，输入判题器
    # anwer_list = [(car_id, start_time, min_path)]
    answer_list = []
    
    # 初始发车时拥挤道路列表为空
    crowd_roads_list = []
    
    for i in range(len(cars)):
        car = cars[i]
        car_id = car[0]
        car_from = car[1]
        car_to = car[2]
        car_plantime = car[4]
        
        # 定义车辆发车时间，第一批车只从计划初始时间为1的车辆中取
        start_time = 1
        if car_plantime == 1:
            init_go_cars.append(car)
            
            # 调用getroute函数得到第一批出发车辆的最短路径
            min_path = getroute(copy.deepcopy(distance_graph), carcar_from-1, car_to-1, roads, crowd_roads_list)
            min_dist_path_by_road = trans_path(car_from, min_dist_path, roads)
            print('path by road:',min_dist_path_by_road)
            
            anwer_list.append((car_id, start_time, min_path))
        
            # 写入txt文件
            with open(answer_path, 'a') as wf:
                wf.write('('+str(car_id)+',')
                wf.write(str(start_time)+',')
                for road_id in range(len(min_dist_path_by_road)-1):
                    wf.write(str(min_dist_path_by_road[road_id])+',')

                wf.write(str(min_dist_path_by_road[-1])+')\r\n')
            
        if len(init_go_cars) >= init_cars:
            break
    return init_go_cars, answer_list


# 后续连续发车，每次发车时根据当前时刻路况修改图权值
def seqschedule(distance_graph, cars, roads, answer_path, crosses, init_go_cars, answer_list):
    # 剩余车辆
    seq_cars = [i for i in cars if i not in init_go_cars]
    
    # 连续发车已出发车辆
    seq_already_go = []
    
    # 保存同一时刻的车辆路径answer，在下一时刻列表清空
    middle_answer = []

    current_time = 2
    car_num = 20
    
    # 每隔1个时间发一定量的车
    for i in range(len(seq_cars)):
        car = seq_cars[i]
        car_id = car[0]
        car_from = car[1]
        car_to = car[2]
        car_plantime = car[4]    
        
        # 出发满足时间条件的车
        if car_plantime > current_time:
            pass
        else:
            start_time = current_time
            seq_already_go.append(car_id)
        
            # 调用判题器获取当前时刻路况，返回当前拥挤道路列表（以“路口——路口”形式保存，即区分方向）
            crowd_roads_list = get_crowd_road(cars, roads, crosses, anwer_list, current_time)
            
            min_path = getroute(copy.deepcopy(distance_graph), car, car_from-1, car_to-1, roads, current_time)
            min_dist_path_by_road = trans_path(car_from-1,min_path, roads)
            print('path by road:',min_dist_path_by_road)

            middle_answer.append((car_id, start_time, min_path))
            
            with open(answer_path, 'a') as wf:
                wf.write('('+str(car_id)+',')
                wf.write(str(start_time)+',')
                for road_id in range(len(min_dist_path_by_road)-1):
                    wf.write(str(min_dist_path_by_road[road_id])+',')

                wf.write(str(min_dist_path_by_road[-1])+')\r\n')
        
        if len(seq_already_go) >= car_num:
            # 每一时刻的所有车辆的answer在下一时刻时添加到answer列表中，用于下一时刻判题器
            answer_list += middle_answer
            seq_already_go = []
            middle_answer = []
            current_time += 1

###########################################################################################

def main():
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))


    roads, cars, crosses = read_data(car_path, road_path, cross_path)
    distance_graph = build_graph(roads, crosses)
    init_go_cars, answer_list = init_schedule(distance_graph, cars, roads, answer_path)
    seqschedule(distance_graph, cars, roads, answer_path, crosses, init_go_cars, answer_list)


if __name__ == "__main__":
    main()

    

