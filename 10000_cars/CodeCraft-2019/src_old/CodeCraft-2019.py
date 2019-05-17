# -*- coding: UTF-8 -*-

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
# 之前计算得到的车辆路径加权
B = 1.0


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


def build_graph(roads, crosses):
    #'''
    #读入路口和道路的信息，生成邻接图
    #通常的邻接图路径表示两点间的距离，而在这个任务中，需要考虑道路限速、车速等，所以可以将两点间的距离替换为该车通过需要的时间
    #'''
    #print('road txt:',roads)
    #print('cross txt:',crosses)

    cross_num = len(crosses)
    inf = float('inf')
    graph = [[inf for i in range(cross_num)] for i in range(cross_num)]
    #print(graph)
    #print(cross_num)
    #print(len(graph))

    for i in range(cross_num):
        cross = crosses[i]
        graph[cross[0]-1][cross[0]-1] = 0

        for j in range(1,5):
            if cross[j] != -1:
                road_id = cross[j]
                road = get_road(road_id, roads)
                distance = road[1]
                speed = road[2]
                channel = road[3]
                start_cross = road[4]
                end_cross = road[5]
                #print('start:',start_cross,' end:',end_cross)
                if road[6] == 1:
                    graph[start_cross-1][end_cross-1] = graph[end_cross-1][start_cross-1] = (1-0.15*(channel-1))*distance
                else:
                    graph[start_cross-1][end_cross-1] = (1-0.15*(channel-1))*distance

    return graph

def get_road(road_id, roads):
    road_id_list = []
    for road in roads:
        road_id_list.append(road[0])

    road_index = road_id_list.index(road_id)

    return roads[road_index]
    

def getroute(graph, car, source, target, roads):
    
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

    distance, path = dijkstra(graph, source)
    dist = {}
    for key, value in path.items():        
        pd = {}
        for k, v in value.items():
            pd[k] = {}
            pd[k]['path'] = v
        dist[key] = pd
    return dist[source][target]['path']


# 发车规则
def schedule(distance_graph, cars, roads, answer_path):

    #得到车辆出发时间
    car_start_time_list = []
    for car in cars:
        car_start_time_list.append(car[-1])
    
    #按出发时间排序
    sorted_car = np.argsort(car_start_time_list)

    current_time = 1
    end_index = 100
    
    #前面所有车路径列表，用于微调后续车的路径寻优
    last_path_list = []
    for i in range(len(cars)):
        #print(i)
        car = cars[sorted_car[i]]
        if car[-1] <= current_time:
            start_time = current_time
        else:
            start_time = car[-1]

        car_id = car[0]
        car_from = car[1]
        car_to = car[2]
        
        min_path = getroute(copy.deepcopy(distance_graph), car, car_from-1, car_to-1, roads)
        min_dist_path_by_road = trans_path(car_from-1,min_path, roads)
        last_path_list = last_path_list + min_dist_path_by_road
        #print('path by cross:',min_path)
        print('path by road:',min_dist_path_by_road)
        # print('\n')

        with open(answer_path, 'a') as wf:
            wf.write('('+str(car_id)+',')
            wf.write(str(start_time)+',')
            for road_id in range(len(min_dist_path_by_road)-1):
                wf.write(str(min_dist_path_by_road[road_id])+',')

            wf.write(str(min_dist_path_by_road[-1])+')\r\n')
        
        if i >= end_index:
            current_time += 8
            end_index += 100


# 路径0-1编码为100维的向量   
def encode(path_by_road, roads):
    road_ids = []
    for road in roads:
        road_ids.append(road[0])

    encoded_path = [0 for i in range(len(road_ids))]
    for i in range(len(road_ids)):
        if road_ids[i] in path_by_road:
            encoded_path[i] = 1

    return encoded_path   
   
 
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
    schedule(distance_graph, cars, roads, answer_path)

if __name__ == "__main__":
    main()
