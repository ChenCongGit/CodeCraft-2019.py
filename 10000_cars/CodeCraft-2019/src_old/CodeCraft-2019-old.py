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
    
def cal_min_dist_path(crosses, distance_graph):
    min_dist_dict = {}
    for i in range(len(crosses)):
        graph_temp = copy.deepcopy(distance_graph)
        dist = {}
        start_cross = i
        distance,path= dijkstra(graph_temp, start_cross)
        for key, value in path[i].items():
            mes = {}
            mes['path'] = value
            mes['distance'] = distance[key]
            dist[key] = mes
        min_dist_dict[start_cross] = dist

    #print(min_dist_dict)
    return min_dist_dict

def schedule(min_dist_dict, cars, roads, answer_path):
    car_start_time_list = []
    for car in cars:
        car_start_time_list.append(car[-1])

    sorted_car = np.argsort(car_start_time_list)
    #for index in sorted_car:
        #print(car_start_time_list[index])

    current_time = 1
    end_index = 100
    for i in range(len(cars)):
        car = cars[sorted_car[i]]
        if car[-1] <= current_time:
            start_time = current_time
        else:
            start_time = car[3]

        car_id = car[0]
        car_from = car[1]
        car_to = car[2]
        #print(min_dist_dict[car_from-1])
        min_dist_path = min_dist_dict[car_from-1][car_to-1]['path']

        min_dist_path_by_road = trans_path(car_from-1,min_dist_path, roads)
        print('path by cross:',min_dist_path)
        # print('path by road:',min_dist_path_by_road)
        # print('\n')

        with open(answer_path, 'a') as wf:
            wf.write('('+str(car_id)+',')
            wf.write(str(start_time)+',')
            for road_id in range(len(min_dist_path_by_road)-1):
                wf.write(str(min_dist_path_by_road[road_id])+',')

            wf.write(str(min_dist_path_by_road[-1])+')\r\n')

        if i >= end_index:
            current_time += 1
            end_index += 10

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
    min_dist_dict = cal_min_dist_path(crosses, distance_graph)
    schedule(min_dist_dict, cars, roads, answer_path)


if __name__ == "__main__":
    main()

    

