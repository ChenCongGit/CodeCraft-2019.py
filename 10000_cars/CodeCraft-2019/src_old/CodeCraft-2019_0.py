# -*- coding: UTF-8 -*-

import numpy as np
# from simulator import realtimetraffic
# from PIL import Image, ImageDraw, ImageFont
from Dijkstra import dijkstra
import copy
import logging
import sys
import random
import time

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')

# 道路限速与车辆速度不匹配
M_1 = 0.5
M_2 = 1.5
S_1 = 2

np.random.seed(951105)

TIME = [0]
fache_id = []
CARDISTRIBUTION = [0, 0, 0]  # 表示在车库中的车的数量、在路上的车的数量以及调度结束到达指定id的车的信息
CARNAMESPACE, ROADNAMESPACE, CROSSNAMESPACE = [], [], []
CROSSDICT, CARDICT, ROADDICT = {}, {}, {}
final_answer_dict = {}


############################################读取文件#############################################
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


############################################准备工作#############################################

# 建立道路和路口对应的字典
def road_cross(roads, crosses):
    road_cross_dict = {}
    cross_road_dict = {}
    for road in roads:
        road_id = road[0]
        road_from = road[4]
        road_to = road[5]
        road_isDuplex = road[6]

        road_cross_dict[road_id] = [[road_from, road_to], road_isDuplex]
        cross_road_dict[(road_from, road_to)] = road_id
        if road_isDuplex == 1:
            cross_road_dict[(road_to, road_from)] = road_id
    return road_cross_dict, cross_road_dict


# 建立道路名称和道路限速对应的字典
def road_id_speed(roads):
    road_id_speed_dict = {}
    road_id_channel_dict = {}
    for road in roads:
        road_id = road[0]
        road_speed = road[2]
        road_channel = road[3]

        road_id_speed_dict[road_id] = road_speed
        road_id_channel_dict[road_id] = road_channel
    return road_id_speed_dict, road_id_channel_dict


# 道路id和roads列表中索引的对应字典
def car_id_cars_index(cars):
    car_id_cars_index_dict = {}
    for i in range(len(cars)):
        car_id = cars[i][0]

        car_id_cars_index_dict[car_id] = i
    return car_id_cars_index_dict

# 将出发点一样的车组成batch
def same_from_to_batch(cars_id):
    batch_dict = {}
    for i in range(len(cars_id)):
        car_id = cars_id[i]
        car_from = CARDICT[car_id].from_
        key = car_from

        if key in batch_dict.keys():
            batch_dict[key].append(car_id)
        else:
            batch_dict[key] = []
            batch_dict[key].append(car_id)
    return batch_dict


############################################建立图模型############################################

# 建立邻接图
def build_graph(roads, crosses):
    cross_num = len(crosses)
    inf = float('inf')
    graph = [[inf for i in range(cross_num)] for i in range(cross_num)]

    for i in range(cross_num):
        cross = crosses[i]
        graph[cross[0] - 1][cross[0] - 1] = 0

        for j in range(1, 5):
            if cross[j] != -1:
                road_id = cross[j]
                road = get_road(road_id, roads)
                distance = road[1]
                speed = road[2]
                channel = road[3]
                start_cross = road[4]
                end_cross = road[5]
                # print('start:',start_cross,' end:',end_cross)
                if road[6] == 1:
                    graph[start_cross - 1][end_cross - 1] = graph[end_cross - 1][start_cross - 1] = (1 - 0.15 * (
                            channel - 1)) * distance
                else:
                    graph[start_cross - 1][end_cross - 1] = (1 - 0.15 * (channel - 1)) * distance
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
        # print('start cross:',start,' end cross:',end)

        for road in roads:
            if (road[4] == start + 1 and road[5] == end + 1) or (road[5] == start + 1 and road[4] == end + 1):
                path.append(road[0])
        start = end
    return path

# ######################################路径寻优##############################################

# 创建最优路径字典
def cal_min_dist_path(crosses, distance_graph):
    min_dist_dict = {}
    for i in range(len(crosses)):
        graph_temp = copy.deepcopy(distance_graph)
        dist = {}
        start_cross = i
        distance, path = dijkstra(graph_temp, start_cross)
        for key, value in path[i].items():
            mes = {}
            mes['path'] = value
            mes['distance'] = distance[key]
            dist[key] = mes
        min_dist_dict[start_cross] = dist

    # print(min_dist_dict)
    return min_dist_dict

##########################################发车规则#############################################

# 初始发车，初始时发几百辆车，道路中没有车，所以直接寻优
def init_schedule(min_dist_dict, cars, roads):
    TIME[0] = 1

    # 已出发车辆id列表
    init_go_cars = []
    init_cars = 400

    for i in range(len(cars)):
        # print(i)
        car = cars[i]
        car_id = car[0]
        car_from = car[1]
        car_to = car[2]
        car_plantime = car[4]

        # 定义车辆发车时间，第一批车只从计划初始时间为1的车辆中取
        start_time = 1
        if car_plantime == 1:
            init_go_cars.append(car)

            # 使用最短路径字典得到第一批出发车辆的最短路径
            min_dist_path = min_dist_dict[car_from - 1][car_to - 1]['path']
            min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)
            # print('path by road:',min_dist_path_by_road)

            CARDICT[car_id].simulateInit(start_time, [min_dist_path_by_road[0]])
            CROSSDICT[CARDICT[car_id].__from__()].carportInitial(CARDICT[car_id].__planTime__(), car_id)

        if len(init_go_cars) >= init_cars:
            break

    return init_go_cars


# 后续连续发车，每次发车时根据当前时刻路况修改图权值
def schedule(min_dist_dict, cars, roads, crosses, answer_path, init_go_cars):
    # 道路名和路口对应字典
    road_cross_dict, cross_road_dict = road_cross(roads, crosses)

    # 道路名和道路速度对应字典
    road_id_speed_dict, road_id_channel_dict = road_id_speed(roads)

    sim = simulation()
    getroadinfor = getroadinformation()

    # 当前时刻已发车辆列表
    on_road_cars = [car[0] for car in init_go_cars]

    # 未发车辆
    in_carport_cars = [car_id for car_id in CARNAMESPACE if car_id not in on_road_cars]

    # 相同出发点车辆
    batch_dict = same_from_to_batch(in_carport_cars)

    while True:

        # 每秒钟取车库的300辆车判断是否发车
        #will_go_cars = in_carport_cars[:300]

        # 每秒钟取车库不同出发点的车辆判断是否发车
        will_go_cars = []
        for in_port_car in batch_dict.values():
            if TIME[0] <= 550:
                will_go_cars += in_port_car[:5]
            else:
                will_go_cars += in_port_car[:200]
        #print(will_go_cars)

        cars_id = on_road_cars + will_go_cars

        print(TIME[0])
        sim.step()

        roadInfor, carInfor = getroadinfor.getinfor()

        # 调用判题器获取当前时刻路况，返回当前拥挤道路列表（以[路口,路口]形式保存，即区分方向）
        crowd_roads_from = []
        crowd_roads_to = []
        will_go_crowd_roads_from = []
        will_go_crowd_roads_to = []
        for key, value in roadInfor.items():
            if TIME[0] < 650:
                if roadInfor[key][2] > 0.6 or ROADDICT[key].channel_ <= 2 and roadInfor[key][2] > 0.50:
                    crowd_roads_from.append(roadInfor[key][0])
                    crowd_roads_to.append(roadInfor[key][1])
                if roadInfor[key][3] > 0.6 or ROADDICT[key].channel_ <= 2 and roadInfor[key][3] > 0.50:
                    crowd_roads_from.append(roadInfor[key][1])
                    crowd_roads_to.append(roadInfor[key][0])
            else:
                if roadInfor[key][2] > 0.8 or ROADDICT[key].channel_ <= 2 and roadInfor[key][2] > 0.60:
                    crowd_roads_from.append(roadInfor[key][0])
                    crowd_roads_to.append(roadInfor[key][1])
                if roadInfor[key][3] > 0.8 or ROADDICT[key].channel_ <= 2 and roadInfor[key][3] > 0.60:
                    crowd_roads_from.append(roadInfor[key][1])
                    crowd_roads_to.append(roadInfor[key][0])

            if TIME[0] < 550:
                if roadInfor[key][2] > 0.25 or ROADDICT[key].channel_ <= 2 and roadInfor[key][2] > 0.15:
                    will_go_crowd_roads_from.append(roadInfor[key][0])
                    will_go_crowd_roads_to.append(roadInfor[key][1])
                if roadInfor[key][3] > 0.25 or ROADDICT[key].channel_ <= 2 and roadInfor[key][3] > 0.15:
                    will_go_crowd_roads_from.append(roadInfor[key][1])
                    will_go_crowd_roads_to.append(roadInfor[key][0])
            elif TIME[0] < 650:
                if roadInfor[key][2] > 0.6 or ROADDICT[key].channel_ <= 2 and roadInfor[key][2] > 0.5:
                    will_go_crowd_roads_from.append(roadInfor[key][0])
                    will_go_crowd_roads_to.append(roadInfor[key][1])
                if roadInfor[key][3] > 0.6 or ROADDICT[key].channel_ <= 2 and roadInfor[key][3] > 0.5:
                    will_go_crowd_roads_from.append(roadInfor[key][1])
                    will_go_crowd_roads_to.append(roadInfor[key][0])
            else:
                pass


        # 当前时刻所有路口已出发车辆数列表
        cross_already_go_cars = [0 for i in range(len(crosses))]

        # 道路id和roads列表中索引的对应字典
        car_id_cars_index_dict = car_id_cars_index(cars)

        on_road_num = 0
        arrive_num = 0
        not_go_num = 0
        will_go_num = 0

        if cars_id == []:
            break
        else:
            for car_id in cars_id:

                car = CARDICT[car_id]
                car_from = car.from_
                car_to = car.to_

                car_plan_time = cars[car_id_cars_index_dict[car_id]][4]
                present_road = car.__presentRoad__()
                last_time_road = car.__lastTimeRoad__()
                next_cross = car.__nextCrossId__()

                # 判断车辆状态，如果为start则尚未出发，如果为end则已到达，其他状态则在路上
                if (car.__state__() == 1 or car.__state__() == 2) and car. __nextCrossId__() != car_to:
                    on_road_num += 1
                    # 通过dj算法获得初步路径
                    if next_cross != car_to and car_id in on_road_cars:#current_car_dict.keys():

                        min_dist_path = min_dist_dict[next_cross - 1][car_to - 1]['path']
                        min_dist_path_by_road = trans_path(next_cross - 1, min_dist_path, roads)

                        # 如果通过dj算法寻找到的出发路径是拥堵状态，则选择该路口另外的道路
                        # 如果算法得到的出发路径不拥堵，则选择该路径
                        if (next_cross in crowd_roads_from) and ((min_dist_path[0] + 1) in crowd_roads_to) or (present_road == min_dist_path_by_road[0]):
                            # 路口可选道路列表和其对应的与车辆速度之差列表
                            choose_road_list = []
                            choose_road_speed_list = []
                            choose_road_channel_list = []

                            # 如果当前时刻车辆与上一时刻不在同一条路上，则用该时刻的寻找到的道路加入answer字典中
                            # 如果当前时刻车辆与上一时刻在同一条路上，则用该时刻寻找到的道路替换上一时刻
                            if present_road != last_time_road:
                                select_road = [road for road in crosses[next_cross - 1][1:5] if road != -1 and road != present_road]
                                # 遍历当前路口的所有方向的道路
                                for road in select_road:
                                    if len(select_road) != 1:
                                        if ((road_cross_dict[road][1] == 1) or (road_cross_dict[road][0][0] == next_cross)):
                                            # 判断该道路是否是拥挤道路，如果不是该道路可选
                                            if (road_cross_dict[road][0][0] not in crowd_roads_from) or (
                                                            road_cross_dict[road][0][1] not in crowd_roads_to):
                                                choose_road_list.append(road)
                                                choose_road_channel_list.append(ROADDICT[road].channel_)

                                    else:
                                        choose_road_list = [road]
                                        choose_road_channel_list = [ROADDICT[road].channel_]


                                if choose_road_list != []:
                                    road_index = np.argmax(choose_road_channel_list)
                                    next_road = choose_road_list[road_index]
                                    car.__route__().append(next_road)
                                else:
                                    next_road = select_road[0]
                                    CARDICT[car_id].__route__().append(next_road)
                        else:
                            if TIME[0] == 1 or present_road != last_time_road:
                                car.__route__().append(min_dist_path_by_road[0])

                elif car. __nextCrossId__() == car_to:
                    arrive_num += 1
                    # 如果车辆状态为end，则在answer中将该车路径去掉，在cars中也将该车去掉，即以后不再遍历该车
                    on_road_cars.remove(car_id)

                    # 写入到达车辆路径
                    with open(answer_path, 'a') as wf:
                        wf.write('(' + str(car_id) + ',')  # 写入car id
                        wf.write(str(car.planTime_) + ',')  # 写入实际出发时间
                        for road_id in range(len(car.route) - 1):  # 写入行驶路径
                            wf.write(str(car.route[road_id]) + ',')

                        wf.write(str(car.route[-1]) + ')\r\n')

                else:
                    # 如果车辆状态为start，即尚未出发，则通过拥堵条件判断该车能否出发
                    # 如果某辆车出发时，其出发路口周围的道路拥堵或者当前时刻已经该路口已经出发了很多车，则延迟该路口的发车
                    if car.planTime_  == -1:
                        not_go_num += 1
                        if (car_plan_time <= TIME[0]) and (car_from not in will_go_crowd_roads_from) and (
                            car_from not in will_go_crowd_roads_to) and ((cross_already_go_cars[car_from - 1] <= 3)
                                or (cross_already_go_cars[car_from - 1] <= 50) and (on_road_num < 800)and TIME[0]>550):
                            will_go_num += 1
                            cross_already_go_cars[car_from - 1] += 1
                            min_dist_path = min_dist_dict[car_from - 1][car_to - 1]['path']
                            min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)

                            car.route = [min_dist_path_by_road[0]]
                            car.planTime_ = TIME[0]+1
                            car.routeIndex = 0
                            CROSSDICT[car.__from__()].carportInitial(car.__planTime__(), car_id)

                            batch_dict[car_from].remove(car_id)
                            on_road_cars.append(car_id)

                car.lastTimeRoad = present_road

        for key, value in batch_dict.items():
            not_go_num += len(value)
        print('cars_id_length',len(cars_id), 'on_road:', on_road_num, 'not_go:', not_go_num, 'will_go:', will_go_num, 'arrive:',arrive_num)
        if CARDISTRIBUTION[2] == CARNAMESPACE.__len__() or (on_road_num <= 500 and TIME[0] > 550 and not_go_num <=500):
            break_time = TIME[0]
            break
        else:
            TIME[0] += 1

    # 最后剩余300辆车时，直接采用最短路径运行
    for i in range(len(on_road_cars)):
        car_id = on_road_cars[i]
        car_from = CARDICT[car_id].from_
        car_to = CARDICT[car_id].to_
        car_plan_time = cars[car_id_cars_index_dict[car_id]][4]

        # 定义车辆发车时间，第一批车只从计划初始时间为1的车辆中取
        start_time = 1
        if car_plan_time <= break_time:
            # 使用最短路径字典得到第一批出发车辆的最短路径
            min_dist_path = min_dist_dict[car_from - 1][car_to - 1]['path']
            min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)

            # 写入到达车辆路径
            with open(answer_path, 'a') as wf:
                wf.write('(' + str(car_id) + ',')  # 写入car id
                wf.write(str(break_time-100) + ',')  # 写入实际出发时间
                for road_id in range(len(min_dist_path_by_road) - 1):  # 写入行驶路径
                    wf.write(str(min_dist_path_by_road[road_id]) + ',')

                wf.write(str(min_dist_path_by_road[-1]) + ')\r\n')

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

    carInfo = open(car_path, 'r').read().split('\n')[1:]
    roadInfo = open(road_path, 'r').read().split('\n')[1:]
    crossInfo = open(cross_path, 'r').read().split('\n')[1:]
    # answerInfo = open(answer_path, 'r').read().split('\n')
    # *****************************Create NameSpace And Dictionary*****************************#
    # create car objects
    # line = (id,from,to,speed,planTime)
    for line in carInfo:
        id_, from_, to_, speed_, planTime_ = line.replace(' ', '').replace('\t', '')[1:-1].split(',')
        CARNAMESPACE.append(int(id_))
        CARDICT[int(id_)] = CAR(int(id_), int(from_), int(to_), int(speed_), int(planTime_))
    # create road objects
    # line = (id,length,speed,channel,from,to,isDuplex)
    for line in roadInfo:
        id_, length_, speed_, channel_, from_, to_, isDuplex_ = line.replace(' ', '').replace('\t', '')[1:-1].split(',')
        ROADNAMESPACE.append(int(id_))
        ROADDICT[int(id_)] = ROAD(int(id_), int(length_), int(speed_), int(channel_), int(from_), int(to_),
                                  int(isDuplex_))
    # create cross objects
    # line = (id,north,east,south,west)
    for line in crossInfo:
        id_, north_, east_, south_, west_ = line.replace(' ', '').replace('\t', '')[1:-1].split(',')
        CROSSNAMESPACE.append(int(id_))
        CROSSDICT[int(id_)] = CROSS(int(id_), int(north_), int(east_), int(south_), int(west_))

    CARNAMESPACE.sort()  # 车的id的排序
    CROSSNAMESPACE.sort()

    roads, cars, crosses = read_data(car_path, road_path, cross_path)
    distance_graph = build_graph(roads, crosses)
    min_dist_dict = cal_min_dist_path(crosses, distance_graph)
    init_go_cars = init_schedule(min_dist_dict, cars, roads)
    schedule(min_dist_dict, cars, roads, crosses, answer_path, init_go_cars)


###############################################################################################
###############################################################################################

class CAR(object):
    def __init__(self, id_, from_, to_, speed_, planTime_):
        """"**** statistic parameters ****"""
        self.id_, self.from_, self.to_, self.speed_, self.planTime_ = id_, from_, to_, speed_, -1
        self.carColor = [int(value) for value in np.random.random_integers(0, 255, [3])]
        # **** dynamic parameters ****#
        self.state, self.x, self.y = 0, 0, 0
        self.lastTimeRoad = None
        self.presentRoad, self.nextCrossId = None, self.from_
        self.deltaX, self.deltaY = 0, 0
        self.wait = False
        self.route, self.routeIndex = None, None

    def simulateInit(self, planTime, route):
        self.planTime_, self.route, self.routeIndex = planTime, route, 0

    def updateDynamic(self, state, x=None, y=None, presentRoad=None, roadSpeed=None, nextCrossId=None):
        # 车不在车库中表示车已准备出发
        # 车的状态car state: 0,1,2,3 分别表示在车库中、等待、完成、结束
        if self.state != 0 or presentRoad is not None:
            self.state = state
        # routeIndex为路的索引，每辆车的路线为route=[id0,id1,id2,id3,.....]
        if presentRoad is not None and self.state != 0 and self.routeIndex < self.route.__len__():
            self.routeIndex += 1
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.presentRoad = presentRoad if presentRoad is not None else self.presentRoad
        if nextCrossId is not None:
            self.nextCrossId = nextCrossId
            toX, toY = CROSSDICT[self.to_].__loc__()
            nextCrossX, nextCrossY = CROSSDICT[nextCrossId].__loc__()
            self.deltaX, self.deltaY = toX - nextCrossX, toY - nextCrossY

    def __id__(self):
        return self.id_

    def __from__(self):
        return self.from_

    def __to__(self):
        return self.to_

    def __speed__(self):
        return self.speed_

    def __planTime__(self):
        return self.planTime_

    def __carColor__(self):
        return self.carColor

    # 车的状态car state: 0,1,2,3 分别表示在车库中、等待、完成、结束
    def __state__(self):
        return self.state

    def __x__(self):
        return self.x

    def __y__(self):
        return self.y

    # 上一时刻车辆所在道路
    def __lastTimeRoad__(self):
        return self.lastTimeRoad

    # 当前道路
    def __presentRoad__(self):
        return self.presentRoad

    # 下一个路口id
    def __nextCrossId__(self):
        return self.nextCrossId

    def __deltaX__(self):
        return self.deltaX

    def __deltaY__(self):
        return self.deltaY

    def __wait__(self):
        return self.wait

    def __route__(self):
        return self.route

    def __routeIndex__(self):
        return self.routeIndex

    # 获取车辆在该道路上的最大车速
    def __v__(self):
        return min(self.speed_, ROADDICT[self.presentRoad].__speed__())

    def __distance__(self):
        return abs(self.deltaX) + abs(self.deltaY)

    def __nextRoad__(self):
        try:
            return self.route[self.routeIndex]
        except:
            return -1


class ROAD(object):
    def __init__(self, id_, length_, speed_, channel_, from_, to_, isDuplex_):
        # **** statistic parameters ****#
        self.id_, self.length_, self.speed_, self.channel_, self.from_, self.to_, self.isDuplex_ = \
            id_, length_, speed_, channel_, from_, to_, isDuplex_
        self.carCapcity = self.channel_ * self.length_

        self.singleCarCapcity = self.channel_ * self.length_

        # **** dynamic parameters ****#
        # absolute bucket
        # 正反向双向通道路径的记录，判断当存在双向标志时，对反向的通道进行记录，否则只有单向的车道记录
        self.forwardBucket = {i: [None for j in range(self.channel_)] for i in range(self.length_)}
        self.backwardBucket = {i: [None for j in range(self.channel_)] for i in
                               range(self.length_)} if self.isDuplex_ else None
        self.fx, self.fy, self.bx, self.by, self.forwardNum, self.backwardNum = [0], [0], [0], [0], [0], [0]
        self.forwardDone, self.backwardDone = [False], [False]
        # relative bucket
        self.provideBucket, self.receiveBucket = None, None
        self.px, self.py, self.provideNum, self.receiveNum = None, None, None, None
        self.provideDone = None

    #
    # determine relative bucket
    #
    # 选择是顺向形式，还是另外一个方向行驶
    def chooseAbsoluteBucket(self, crossId, pr):
        if crossId == self.from_ and pr == 'provide':
            return 'backward'
        elif crossId == self.from_ and pr == 'receive':
            return 'forward'
        elif crossId == self.to_ and pr == 'provide':
            return 'forward'
        elif crossId == self.to_ and pr == 'receive':
            return 'backward'
        #else:
            #print("Keywords mistake in CAR.chooseAbsoluteBucket()")

    def setBucket(self, crossId):
        bucket = self.chooseAbsoluteBucket(crossId, 'provide')
        if bucket == 'forward':
            self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                [self.forwardBucket, self.fx, self.fy, self.forwardDone, self.forwardNum]
            if self.isDuplex_:
                self.receiveBucket, self.receiveNum = \
                    self.backwardBucket, self.backwardNum
            else:
                self.receiveBucket, self.receiveNum = None, None
        else:
            self.receiveBucket, self.receiveNum = \
                self.forwardBucket, self.forwardNum
            if self.isDuplex_:
                self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                    self.backwardBucket, self.bx, self.by, self.backwardDone, self.backwardNum
            else:
                self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                    None, None, None, None, None

    #
    # 道路初始化
    #
    def stepInit(self):
        # dynamic param initialization
        self.fx, self.fy, self.bx, self.by = [0], [0], [0], [0]
        self.forwardDone, self.backwardDone = [False], [False]
        self.provideBucket, self.receiveBucket = None, None
        self.px, self.py, self.provideNum, self.receiveNum = None, None, None, None
        self.provideDone = None
        # car state initialization
        for i in range(self.length_):
            for j in range(self.channel_):
                if self.forwardBucket[i][j] is not None:
                    car = CARDICT[self.forwardBucket[i][j]]
                    car.updateDynamic(state=1)
                if self.isDuplex_:
                    if self.backwardBucket[i][j] is not None:
                        car = CARDICT[self.backwardBucket[i][j]]
                        car.updateDynamic(state=1)  # 车的状态car state: 0,1,2,3 分别表示在车库中、等待、完成、结束
        # first step
        for channel in range(self.channel_):
            self.moveInChannel(self.forwardBucket, channel)
            if self.isDuplex_:
                self.moveInChannel(self.backwardBucket, channel)

    #
    # function for bucket action
    #
    def moveInChannel(self, bucket, channel):
        # 车的状态car state: 0,1,2,3 分别表示在车库中、等待、完成、结束,一旦car到达完毕状态，road中car所在车道做一次”道内移动“
        # 设置标志(警卫)，用来监控道路状态,用于判断是否可以移动
        previousCar, previousState = -1, 1
        for i in range(self.length_):
            if bucket[i][channel] is not None:
                car = CARDICT[bucket[i][channel]]
                # 获取车道和车的最小速度
                v = car.__v__()
                if car.__state__() == 2:  # 车的状态为2表示  完成
                    previousCar, previousState = i, 2
                    continue
                elif i - v > previousCar:
                    bucket[i - v][channel] = bucket[i][channel]  # 移动车一次
                    bucket[i][channel] = None  # 并将移动后的空出来的车位设置为空
                    previousCar, previousState = i - v, 2
                    car.updateDynamic(state=2, x=previousCar)  # 对车的状态进行更新，对车的位置进行更新
                elif previousState == 2:
                    if previousCar + 1 != i:
                        bucket[previousCar + 1][channel] = bucket[i][channel]
                        bucket[i][channel] = None
                    previousCar, previousState = previousCar + 1, 2
                    car.updateDynamic(state=2, x=previousCar)
                else:
                    previousCar, previousState = i, 1

    def findCar(self, st, end, channel, bucket):
        # find car backward
        for i in range(end, st, -1):
            if bucket[i][channel] is not None:
                return i
        return -1

    #
    # provide car
    #
    def firstPriorityCar(self):
        # if self.provideBucket is None:
        #     print("Please do CAR.setBucket() first!")
        while True:
            if self.px[0] == self.length_:
                break
            carId = self.provideBucket[self.px[0]][self.py[0]]
            if carId is not None and CARDICT[carId].__state__() != 2:
                car = CARDICT[carId]
                left = car.__v__()
                # speed enough and no front car
                if left > self.px[0] and self.findCar(-1, self.px[0] - 1, self.py[0], self.provideBucket) == -1:
                    return self.provideBucket[self.px[0]][self.py[0]]
            if self.py[0] == self.channel_ - 1:
                self.px[0], self.py[0] = self.px[0] + 1, 0
            else:
                self.py[0] += 1
        self.provideDone[0] = True
        return -1

    def firstPriorityCarAct(self, action):
        # if self.provideBucket is None:
        #     print("Please do CAR.setBucket() first!")
        if action == 0:
            self.provideBucket[self.px[0]][self.py[0]] = None
            self.provideNum[0] -= 1
        elif action == 1:
            carId = self.provideBucket[self.px[0]][self.py[0]]
            self.provideBucket[self.px[0]][self.py[0]] = None
            self.provideBucket[0][self.py[0]] = carId
        self.moveInChannel(self.provideBucket, self.py[0])

    #
    # receive car
    #
    def receiveCar(self, carId):
        # if self.receiveBucket is None:
        #     print("Please do CAR.setBucket() first!")
        car = CARDICT[carId]
        leftX = min(self.speed_, car.__speed__()) - car.__x__()
        nextCrossId = self.from_ if car.__nextCrossId__() != self.from_ else self.to_
        if leftX <= 0:
            car.updateDynamic(state=2, x=0)
            return 1
        # find front car
        for i in range(self.channel_):
            frontCarLoc = self.findCar(self.length_ - leftX - 1, self.length_ - 1, i, self.receiveBucket)
            # if no front car
            if frontCarLoc == -1:
                self.receiveBucket[self.length_ - leftX][i] = carId
                self.receiveNum[0] += 1
                car.updateDynamic(state=2, x=self.length_ - leftX, y=i, presentRoad=self.id_, roadSpeed=self.speed_,
                                  nextCrossId=nextCrossId)
                return 0
            frontCar = CARDICT[self.receiveBucket[frontCarLoc][i]]
            # if frontCar.state == waiting
            if frontCar.__state__() == 1:
                return 2
            # if frontCar.state == finish and frontCar.x != road.__length__()-1
            elif frontCarLoc != self.length_ - 1:
                self.receiveBucket[frontCarLoc + 1][i] = carId
                self.receiveNum[0] += 1
                car.updateDynamic(state=2, x=frontCarLoc + 1, y=i, presentRoad=self.id_, roadSpeed=self.speed_,
                                  nextCrossId=nextCrossId)
                return 0
            # if frontCar.state == finish and frontCar.x == road.__length__()-1
            else:
                continue
        # if cars' state in all channel is equal to finish
        car.updateDynamic(state=2, x=0)
        return 1

    def __id__(self):
        return self.id_

    def __length__(self):
        return self.length_

    def __speed__(self):
        return self.speed_

    def __channel__(self):
        return self.channel_

    def __from__(self):
        return self.from_

    def __to__(self):
        return self.to_

    def __isDuplex__(self):
        return self.isDuplex_

    def __carCapcity__(self):
        return self.carCapcity

    def __singleCarCapcity__(self):
        return self.singleCarCapcity

    #
    # show statistic parameters
    #
    def __forwardBucket__(self):
        return self.forwardBucket

    def __backwardBucket__(self):
        return self.backwardBucket

    def __fx__(self):
        return self.fx[0]

    def __fy__(self):
        return self.fy[0]

    def __bx__(self):
        return self.bx[0]

    def __by__(self):
        return self.by[0]

    def __forwardNum__(self):
        return self.forwardNum[0]

    def __backwardNum__(self):
        return self.backwardNum[0]

    def __forwardDone__(self):
        return self.forwardDone[0]

    def __backwardDone__(self):
        return self.backwardDone[0]

    def __provideBucket__(self):
        return self.provideBucket

    def __receiveBucket__(self):
        return self.receiveBucket

    def __px__(self):
        return self.px[0]

    def __py__(self):
        return self.py[0]

    def __provideNum__(self):
        return self.provideNum[0]

    def __receiveNum__(self):
        return self.receiveNum[0]

    def __provideDone__(self):
        return self.provideDone[0]


def takeSecond(elem):
    return elem[1]


class CROSS(object):
    def __init__(self, id_, north_, east_, south_, west_):
        # **** statistic parameters ****#
        self.id_ = id_
        self.roadIds = [north_, east_, south_, west_]
        self.carport = {}
        self.left = []
        # absolute loc
        self.x, self.y = 0, 0
        self.mapX, self.mapY = 0, 0
        # priorityMap
        self.directionMap = {north_: {east_: 1, south_: 2, west_: -1}, east_: {south_: 1, west_: 2, north_: -1},
                             south_: {west_: 1, north_: 2, east_: -1}, west_: {north_: 1, east_: 2, south_: -1}}
        # relationship with roads
        self.providerDirection, self.receiverDirection, self.validRoadDirecction = [], [], []
        for index, roadId in enumerate(self.roadIds):
            road = ROADDICT[roadId] if roadId != -1 else None
            if road is not None and (road.__isDuplex__() or road.__to__() == self.id_):
                self.providerDirection.append(index)
            if road is not None and (road.__isDuplex__() or road.__from__() == self.id_):
                self.receiverDirection.append(index)
            if road is not None:
                self.validRoadDirecction.append(index)

        self.provider = [[direction, self.roadIds[direction]] for direction in self.providerDirection]
        self.receiver = [self.roadIds[direction] for direction in self.receiverDirection]
        self.validRoad = [self.roadIds[direction] for direction in self.validRoadDirecction]
        self.provider.sort(key=takeSecond)
        self.providerDirection = [self.provider[i][0] for i in range(self.provider.__len__())]
        self.provider = [self.provider[i][1] for i in range(self.provider.__len__())]

        # **** dynamic parameters ****#
        self.readyCars = []
        self.carportCarNum = 0
        self.finishCarNum = 0
        # **** flag ****#
        self.done = False
        self.update = False

    # main functions
    def step(self):
        self.update = False
        for roadId in self.validRoad:
            ROADDICT[roadId].setBucket(self.id_)
        # data preapre
        nextCarId, nextCar, nextRoad, nextDirection = [], [], [], []
        #
        # 0,1,2,3 denote north,east,south,west
        #
        for index in range(self.provider.__len__()):
            nextCarId.append(ROADDICT[self.provider[index]].firstPriorityCar())
            # if first priority car exists
            if nextCarId[index] != -1:
                nextCar.append(CARDICT[nextCarId[index]])
                nextRoad.append(nextCar[index].__nextRoad__())
                # nextRoad == -1 => terminal
                if nextRoad[index] == -1:
                    nextDirection.append(2)
                else:
                    #print(nextCarId[index].id_, nextCarId[index].route,'nnnnnnnnn')
                    #print(self.provider, nextRoad)
                    nextDirection.append(self.direction(self.provider[index], nextRoad[index]))
            else:
                nextCar.append(-1)
                nextRoad.append(-1)
                nextDirection.append(-1)
        # loop
        for presentRoadIndex in range(self.provider.__len__()):
            while nextCar[presentRoadIndex] != -1:
                # same next road and high priority lead to conflict
                provider = ROADDICT[self.provider[presentRoadIndex]]
                for otherRoadIndex in range(self.provider.__len__()):
                    # conflict
                    # first priority car exists at road self.provider[otherRoadIndex]
                    if nextCar[otherRoadIndex] != -1 and \
                            self.isConflict(self.providerDirection[presentRoadIndex], nextDirection[presentRoadIndex],
                                            self.providerDirection[otherRoadIndex], nextDirection[otherRoadIndex]):
                        break
                if nextRoad[presentRoadIndex] == -1:
                    provider.firstPriorityCarAct(0)
                    CARDISTRIBUTION[1] -= 1
                    CARDISTRIBUTION[2] += 1
                    self.finishCarNum += 1
                    self.update = True
                else:
                    nextroad_ = ROADDICT[nextRoad[presentRoadIndex]]
                    action = nextroad_.receiveCar(nextCar[presentRoadIndex].__id__())
                    if action == 2:
                        break
                    self.update = True
                    provider.firstPriorityCarAct(action)
                nextCarId[presentRoadIndex] = provider.firstPriorityCar()
                if nextCarId[presentRoadIndex] != -1:
                    nextCar[presentRoadIndex] = CARDICT[nextCarId[presentRoadIndex]]
                    nextRoad[presentRoadIndex] = nextCar[presentRoadIndex].__nextRoad__()
                    # nextRoad == -1 => terminal
                    if nextRoad[presentRoadIndex] == -1:
                        nextDirection[presentRoadIndex] = 2
                    else:
                        nextDirection[presentRoadIndex] = self.direction(self.provider[presentRoadIndex],
                                                                         nextRoad[presentRoadIndex])
                else:
                    nextCar[presentRoadIndex] = -1
                    nextRoad[presentRoadIndex] = -1
                    nextDirection[presentRoadIndex] = -1
        done = True
        for fromA in range(self.provider.__len__()):
            if nextCar[fromA] != -1:
                done = False
        self.done = done

    def outOfCarport(self):
        self.readyCars = self.left
        self.left = []
        if TIME[0] in self.carport.keys():
            self.carport[TIME[0]].sort()
            self.readyCars.extend(self.carport[TIME[0]])
        if self.readyCars.__len__() == 0:
            return
        # self.readyCars.sort()
        for roadId in self.receiver:
            ROADDICT[roadId].setBucket(self.id_)
        for i in range(self.readyCars.__len__()):
            carId = self.readyCars[i]
            roadId = CARDICT[carId].__nextRoad__()
            road = ROADDICT[roadId]
            #if roadId not in self.receiver:
            #    print("Car(%d).Road(%d) not in cross(%d).function:class.outOfCarport" % (carId, roadId, self.id_))
            act = road.receiveCar(carId)
            if act != 0:
                self.left = self.readyCars[i:]
                break
            # assert act==0, print("Time(%d),Cross(%d),Road(%d),Car(%d) can't pull away from carport"%(TIME[0],self.id_,roadId,carId))
            self.carportCarNum -= 1
            CARDISTRIBUTION[0] -= 1
            CARDISTRIBUTION[1] += 1

    #
    # other functions
    #
    def isConflict(self, fromA, directionA, fromB, directionB):
        # -1,0,1,2,3,4,5 => guard_w,n,e,s,w,guard_n,guard_e
        # -1 ,1, 2 => right left, straight
        # reason why:
        # direction:-1,1,2
        # 0-1=-1=>3=>west,0+1=1=>east,0+2=>2=>south
        # 1-1=0=>north,1+2=2=>south,1+2=3=>west
        # and so...
        #       0
        #   3       1
        #       2
        #
        if (fromA + directionA) % 4 == (fromB + directionB) % 4 and directionA < directionB:
            return True
        else:
            return False

    def direction(self, providerId, receiverId):
        #print(CARDICT[car_id], providerId, receiverId, self.directionMap)
        return self.directionMap[providerId][receiverId]

    def setDone(self, bool):
        self.done = bool

    def setLoc(self, x, y):
        self.x, self.y = x, y

    def setMapLoc(self, mapX, mapY):
        self.mapX, self.mapY = mapX, mapY

    def roadDirection(self, roadId):
        if self.roadIds[0] == roadId:
            return 0
        elif self.roadIds[1] == roadId:
            return 1
        elif self.roadIds[2] == roadId:
            return 2
        elif self.roadIds[3] == roadId:
            return 3
        else:
            return -1

    def carportInitial(self, timePlan, carId):
        if timePlan not in self.carport.keys():
            self.carport[timePlan] = [carId]
        else:
            self.carport[timePlan].append(carId)
        self.carportCarNum += 1

    #
    # show statistic parameters
    #
    def __id__(self):
        return self.id_

    def __roadIds__(self):
        return self.roadIds

    def __providerDirection__(self):
        return self.providerDirection

    def __receiverDirection__(self):
        return self.receiverDirection

    def __validRoadDirection__(self):
        return self.validRoadDirection

    def __provider__(self):
        return self.provider

    def __receiver__(self):
        return self.receiver

    def __validRoad__(self):
        return self.validRoad

    def __x__(self):
        return self.x

    def __y__(self):
        return self.y

    def __mapX__(self):
        return self.mapX

    def __mapY__(self):
        return self.mapY

    def __done__(self):
        return self.done

    #
    # show dynamic parameters
    #
    def __carportCarNum__(self):
        return self.carportCarNum

    def __finishCarNum__(self):
        return self.finishCarNum

    def __update__(self):
        return self.update

    #
    # show some important info
    #
    def __loc__(self):
        return self.x, self.y

    def __mapLoc__(self):
        return self.mapX, self.mapY


# 模拟器
class simulation(object):
    def __init__(self):
        self.dead = False

    def step(self):
        #print("time:%d" % TIME[0])
        for crossId in CROSSNAMESPACE:
            CROSSDICT[crossId].setDone(False)
        # print("pre-movement...")
        for road in ROADNAMESPACE:
            ROADDICT[road].stepInit()
        # print("while loop...")
        unfinishedCross = CROSSNAMESPACE
        while unfinishedCross.__len__() > 0:
            self.dead = True
            nextCross = []
            for crossId in unfinishedCross:
                cross = CROSSDICT[crossId]
                cross.step()
                if not cross.__done__():
                    nextCross.append(crossId)
                if cross.__update__() or cross.__done__():
                    # if TIME[0] == 55:
                    #     print(crossId, cross.__update__(), cross.__done__())
                    self.dead = False
            unfinishedCross = nextCross
            # if TIME[0] == 55:
            #     print(unfinishedCross)
            assert self.dead is False, print("dead lock in", unfinishedCross)
        # print("car pulling away from carport")
        for i in range(CROSSNAMESPACE.__len__()):
            crossId = CROSSNAMESPACE[i]
            for roadId in CROSSDICT[crossId].__validRoad__():
                ROADDICT[roadId].setBucket(crossId)
            #print(crossId, CROSSDICT[crossId].carport)
            CROSSDICT[crossId].outOfCarport()

    # def simulate(self):
    #     # visualize = visualization()
    #     # visualize.crossLocGen()
    #
    #     getroadinfor = getroadinformation()
    #
    #     while True:
    #         self.step()


# 获取路况的实时信息
class getroadinformation(object):
    def __init__(self):
        self.crossRadius = 14
        self.crossDistance = 150
        self.channelWidth = 5
        self.channelDistance = 3

    def getinfor(self):
        # get road info
        roadInfor = {}  # {roadId:[road_from_id,road_to_id,car_num_from_min_to_max/road.__singleCarCapcity__(),
        # car_num_from_max_to_min/road.__singleCarCapcity__()]}
        car_infor = {}
        for roadId in ROADNAMESPACE:
            road = ROADDICT[roadId]

            road_from_id = road.__from__()
            road_to_id = road.__to__()
            length = road.__length__()
            channel = road.__channel__()
            fromX, fromY = CROSSDICT[road.__from__()].__mapLoc__()
            toX, toY = CROSSDICT[road.__to__()].__mapLoc__()

            carNum2 = -1  # 当容量为负数时表示这个方向不通
            carNum1, car_infor = self.getBucket(road, 'forward', length, channel, fromX, fromY, toX, toY, road_from_id,
                                                road_to_id, car_infor)  # forward 表示直行

            if road.__isDuplex__():
                carNum2, car_infor = self.getBucket(road, 'backward', length, channel, fromX, fromY, toX, toY,
                                                    road_to_id, road_from_id, car_infor)

            roadInfor[roadId] = [road_from_id, road_to_id, carNum1 / road.__singleCarCapcity__(),
                                 carNum2 / road.__singleCarCapcity__()]
        return roadInfor, car_infor

    def getBucket(self, road, lane, length, channel, fromX, fromY, toX, toY, road_from_id, road_to_id, car_infor):
        bucket = road.__forwardBucket__() if lane != 'backward' else road.__backwardBucket__()

        tempCarNum = 0
        for i in range(length):
            for j in range(channel):
                if bucket[i][j] is not None:
                    tempCarNum = tempCarNum + 1
                    car_id = CARDICT[bucket[i][j]].__id__()
                    car_state = CARDICT[bucket[i][j]].__state__()
                    car_infor[car_id] = [car_state]

        return tempCarNum, car_infor  # , flag

if __name__ == "__main__":
    main()


