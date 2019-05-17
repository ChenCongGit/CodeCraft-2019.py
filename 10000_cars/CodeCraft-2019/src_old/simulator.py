import numpy as np
#from PIL import Image, ImageDraw, ImageFont
from Dijkstra import dijkstra
#from .Discriminator import realtimetraffic,CAR,ROAD,CROSS
import copy
import logging
import sys
import time



"""
logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')

"""

M_1 = 0.5
M_2 = 1.5
M_3 = 1.7
S_1 = 2
S_2 = 3

np.random.seed(951105)

TIME = [0]
CARDISTRIBUTION = [0, 0, 0]  # 表示在车库中的车的数量、在路上的车的数量以及调度结束到达指定id的车的信息
CARNAMESPACE, ROADNAMESPACE, CROSSNAMESPACE = [], [], []
CROSSDICT, CARDICT, ROADDICT = {}, {}, {}

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


# 构建道路地图
def build_graph(roads, road_id_list, crosses):
    '''
    读入路口和道路的信息，生成邻接图
    通常的邻接图路径表示两点间的距离，而在这个任务中，需要考虑道路限速、车速等，所以可以将两点间的距离替换为该车通过需要的时间
    '''
    # print('road txt:',roads)
    # print('cross txt:',crosses)

    cross_num = len(crosses)
    inf = float('inf')
    graph = [[inf for i in range(cross_num)] for i in range(cross_num)]
    # print(graph)
    # print(cross_num)
    # print(len(graph))

    for i in range(cross_num):
        cross = crosses[i]
        graph[cross[0] - 1][cross[0] - 1] = 0  # 自己到自己的距离为0

        for j in range(1, 5):  # 1,2,3,4 循环
            if cross[j] != -1:
                road_id = cross[j]
                road = get_road(road_id, roads, road_id_list)  # road为roads中的某一行
                distance = road[1]
                start_cross = road[4]
                end_cross = road[5]
                # print('start:',start_cross,' end:',end_cross)
                if road[6] == 1:  # 如果是双向的,则路的起点和终点间的距离一样
                    graph[start_cross - 1][end_cross - 1] = graph[end_cross - 1][start_cross - 1] = distance
                else:  # 否则终点到起点的距离为inf
                    graph[start_cross - 1][end_cross - 1] = distance

    return graph


# get_road是否可以优化，去掉每一次的for循环
def get_road(road_id, roads, road_id_list):
    # road_id_list = []
    # for road in roads:
    # road_id_list.append(road[0])
    road_index = road_id_list.index(road_id)
    return roads[road_index]


def cal_min_dist_path(crosses, distance_graph):
    '''
    利用dijkstra算法计算各点间的最终路径，保存在字典中，编号从0－（路口数－1）
    '''
    min_dist_dict = {}
    for i in range(len(crosses)):
        graph_temp = copy.deepcopy(distance_graph)
        dist = {}
        start_cross = i
        distance, path = dijkstra(graph_temp, start_cross)  # 调用后graph_temp会发生改变

        for key, value in path[i].items():  # 这儿的path[i].items()可以改为path.items()因为就只有键i的值
            mes = {}
            mes['path'] = value
            mes['distance'] = distance[key]
            dist[key] = mes
        min_dist_dict[start_cross] = dist

    # print(min_dist_dict)
    return min_dist_dict


# 路径转换解码非常慢，经过了两次循环
def trans_path(car_from, min_dist_path, roads):
    path = []
    start = car_from
    for i in range(len(min_dist_path)):
        end = min_dist_path[i]
        # print('start cross:',start,' end cross:',end)

        for road in roads:  # roads#(id,length,speed,channel,from,to,isDuplex)
            if (road[4] == start + 1 and road[5] == end + 1) or (road[5] == start + 1 and road[4] == end + 1):
                path.append(road[0])
        start = end

    return path


# 初赛的时候首先对道路的最低限速进行一个一个大数据的统计，本次练习赛中的道路速度最低为4


def getroute(graph, car, source, target, roads):
    for i in range(len(roads)):
        if roads[i][6] == 1:
            if car[3] == roads[i][2]:
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_1
                graph[roads[i][5] - 1][roads[i][4] - 1] *= M_1
            elif (S_2 > car[3] - roads[i][2] >= S_1) or (S_2 > roads[i][2] - car[3] >= S_1):
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_2
                graph[roads[i][5] - 1][roads[i][4] - 1] *= M_2
            else:
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_3
                graph[roads[i][5] - 1][roads[i][4] - 1] *= M_3

        else:
            if car[3] == roads[i][2]:
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_1
            elif (S_2 > car[3] - roads[i][2] >= S_1) or (S_2 > roads[i][2] - car[3] >= S_1):
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_2
            else:
                graph[roads[i][4] - 1][roads[i][5] - 1] *= M_3

    # print(len(graph))
    distance, path = dijkstra(graph, source)
    dist = {}
    for key, value in path.items():
        pd = {}
        for k, v in value.items():
            pd[k] = {}
            pd[k]['path'] = v
            pd[k]['distance'] = distance[k]
        dist[key] = pd

    return dist[source][target]['path'], dist[source][target]['distance']


def schedule_time_node(init_num, ALL_id, current_time, time_interval, None_id,
                       No_schedul_car, ING_car_id, answer_path, roads):
    """
    实现每个时间节点的开始对哪些车辆进行统一安排发车
    :param None_id: 没有车辆的路口id记录
    :param No_schedul_car: 没有安排调度的车辆情况记录[car_from,car_to,car[-1],predict_time,min_dist_path]
    :param ING_car_id:正在行驶的车辆的id记录
    :return:
    """
    num = 0
    temp_No_schedul_car = copy.deepcopy(No_schedul_car)
    temp_YOU_id = []
    for key, value in No_schedul_car.items():
        if value[2] > current_time:
            continue
        car_from = value[0]
        # car_to = value[1]
        # print(min_dist_dict[car_from-1])
        temp_No_schedul_car.pop(key)
        ING_car_id[key] = current_time + value[3]
        # min_dist_path = min_dist_dict[car_from-1][car_to-1]['path']
        min_dist_path = No_schedul_car[key][-1]
        min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)

        CARDICT[key].simulateInit(current_time, min_dist_path_by_road)
        CROSSDICT[CARDICT[key].__from__()].carportInitial(CARDICT[key].__planTime__(), key)

        # for i in range(len(value[-1])):
        # temp_YOU_id.append(value[-1][i])
        temp_YOU_id = temp_YOU_id + value[-1]
        with open(answer_path, 'a') as wf:
            wf.write('(' + str(key) + ',')  # 写入car id
            wf.write(str(current_time) + ',')  # 写入实际出发时间
            for road_id in range(len(min_dist_path_by_road) - 1):  # 写入行驶路径
                wf.write(str(min_dist_path_by_road[road_id]) + ',')
            wf.write(str(min_dist_path_by_road[-1]) + ')\r\n')

        num = num + 1
        if num >= init_num:
            break

    No_schedul_car = temp_No_schedul_car
    YOU_id = list(set(ALL_id).difference(set(None_id)))  # 被占用的
    YOU_id = list(set(YOU_id).union(set(temp_YOU_id)))  # 添加新安排的车辆后被占用的道路口id
    None_id = list(set(ALL_id).difference(set(YOU_id)))  # 更新的无占用路口id

    return None_id, No_schedul_car, ING_car_id


def schedule_time_slice(ALL_car, ALL_id, current_time, time_interval, None_id, No_schedul_car, ING_car_id,
                        answer_path, roads, base_priority=0.6):
    """
    实现每个时间片段内的车辆的插入
    :param time_interval: 每个时间片段的长度-1
    :param None_id: 没有车辆的路口id记录
    :param No_schedul_car: 没有安排调度的车辆情况记录[car_from,car_to,car[-1],predict_time,min_dist_path]
    :param ING_car_id:正在行驶的车辆的id记录
    :return:返回本时间片段内安排的车辆情况
    """
    #flag=0
    for i in range(time_interval):  # range(time_interval):
        current_time = current_time + i
        TIME[0]=current_time
        roadInfor,carInfor = realtimetraffic()
        print("roadInfor", roadInfor)
        print("carInfor",carInfor)
        if i not in [2, 3, 4, 6]:
            continue

        # 每隔 1 检查是否有车已经到达目的地，然后None_id更新时用到达车辆的路段id与None_id取并集：list(set(a).union(set(b)))

        temp_ING_car_id = copy.deepcopy(ING_car_id)
        for key_id, value_time in ING_car_id.items():
            if current_time >= value_time:  # 表明已经到达指定地点
                temp_ING_car_id.pop(key_id)
                None_id = list(set(None_id).union(set(ALL_car[key_id][-1])))

        ING_car_id = temp_ING_car_id

        for key, value in No_schedul_car.items():
            if value[2]>current_time:
                continue
            priority = len(list(set(None_id).intersection(set(value[-1])))) / len(value[-1])

            if priority >= base_priority:  # and #value[3] <= 1.5 * (time_interval):
                temp_No_schedul_car0 = copy.deepcopy(No_schedul_car)
                # temp_No_schedul_car1 = copy.deepcopy(No_schedul_car)
                YOU_id = list(set(ALL_id).difference(set(None_id)))  # 被占用的
                YOU_id = list(set(YOU_id).union(set(value[-1])))  # 添加新安排的车辆后被占用的道路口id
                None_id = list(set(ALL_id).difference(set(YOU_id)))  # 更新的无占用路口id
                # temp_No_schedul_car1.pop(key)
                min_dist_path = temp_No_schedul_car0[key][-1]
                temp_No_schedul_car0.pop(key)
                No_schedul_car = temp_No_schedul_car0
                # No_schedul_car = temp_No_schedul_car1
                ING_car_id[key] = current_time + value[3]
                car_from = value[0]
                # car_to = value[1]
                # print(min_dist_dict[car_from-1])

                # min_dist_path = min_dist_dict[car_from - 1][car_to - 1]['path']

                min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)  # 将路口id转换为道路id



                CARDICT[key].simulateInit(current_time, min_dist_path_by_road)
                CROSSDICT[CARDICT[key].__from__()].carportInitial(CARDICT[key].__planTime__(), key)





                with open(answer_path, 'a') as wf:
                    wf.write('(' + str(key) + ',')  # 写入car id
                    wf.write(str(current_time) + ',')  # 写入实际出发时间
                    for road_id in range(len(min_dist_path_by_road) - 1):  # 写入行驶路径
                        wf.write(str(min_dist_path_by_road[road_id]) + ',')

                    wf.write(str(min_dist_path_by_road[-1]) + ')\r\n')
                    # break

    return None_id, No_schedul_car, ING_car_id


def schedule(initNUM,distance_graph, ALL_id, cars, roads, answer_path, init_num,
             time_interval, ROAD_SPEED):
    # 得到车辆出发时间
    # None_id=[] #没有车的路口id
    YOU_id = []  # 有车的路口id
    # weighted_dist_path={}#加权后的每一辆车的路径信息，{车的id：[路径，距离]}
    No_schedul_car = {}  # {id: [from，to, planetime, 车的最坏运行时间, [车的路径]]}#没有安排的车的信息
    ALL_car = {}
    ING_car_id = {}  # 表示正在道路上运行的车辆的id及对应的到达时间,是动态变化的，到达时间的计算：当前时间+行驶花费时间
    car_start_time_list = []
    for car in cars:
        car_start_time_list.append(car[-1])

    # 按出发时间排序
    sorted_car = np.argsort(car_start_time_list)  # 返回的是列表中数值从小到大排列的索引
    # for index in sorted_car:
    # print(car_start_time_list[index])

    # ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    #####*****************begain Initialization******************


    current_time = 1

    """初始化没有调度的车辆记录"""
    # 因为前100辆车在初始化的时候全部被调度了，故而这儿不需要保留，直接从第二个时间片段开始调度,往前一个时间片段中插入

    #starttime1 = time.time()
    for i in range(initNUM, len(cars)):
        car = cars[sorted_car[i]]  # 读取到car的一整行数据#(id,from,to,speed,planTime)
        car_id = car[0]
        car_from = car[1]
        car_to = car[2]

        min_dist_path, min_dist_distance = getroute(copy.deepcopy(distance_graph), car, car_from - 1, car_to - 1, roads)
        # weighted_dist_path[car_id]=[min_dist_path,min_dist_distance]
        # print(min_dist_dict[car_from-1])
        # min_dist_path = min_dist_dict[car_from-1][car_to-1]['path']
        # min_dist_distance=min_dist_dict[car_from-1][car_to-1]['distance']
        predict_time = min_dist_distance / min(car[3], ROAD_SPEED)
        No_schedul_car[car_id] = [car_from, car_to, car[-1], predict_time, min_dist_path]
        ALL_car[car_id] = [car_from, car_to, car[-1], predict_time, min_dist_path]

    # ALL_car=copy.deepcopy(No_schedul_car)#这儿只能做深拷贝不然会改变No_schedul_car的值,但是深拷贝太耗时
    #print("未调度车辆路线计算", time.time() - starttime1)

    #starttime2 = time.time()
    """初始化路上车辆id记录和没有车的路口id"""
    for i in range(0, initNUM):
        car = cars[sorted_car[i]]  # 读取到car的一整行数据#(id,from,to,speed,planTime)
        car_id = car[0]
        car_from = car[1]
        car_to = car[2]

        min_dist_path, min_dist_distance = getroute(copy.deepcopy(distance_graph), car, car_from - 1, car_to - 1, roads)

        # print(min_dist_dict[car_from-1])
        # min_dist_path = min_dist_dict[car_from-1][car_to-1]['path']
        # min_dist_distance=min_dist_dict[car_from-1][car_to-1]['distance']
        predict_time = min_dist_distance / min(car[3], ROAD_SPEED)
        ING_car_id[car_id] = current_time + predict_time
        ALL_car[car_id] = [car_from, car_to, car[-1], predict_time, min_dist_path]
        # for j in range(len(min_dist_path)):
        # YOU_id.append(min_dist_path[j])
        YOU_id = YOU_id + min_dist_path
        min_dist_path_by_road = trans_path(car_from - 1, min_dist_path, roads)  # 将路口id转换为道路id




        CARDICT[car_id].simulateInit(current_time, min_dist_path_by_road)
        CROSSDICT[CARDICT[car_id].__from__()].carportInitial(CARDICT[car_id].__planTime__(), car_id)





        # print('path by cross:',min_dist_path)
        # print('path by road:',min_dist_path_by_road)
        # print('\n')

        # 前100辆车的起始时间一般为1
        with open(answer_path, 'a') as wf:
            wf.write('(' + str(car[0]) + ',')  # 写入car id
            wf.write(str(current_time) + ',')  # 写入实际出发时间
            for road_id in range(len(min_dist_path_by_road) - 1):  # 写入行驶路径
                wf.write(str(min_dist_path_by_road[road_id]) + ',')

            wf.write(str(min_dist_path_by_road[-1]) + ')\r\n')

    #print("初始调度车辆路线计算", time.time() - starttime2)
    """有车的路口id去重"""
    # YOU_id=list(set(YOU_id))
    """获取没有车的路口id"""
    None_id = list(set(ALL_id).difference(set(YOU_id)))

    No_scheduled_num = len(cars) - init_num  # 没有调度车的总数

    while No_scheduled_num > 0:

        TIME[0]=current_time

        print("hahah,TIME[0]",TIME[0])
        print("hahah,current_time",current_time)
        # 首次进入已经有init_num的车辆被安排，故而先进行时间片段内的插入操作
        None_id, No_schedul_car, ING_car_id = schedule_time_slice(ALL_car, ALL_id, current_time, time_interval, None_id,
                                                                  No_schedul_car, ING_car_id, answer_path, roads)
        #roadInfor = realtimetraffic()
        if len(No_schedul_car) > 0:
            current_time += time_interval
            TIME[0] = current_time
            roadInfor,carInfor = realtimetraffic()
            print("roadInfor",roadInfor)
            print("carInfor",carInfor)
            None_id, No_schedul_car, ING_car_id = schedule_time_node(init_num, ALL_id, current_time, time_interval,
                                                                     None_id,
                                                                     No_schedul_car, ING_car_id, answer_path, roads)

        No_scheduled_num = len(No_schedul_car)








"""
初始时刻发车400辆车，


时间片内10s内
for i in range(10s)：
    每一秒进行模拟
    if i in [2,4,6,8]：
       #对车辆进行安排发车
       计算现在各条路的路占率
       如果该路某一个方向的车辆占有量特别大的话，直接将该方向的距离调节为无穷大
       计算更新后的地图，准备发车车辆对应的地图
       然后发车
    

"""



def main():
    #print("enen")
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        #exit(1)
    #print("lalala")
    #car_path = sys.argv[1]
    #road_path = sys.argv[2]
    #cross_path = sys.argv[3]
    #answer_path = sys.argv[4]
    starttime = time.time()
    root='/home/liushixin/AI/chendali/Chencong/huawei2019/me/SDK/SDK_python/CodeCraft-2019/config/'
    car_path = root+"car.txt"#sys.argv[1]
    road_path = root+"road.txt"#sys.argv[2]
    cross_path = root+"cross.txt"#sys.argv[3]
    answer_path = root+"answer.txt"#sys.argv[4]
    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))


    roads, cars, crosses = read_data(car_path, road_path, cross_path)

    # load .txt files
    carInfo = open(car_path, 'r').read().split('\n')[1:]
    roadInfo = open(road_path, 'r').read().split('\n')[1:]
    crossInfo = open(cross_path, 'r').read().split('\n')[1:]
    #answerInfo = open(answer_path, 'r').read().split('\n')
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


    road_id_list = []
    for road in roads:
        road_id_list.append(road[0])
    distance_graph = build_graph(roads,road_id_list, crosses)
    #min_dist_dict = cal_min_dist_path(crosses, distance_graph)
    #ALL_id=[]
    #for road in roads:
        #ALL_id.append(road[0])
    ALL_id=road_id_list


    # 初赛的时候首先对道路的最低限速进行一个一个大数据的统计，本次练习赛中的道路速度最低为4,ROAD_SPEED=4
    init_num=105
    time_interval=8
    initNUM = 400
    print("jinru")
    schedule(initNUM,distance_graph, ALL_id, cars, roads, answer_path,init_num,time_interval,ROAD_SPEED=5)
    print("耗时:", time.time() - starttime)
#ROAD_SPEED=5 0.8 [2,4,6,8]







#**************************************Discriminator************************************************


class CAR(object):
    def __init__(self, id_, from_, to_, speed_, planTime_):
        """"**** statistic parameters ****"""
        self.id_, self.from_, self.to_, self.speed_, self.planTime_ = id_, from_, to_, speed_, -1
        self.carColor = [int(value) for value in np.random.random_integers(0, 255, [3])]
        # **** dynamic parameters ****#
        self.state, self.x, self.y = 0, 0, 0
        self.presentRoad, self.nextCrossId = None, self.from_
        self.deltaX, self.deltaY = 0, 0
        self.wait = False
        self.route, self.routeIndex = None, None

    """仿真初始化"""

    def simulateInit(self, planTime, route):
        self.planTime_, self.route, self.routeIndex = planTime, route, 0

    """相关参数的动态更新"""

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

    """*******CAR类的属性*******"""
    """静态属性"""

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

    """用于可视化//////////////////////"""

    def __carColor__(self):
        return self.carColor

    """动态属性"""

    # 车的状态car state: 0,1,2,3 分别表示在车库中、等待、完成、结束
    def __state__(self):
        return self.state

    def __x__(self):
        return self.x

    def __y__(self):
        return self.y

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

    """属性计算"""

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
        else:
            print("Keywords mistake in CAR.chooseAbsoluteBucket()")

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
        if self.provideBucket is None:
            print("Please do CAR.setBucket() first!")
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
        if self.provideBucket is None:
            print("Please do CAR.setBucket() first!")
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
        if self.receiveBucket is None:
            print("Please do CAR.setBucket() first!")
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

    """*******ROAD类的属性*******"""

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
        self.directionMap = {north_: {east_: 1, south_: 2, west_: -1},east_: {south_: 1, west_: 2, north_: -1},south_: {west_: 1, north_: 2, east_: -1},west_: {north_: 1, east_: 2, south_: -1}}
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
            if roadId not in self.receiver:
                print("Car(%d).Road(%d) not in cross(%d).function:class.outOfCarport" % (carId, roadId, self.id_))
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
        print("time:%d" % TIME[0])
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
                    if TIME[0] == 55:
                        print(crossId, cross.__update__(), cross.__done__())
                    self.dead = False
            unfinishedCross = nextCross
            if TIME[0] == 55:
                print(unfinishedCross)
            assert self.dead is False, print("dead lock in", unfinishedCross)
        # print("car pulling away from carport")
        for i in range(CROSSNAMESPACE.__len__()):
            crossId = CROSSNAMESPACE[i]
            for roadId in CROSSDICT[crossId].__validRoad__():
                ROADDICT[roadId].setBucket(crossId)
            CROSSDICT[crossId].outOfCarport()

    def simulate(self):
        # visualize = visualization()
        # visualize.crossLocGen()

        getroadinfor = getroadinformation()

        while True:
            self.step()
            # 可视化
            # visualize.drawMap()
            # CARDISTRIBUTION[0]为没有调度的车的总数，CARDISTRIBUTION[1]在路上的车的总数，CARDISTRIBUTION[2]调度结束的车的总数
            print("CARDISTRIBUTION[2]", CARDISTRIBUTION[2])
            # 被全部调度完之后停止
            if CARDISTRIBUTION[2] == CARNAMESPACE.__len__():
                # print("CARDISTRIBUTION[2]",CARDISTRIBUTION[2])
                break
            if self.dead:
                break
            TIME[0] += 1


# 获取路况的实时信息
class getroadinformation(object):
    def __init__(self):
        # self.maxX, self.maxY = 0, 0
        # self.savePath = 'C:/Users/Wenrui-GUO/Desktop/simulatePictures222'
        # ** cross param **#
        self.crossRadius = 14
        self.crossDistance = 150
        # self.crossColor = [25, 200, 0]
        # ** road param **#
        # self.roadColor = [0, 0, 0]  # black
        # self.roadLineType = 4
        self.channelWidth = 5
        self.channelDistance = 3
        # self.lineWidth = 2
        # self.time = 0

    def getinfor(self):
        # get road info
        roadInfor = {}  # {roadId:[road_from_id,road_to_id,car_num_from_min_to_max/road.__singleCarCapcity__(),
        # car_num_from_max_to_min/road.__singleCarCapcity__()]}
        car_infor = {}
        for roadId in ROADNAMESPACE:
            road = ROADDICT[roadId]
            # fromX, fromY = CROSSDICT[road.__from__()].__mapLoc__()
            # cross_from_id=CROSSDICT[road.__from__()].__id__()
            # toX, toY = CROSSDICT[road.__to__()].__mapLoc__()
            # cross_to_id=CROSSDICT[road.__to__()].__id__()

            """
            查看地图中road的信息，road_from_id小于road_to_id
            """
            road_from_id = road.__from__()
            road_to_id = road.__to__()
            length = road.__length__()
            channel = road.__channel__()
            fromX, fromY = CROSSDICT[road.__from__()].__mapLoc__()
            toX, toY = CROSSDICT[road.__to__()].__mapLoc__()
            # plot line
            # cv.line(img, (fromX, fromY), (toX, toY), color=self.roadColor, thickness=2)
            # plot bucket
            #
            # #flag1=-1
            # #flag2=-1
            # car_num_from_min_to_max=0
            # car_num_from_max_to_min=0
            # carNum1=-1
            carNum2 = -1  # 当容量为负数时表示这个方向不通
            carNum1, car_infor = self.getBucket(road, 'forward', length, channel, fromX, fromY, toX, toY, road_from_id,
                                                road_to_id, car_infor)  # forward 表示直行

            if road.__isDuplex__():
                carNum2, car_infor = self.getBucket(road, 'backward', length, channel, fromX, fromY, toX, toY,
                                                    road_from_id, road_to_id, car_infor)

            # if flag1==1:#从小到大
            #     car_num_from_min_to_max=car_num_from_min_to_max+carNum1
            #
            #     if flag2==1:#从小到大
            #         car_num_from_min_to_max = car_num_from_min_to_max + carNum2
            #
            #     elif flag2==2:
            #         car_num_from_max_to_min=car_num_from_max_to_min+carNum2
            #
            # elif flag1==2:
            #     car_num_from_max_to_min = car_num_from_max_to_min + carNum1
            #
            #     if flag2==1:#从小到大
            #         car_num_from_min_to_max = car_num_from_min_to_max + carNum2
            #     elif flag2 == 2:
            #         car_num_from_max_to_min = car_num_from_max_to_min + carNum2

            # roadInfor[roadId]=[road_from_id,road_to_id,car_num_from_min_to_max/road.__singleCarCapcity__(),
            #                    car_num_from_max_to_min/road.__singleCarCapcity__()]
            roadInfor[roadId] = [road_from_id, road_to_id, carNum1 / road.__singleCarCapcity__(),
                                 carNum2 / road.__singleCarCapcity__()]

            if carNum1 / road.__singleCarCapcity__() > 0.5 or carNum2 / road.__singleCarCapcity__() > 0.5:
                print("roadId", roadId)
                print("carNum1/road.__singleCarCapcity__()", carNum1 / road.__singleCarCapcity__())
                print("carNum2/road.__singleCarCapcity__()", carNum2 / road.__singleCarCapcity__())

        return roadInfor, car_infor

    def getBucket(self, road, lane, length, channel, fromX, fromY, toX, toY, road_from_id, road_to_id, car_infor):
        bucket = road.__forwardBucket__() if lane != 'backward' else road.__backwardBucket__()

        # direction = self.bucketDirection(fromX, fromY, toX, toY, lane)

        """
        方向的判断，根据可视化的结果，各路口的id存在的关系：

        右边的id大于左边的id，上边的id大于下面的id
        对于新的地图，首先应该根据可视化来判断，加入先验信息
        if direction == 'north':
            从小的cross_id 到大的cross_id
        elif direction == 'south':
            从大的cross_id 到小的cross_id
        elif direction == 'east':
            从小的cross_id 到大的cross_id
        elif direction == 'west':
            从大的cross_id 到小的cross_id
        """
        # flag=None
        # if direction == 'north':
        #     flag=1
        #     #从小的cross_id 到大的cross_id
        # elif direction == 'south':
        #     flag=2
        #     #从大的cross_id 到小的cross_id
        # elif direction == 'east':
        #     flag=1
        #     #从小的cross_id 到大的cross_id
        # elif direction == 'west':
        #     flag=2
        #     #从大的cross_id 到小的cross_id

        tempCarNum = 0
        for i in range(length):
            for j in range(channel):
                if bucket[i][j] is not None:
                    tempCarNum = tempCarNum + 1
                    car_id = CARDICT[bucket[i][j]].__id__()
                    car_state = CARDICT[bucket[i][j]].__state__()
                    car_speed = CARDICT[bucket[i][j]].__speed__()
                    car_infor[car_id] = [car_state, road_to_id, car_speed]

        return tempCarNum, car_infor  # , flag



        # def bucketDirection(self, fromX, fromY, toX, toY, lane):
        #     if fromY > toY:
        #         direction = 'north' if lane == 'forward' else 'south'
        #     elif fromY < toY:
        #         direction = 'south' if lane == 'forward' else 'north'
        #     elif fromX < toX:
        #         direction = 'east' if lane == 'forward' else 'west'
        #     else:
        #         direction = 'west' if lane == 'forward' else 'east'
        #     return direction


def realtimetraffic():
    """
    获取路况的是实时信息
    :return:roadInfor为{roadId:[road_from_id,road_to_id,car_num_from_min_to_max,car_num_from_max_to_min]}
            carInfor为[car_state,road_to_id,car_speed]
    """
    sim = simulation()
    getroadinfor = getroadinformation()
    sim.step()
    roadInfor,carInfor = getroadinfor.getinfor()
    return roadInfor,carInfor


if __name__ == "__main__":
    main()