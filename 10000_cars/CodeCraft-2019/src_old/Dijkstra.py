
def dijkstra(graph,src):
    if graph is None:
        print('2')
        return None
    nodes = [i for i in range(len(graph))]
    visited=[]
    if src in nodes:
        visited.append(src)
        nodes.remove(src)
    else:
        print('1')
        return None
    distance={src:0}
    for i in nodes:
        distance[i]=graph[src][i]
    # print(distance)
    path={src:{src:[]}}
    k=pre=src
    while nodes:
        mid_distance=float('inf')
        for v in visited:
            for d in nodes:
                new_distance = graph[src][v]+graph[v][d]
                if new_distance < mid_distance:
                    mid_distance=new_distance
                    #graph[src][d]=new_distance
                    k=d
                    pre=v
        graph[src][k]=mid_distance
        distance[k]=mid_distance
        path[src][k]=[i for i in path[src][pre]]
        path[src][k].append(k)
        visited.append(k)
        nodes.remove(k)
        #print(visited,nodes)
    return distance,path


if __name__ == '__main__':
    graph_list = [ [0, 2, 1, 4, 5, 1],
            [1, 0, 4, 2, 3, 4],
            [2, 1, 0, 1, 2, 4],
            [3, 5, 2, 0, 3, 3],
            [2, 4, 3, 4, 0, 1],
            [3, 4, 7, 3, 1, 0]]
    inf = float('inf')

    #for i,d in enumerate(graph_list):
        #print(i,d)

    distance,path= dijkstra(graph_list, 0)
    #distance
    print(distance,path)