# author: Krupal Shah

#Libraries 
import signal
from collections import defaultdict
import re
import copy
import heapq
import sys

# The Class graph is defined in orde to save details of each node in the graph. 
# The lt function is created in order to overwrite the capabilities of the original 'less than', so that the heapify pop function would return
# the node with the least cost. This class is used for the implementation of AStar and RBFS algorithm
# State: The current state of the node
# Parent: Node's parent
# Child: Node's child
# Move: Rotation done to reach the current node
# Cost: The cost (path_cost + heuristic cost)
# Path_cost: The path cost for the node

class Graph:
    def __init__(self,state, parent, child, move, cost, path_cost):
        self.state = state
        self.parent = parent
        self.child = child
        self.move = move
        self.cost = cost
        self.path_cost = path_cost

    def __lt__(self,other):
        return self.cost < other.cost

# The Class Graph_BFS is used for the implementation of BFS Algorithm

class Graph_BFS:
    def __init__(self, parent, child, move):
        self.parent = parent
        self.child = child
        self.move = move

# The heuristic function calculates the minimum number of tile movement required for each source to reach its respective destination.
# It does so by finding the index difference if the souce and destination are on the same axis, else it will find a common intersection point
# and then calculating the number of tile movements.
def heuristic(source, destination): 
    equator = [(90, 0), (90, 30), (90, 60), (90, 90), (90, 120), (90, 150), (90, 180), (90, 210),(90, 240), (90, 270), (90, 300), (90, 330)]
    longitude_180 = [(0, 0), (30, 0), (60, 0), (90, 0), (120, 0), (150, 0), (180, 180), (150, 180), (120, 180), (90, 180), (60, 180), (30, 180)]
    longitude_270 = [(0, 0), (30, 90), (60, 90), (90, 90), (120, 90), (150, 90), (180, 180),(150, 270), (120, 270), (90, 270), (60, 270), (30, 270)]
    i = 0
    final_moves = 0
    length = len(source)
    for i in range(length):
        num_of_moves = 0
        if source[i] != destination[i]:
            # If the source and destination is in equator tile
            if source[i] in equator and destination[i] in equator:
                num_of_moves = abs(equator.index(source[i]) - equator.index(destination[i]))
                if num_of_moves > len(equator)/2:
                    num_of_moves = len(equator) - num_of_moves
                final_moves += int(num_of_moves)
                continue

            # If source and destination is in longitude (0/180)

            if source[i] in longitude_180 and destination[i] in longitude_180:
                # If destination in longitude(0/180)
                num_of_moves = abs(longitude_180.index(source[i]) - longitude_180.index(destination[i]))
                if num_of_moves > len(longitude_180)/2:
                    num_of_moves = len(longitude_180) - num_of_moves
                final_moves += int(num_of_moves)
                continue
            #If source destination is in longitude(90/270)
            if source[i] in longitude_270 and destination[i] in longitude_270:
                num_of_moves = abs(longitude_270.index(source[i]) - longitude_270.index(destination[i]))
                if num_of_moves > len(longitude_270)/2:
                    num_of_moves = len(longitude_270) - num_of_moves
                final_moves += int(num_of_moves)
                continue

             # If source in equator and destination in longitude(0/180)   
            if source[i] in equator and destination[i] in longitude_180:
                temp_move_0 = abs(equator.index((90,0)) - equator.index(source[i]))
                if temp_move_0 > len(longitude_270)/2:
                    temp_move_0 = len(longitude_270) - temp_move_0
                temp_move_180 = abs(equator.index((90,180)) - equator.index(source[i]))
                if temp_move_180 > len(longitude_270)/2:
                    temp_move_180 = len(longitude_270) - temp_move_180
                num_of_moves_1 = abs(longitude_180.index((90,0)) - longitude_180.index(destination[i]))
                if num_of_moves_1 > len(longitude_180)/2:
                    num_of_moves_1 = len(longitude_180) - num_of_moves_1
                num_of_moves_1 += temp_move_0
                num_of_moves_2 = abs(longitude_180.index((90,180)) - longitude_180.index(destination[i]))
                if num_of_moves_2 > len(longitude_180)/2:
                    num_of_moves_2 = len(longitude_180) - num_of_moves_2
                num_of_moves_2 += temp_move_180
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
            
            # If source in equator and destination in longitude 270
            if source[i] in equator and destination[i] in longitude_270:
                temp_move_90 = abs(equator.index((90,90)) - equator.index(source[i]))
                if temp_move_90 > len(longitude_270)/2:
                    temp_move_90 = len(longitude_270) - temp_move_90
                temp_move_270 = abs(equator.index((90,270)) - equator.index(source[i]))
                if temp_move_270 > len(longitude_270)/2:
                    temp_move_270 = len(longitude_270) - temp_move_270

                num_of_moves_1 = abs(longitude_270.index((90,90)) - longitude_270.index(destination[i]))
                if num_of_moves_1 > len(longitude_270)/2:
                    num_of_moves_1 = len(longitude_270) - num_of_moves_1
                num_of_moves_1 += temp_move_90
            
                num_of_moves_2 = abs(longitude_270.index((90,270)) - longitude_270.index(destination[i]))
                if num_of_moves_2 > len(longitude_270)/2:
                    num_of_moves_2 = len(longitude_270) - num_of_moves_2
                num_of_moves_2 += temp_move_270
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
                # If source is in longitude(0/180) and destination in equator
            if source[i] in longitude_180 and destination[i] in equator:
                temp_90 = abs(longitude_180.index((90,0)) - longitude_180.index(source[i]))
                if temp_90 > len(longitude_270)/2:
                    temp_90 = len(longitude_270) - temp_90
                temp_180 = abs(longitude_180.index((90,180)) - longitude_180.index(source[i]))
                if temp_180 > len(longitude_270)/2:
                    temp_180 = len(longitude_270) - temp_180
                num_of_moves_1 = abs(equator.index((90,0)) - equator.index(destination[i]))
                if num_of_moves_1 > len(longitude_180)/2:
                    num_of_moves_1 = len(longitude_180) - num_of_moves_1
                num_of_moves_1 += temp_90
                num_of_moves_2 = abs(equator.index((90,180)) - equator.index(destination[i]))
                if num_of_moves_2 > len(longitude_180)/2:
                    num_of_moves_2 = len(longitude_180) - num_of_moves_2
                num_of_moves_2 += temp_180
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
            # If source in longitude_180 and destination in longitude_270
            if source[i] in longitude_180 and destination[i] in longitude_270:
                temp_0 = abs(longitude_180.index((0,0)) - longitude_180.index(source[i]))
                if temp_0 > len(longitude_270)/2:
                    temp_0 = len(longitude_270) - temp_0
                temp_180 = abs(longitude_180.index((180,180)) - longitude_180.index(source[i]))
                if temp_180 > len(longitude_270)/2:
                    temp_180 = len(longitude_270) - temp_180
                num_of_moves_1 = abs(longitude_270.index((0,0)) - longitude_270.index(destination[i]))
                if num_of_moves_1 > (len(longitude_270)/2):
                    num_of_moves_1 = len(longitude_270) - num_of_moves_1
                num_of_moves_1 = temp_0 + num_of_moves_1
                num_of_moves_2 = abs(longitude_270.index((180,180)) - longitude_270.index(destination[i]))
                if num_of_moves_2 > len(longitude_270)/2:
                    num_of_moves_2 = len(longitude_270) - num_of_moves_2
                num_of_moves_2 = temp_180 + num_of_moves_2
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
                # If source in longitude_270 and destination in equator
            if source[i] in longitude_270 and destination[i] in equator:
                temp_0 = abs(longitude_270.index((90,270)) - longitude_270.index(source[i]))
                if temp_0 > len(longitude_270)/2:
                    temp_0 = len(longitude_270) - temp_0
                temp_180 = abs(longitude_270.index((90,90)) - longitude_270.index(source[i]))
                if temp_180 > len(longitude_270)/2:
                    temp_180 = len(longitude_270) - temp_180
                num_of_moves_1 = abs(equator.index((90,270)) - equator.index(destination[i]))
                if num_of_moves_1 > len(longitude_180)/2:
                    num_of_moves_1 = len(longitude_180) - num_of_moves_1
                num_of_moves_1 += temp_0
                num_of_moves_2 = abs(equator.index((90,90)) - equator.index(destination[i]))
                if num_of_moves_2 > len(longitude_180)/2:
                    num_of_moves_2 = len(longitude_180) - num_of_moves_2
                num_of_moves_2 += temp_180
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
                #If source in longitude 270 and destination in longitude 180
            if source[i] in longitude_270 and destination[i] in longitude_180:
                temp_0 = abs(longitude_270.index((0,0)) - longitude_270.index(source[i]))
                if temp_0 > len(longitude_270)/2:
                    temp_0 = len(longitude_270) - temp_0
                temp_180 = abs(longitude_270.index((180,180)) - longitude_270.index(source[i]))
                if temp_180 > len(longitude_270)/2:
                    temp_180 = len(longitude_270) - temp_180
                num_of_moves_1 = abs(longitude_180.index((0,0)) - longitude_180.index(destination[i]))
                if num_of_moves_1 > len(longitude_180)/2:
                    num_of_moves_1 = len(longitude_180) - num_of_moves_1
                num_of_moves_1 += temp_0
                num_of_moves_2 = abs(longitude_180.index((180,180)) - longitude_180.index(destination[i]))
                if num_of_moves_2 > len(longitude_180)/2:
                    num_of_moves_2 = len(longitude_180) - num_of_moves_2
                num_of_moves_2 += temp_180
                final_moves += int(min(num_of_moves_1,num_of_moves_2))
                continue
    return (final_moves/8)
            
# The final heuristic cost returned is divided by 8 because as the algorithm goes deeper into the graph, the heuristic cost would start 
# dominating and path cost will have no effect.         


# The below set of functions determine the changes in the cooridinates of the source after making the specific rotation
# find_next_equator: It will return the coordinates after forward rotating the equator axis
# find_prev_equator: It will return the coordinates after backward rotating the equator axis
# find_next_longitude_180: It will return the coordinates after forward rotating the longitude(0/180) axis
# find_prev_longitude_180: It will return the coordinates after backward rotating the longitude(0/180) axis
# find_next_longitude_270: It will return the coordinates after forward rotating the longitude(90/270) axis
# find_prev_longitude_270: It will return the coordinates after backward rotating the longitude(90/270) axis

def find_next_equator(current1):
    equator = [(90, 0), (90, 30), (90, 60), (90, 90), (90, 120), (90, 150), (90, 180), (90, 210),
                (90, 240), (90, 270), (90, 300), (90, 330)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in equator:
            index = equator.index(current[i])
            if index == len(equator)-1:
                current[i] = equator[0]
            else:
                current[i] = equator[index+1]
        i += 1

    return current

def find_prev_equator(current1):
    equator = [(90, 0), (90, 30), (90, 60), (90, 90), (90, 120), (90, 150), (90, 180), (90, 210),
                (90, 240), (90, 270), (90, 300), (90, 330)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in equator:
            # print(current[i])
            index = equator.index(current[i])
            if index == 0:
                current[i] = equator[(len(equator)-1)]
            else:
                current[i] = equator[index-1]
        i += 1
    return current

def find_next_longitude_180(current1):
    longitude_180 = [(0, 0), (30, 0), (60, 0), (90, 0), (120, 0), (150, 0), (180, 180), (150, 180), (120, 180), (90, 180), (60, 180), (30, 180)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in longitude_180:
            index = longitude_180.index(current[i])
            if index == len(longitude_180)-1:
                current[i] = longitude_180[0]
            else:
                current[i] = longitude_180[index+1]
        i += 1

    return current

def find_prev_longitude_180(current1):
    longitude_180 = [(0, 0), (30, 0), (60, 0), (90, 0), (120, 0), (150, 0), (180, 180), (150,
                                                                                            180), (120, 180), (90, 180), (60, 180), (30, 180)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in longitude_180:
            index = longitude_180.index(current[i])
            if index == len(longitude_180)-1:
                current[i] = longitude_180[index-1]
            else: 
                current[i] = longitude_180[index-1]
        i += 1
    return current

def find_next_longitude_270(current1):
    longitude_270 = [(0, 0), (30, 90), (60, 90), (90, 90), (120, 90), (150, 90), (180, 180),
                        (150, 270), (120, 270), (90, 270), (60, 270), (30, 270)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in longitude_270:
            index = longitude_270.index(current[i])
            if index == len(longitude_270)-1:
                current[i] = longitude_270[0]
            else:
                current[i] = longitude_270[index+1]
        i += 1

    return current

def find_prev_longitude_270(current1):
    longitude_270 = [(0, 0), (30, 90), (60, 90), (90, 90), (120, 90), (150, 90), (180, 180),
                        (150, 270), (120, 270), (90, 270), (60, 270), (30, 270)]
    i = 0
    current = copy.deepcopy(current1)
    while i < len(current):
        if current[i] in longitude_270:
            index = longitude_270.index(current[i])
            if index == len(longitude_270)-1:
                current[i] = longitude_270[index-1]
            else:
                current[i] = longitude_270[index-1]
        i += 1

    return current

 # The below function is for Recursive Best first Search. The function will call RBFS function which is its child function to implement
 # the search.   

def Recursive_Best_First_Search(source,destination):
    states = 0
    max_queue = 0
    return RBFS(Graph(source , None, None, None, heuristic(source, destination),0), destination, 10000000000000, states, max_queue)
def RBFS(node, destination, f_limit, states, max_queue):
    G = []
    found = 0
    current_node = node.state
    # If goal is found, then print the path, path length, states, and max_queue length
    if current_node == destination:
        cost = node.cost
        found = 1
        print("Found")
        final_path = []
        while node.parent != None:
            final_path.append(node.move)
            node = node.parent
        print("Path is: ",final_path)
        print("Path length: ",len(final_path))
        print("Number of states: ",states)
        print("Max Queue Length: ",max_queue)
        return found, cost
    else:
        # Call respective functions in order to find out the child node for the respective parent node and update their cost.
        next_equator = find_next_equator(current_node)
        G.append(Graph(next_equator, node,next_equator,'inc_equator', max(node.path_cost + 1 + heuristic(next_equator,destination),node.cost),node.path_cost + 1))
        states += 1

        prev_equator = find_prev_equator(current_node)
        G.append(Graph(prev_equator, node,prev_equator,'dec_equator',max(node.path_cost + 1 + heuristic(prev_equator, destination),node.cost),node.path_cost + 1))
        states += 1

        next_longitude_180 = find_next_longitude_180(current_node)
        G.append(Graph(next_longitude_180, node,next_longitude_180,'inc_longitude_180', max(node.path_cost+1+heuristic(next_longitude_180, destination),node.cost),node.path_cost+1))
        states += 1

        prev_longitude_180 = find_prev_longitude_180(current_node)
        G.append(Graph(prev_longitude_180, node,prev_longitude_180,'dec_longitude_180',max(node.path_cost+1+heuristic(prev_longitude_180, destination),node.cost),node.path_cost+1))
        states += 1

        next_longitude_270 = find_next_longitude_270(current_node)
        G.append(Graph(next_longitude_270, node,next_longitude_270,'inc_longitude_270',max(node.path_cost+1+heuristic(next_longitude_270, destination),node.cost),node.path_cost+1))
        states += 1

        prev_longitude_270 = find_prev_longitude_270(current_node)
        G.append(Graph(prev_longitude_270, node,prev_longitude_270,'dec_longitude_270',max(node.path_cost+1+heuristic(prev_longitude_270, destination),node.cost),node.path_cost+1))
        states += 1

  # The below while will run until the goal is found. It will recursively call the RBFS until the goal is reached.

    while True:
        def signal_handler(sig, frame):
            print("You pressed CTRL-C")
            print("Current Details:")
            print("States expanded:", states)
            print("Max Queue Size:", max_queue)
            sys.exit()
        signal.signal(signal.SIGINT, signal_handler)
        heapq.heapify(G)
        best_node = G.pop(0)
        heapq.heapify(G)
        alternative = G[0].cost
        max_queue = max(len(G), states)
        if best_node.cost > f_limit:
            return found,best_node.cost
        found, best_node.cost = RBFS(best_node,destination,min(alternative,f_limit),states, max_queue)
        heapq.heappush(G, best_node)
        if found != 0:
            return found, best_node.cost

# The below function is for searching the path using Breadth First Search. In this, for each parent node, it will find the childs.
# Then it will search for the goal in the child nodes. After that it will take each individual child treat as a parent and find its 
# respective childs. Also, it will keep track of redundant nodes by checking whether the child node already exist or not.

def BFS(source,destination):
    frontier = []
    G = [Graph_BFS(None,source,'None')]
    frontier.append(list(source))
    states = 0
    final_path = []
    max_queue = 0
    found = 0
    while found == 0:
        current_node = frontier.pop(0)
        queue_length = len(frontier)
        def signal_handler(sig, frame):
            print("You pressed CTRL-C")
            print("Current Details:")
            print("States expanded:", states)
            print("Max Queue Size:", queue_length)
            sys.exit()
        signal.signal(signal.SIGINT, signal_handler)
        if max_queue < queue_length:
            max_queue = queue_length
        if current_node == destination:
            found = 1
            i = len(G) - 1
            while i > 0:
                if G[i].child == destination:
                    final_path.append(G[i].move)
                    destination = G[i].parent
                i -= 1
            return states, max_queue, final_path, len(final_path)

        else:
            
            next_equator = find_next_equator(current_node)
            if next_equator not in frontier:
                G.append(Graph_BFS(current_node,next_equator,'inc_equator'))
                frontier.append(list(next_equator))
                states += 1
            prev_equator = find_prev_equator(current_node)
            if prev_equator not in frontier:
                G.append(Graph_BFS(current_node,prev_equator,'dec_equator'))
                frontier.append(list(prev_equator))
                states += 1
            next_longitude_180 = find_next_longitude_180(current_node)
            if next_longitude_180 not in frontier:
                G.append(Graph_BFS(current_node,next_longitude_180,'inc_longitude_180'))
                frontier.append(list(next_longitude_180))
                states += 1
            prev_longitude_180 = find_prev_longitude_180(current_node)
            if prev_longitude_180 not in frontier:
                G.append(Graph_BFS(current_node,prev_longitude_180,'dec_longitude_180'))
                frontier.append(list(prev_longitude_180))
                states += 1
            next_longitude_270 = find_next_longitude_270(current_node)
            if next_longitude_270 not in frontier:
                G.append(Graph_BFS(current_node,next_longitude_270,'inc_longitude_270'))
                frontier.append(list(next_longitude_270))
                states += 1
            prev_longitude_270 = find_prev_longitude_270(current_node)
            if prev_longitude_270 not in frontier:
                G.append(Graph_BFS(current_node,prev_longitude_270,'dec_longitude_270'))
                frontier.append(list(prev_longitude_270))
                states += 1

# The below will implement the searh using AStar algorithm. 
# For each parent node, it will find out the heuristic cost and add cost for each
# node as path_cost + heuristic cost for each of its child.
# Each the node with the least cost is selected and it is checked for the goal. 
# This process will continue until the goal is reached

def AStar(source,destination):
    frontier = []
    G = [Graph(source,None,None,'None',heuristic(source,destinations),0)]
    frontier.append(list(source))
    states = 0
    final_path = []
    max_queue = 0
    found = 0
    while found == 0:
        def signal_handler(sig, frame):
            print("You pressed CTRL-C")
            print("Current Details:")
            print("States expanded:", states)
            print("Max Queue Size:", max_queue)
            sys.exit()
        signal.signal(signal.SIGINT, signal_handler)
        heapq.heapify(G)
        node = G.pop(0)
        max_queue = max(max_queue, len(G))
        if node.state == destination:
            # If the goal state is found, then return the states, max_queue length, path, and path length
            found = 1
            print("Found")
            while node.parent != None:
                final_path.append(node.move)
                node = node.parent
            return states, max_queue, final_path, len(final_path)

        else:
            
            next_equator = find_next_equator(node.state)
            if next_equator not in frontier:
                G.append(Graph(next_equator,node, next_equator,'inc_equator',node.path_cost + 1 + heuristic(next_equator,destination),node.path_cost+1))
                frontier.append(list(next_equator))
                states += 1
            prev_equator = find_prev_equator(node.state)
            if prev_equator not in frontier:
                G.append(Graph(prev_equator,node, prev_equator,'dec_equator',node.path_cost + 1 + heuristic(prev_equator, destination),node.path_cost + 1))
                frontier.append(list(prev_equator))
                states += 1
            next_longitude_180 = find_next_longitude_180(node.state)
            if next_longitude_180 not in frontier:
                G.append(Graph(next_longitude_180,node, next_longitude_180,'inc_longitude_180', node.path_cost + 1 + heuristic(next_longitude_180,destination),node.path_cost+1))
                frontier.append(list(next_longitude_180))
                states += 1
            prev_longitude_180 = find_prev_longitude_180(node.state)
            if prev_longitude_180 not in frontier:
                G.append(Graph(prev_longitude_180,node,prev_longitude_180,'dec_longitude_180',node.path_cost + 1 + heuristic(prev_longitude_180,destination),node.path_cost+1))
                frontier.append(list(prev_longitude_180))
                states += 1
            next_longitude_270 = find_next_longitude_270(node.state)
            if next_longitude_270 not in frontier:
                G.append(Graph(next_longitude_270,node,next_longitude_270,'inc_longitude_270',node.path_cost + 1 + heuristic(next_longitude_270,destination),node.path_cost+1))             
                frontier.append(list(next_longitude_270))
                states += 1
            prev_longitude_270 = find_prev_longitude_270(node.state)
            if prev_longitude_270 not in frontier:
                G.append(Graph(prev_longitude_270,node,prev_longitude_270,'dec_longitude_270',node.path_cost + 1 + heuristic(prev_longitude_270,destination),node.path_cost+1))
                frontier.append(list(prev_longitude_270))
                states += 1

# In the main part the arguments are parsed and respective function is called.
    
if __name__ == '__main__':
    arguments = sys.argv[1:]
    Algorithm = arguments[0]
    File = arguments[1]
    data = []
    # Reading the input file
    input = open(File,"r")
    contents = input.readlines()
    for line in contents:
        x = line.split()
        for y in x:
            temp = re.findall(r'\d+', y)
            data.append(list(map(int, temp)))
    sources = []
    destinations = []
    data.pop(0)
    counter = 0
    while counter <= (len(data)-2):
        sources.append(tuple(data[counter+1]))
        destinations.append(tuple(data[counter+2]))
        counter += 3
    if Algorithm == "BFS":
        states, max_queue, path, length = BFS(sources, destinations)
        
        signal.pause()
        print("Total States expanded:",states)
        print("Max Queue Length:",max_queue)
        print("Path:",path)
        print("Path_length:",length)
    if Algorithm == 'AStar':
        states, max_queue, path, length = AStar(sources, destinations)
        signal.pause()
        print("Total States expanded:",states)
        print("Max Queue Length:",max_queue)
        print("Path:",path)
        print("Path_length:",length)
    if Algorithm == 'RBFS':
        found, best_cost = Recursive_Best_First_Search(sources, destinations)
        
