
import os
import sys
from collections import deque
import heapq
import math
import itertools


try: 
    import queue
except ImportError:
    import Queue as queue

class Node:
  def __init__(self,f,g,h):
    self.f = f
    self.g = g
    self.h = h
    
class Search:
    W,H,X,Y,Z,no_of_targets =0,0,0,0,0,0
    algo_index = -1
    target_array = []
    grid = []
    rows =    [-1,0,1,0,-1,-1,1,1]
    columns = [0,1,0,-1,-1,1,1,-1]
    ItemCounter = itertools.count()
    
        
    def prepare_input (self,data,search_output_file):
        output=""
        algo = data[0].replace(" ", "").upper()
        arr = list(map(int, data[1].split()))
        self.W = int(arr[0])
        self.H = int(arr[1])
        arr = list(map(int, data[2].split()))
        self.X = int(arr[0])
        self.Y = int(arr[1])
        self.Z = int(data[3])
        self.no_of_targets = int(data[4])
        if self.no_of_targets <= 0:
            search_output_file.write("FAIL")
            return
        for i in range(5,5+self.no_of_targets):
            self.target_array.append([int(s) for s in data[i].split()])
        if len(self.target_array) == 0:
            search_output_file.write("FAIL")
            return
        matrix_start = 5+self.no_of_targets
      
        for j in range(matrix_start,matrix_start + self.H):
            row_values = []
            self.grid.append([int(s) for s in data[j].split()])
        
        if algo == "A*":
            answer = self.a()
            self.printToFile(answer)
        elif algo == "UCS":
            answer = self.ucs()
            self.printToFile(answer)
        elif algo == "BFS":
            answer = self.bfs()
            self.printToFile(answer)
        else:
            search_output_file.write("FAIL")

    def printToFile(self,answer):
        for item in answer:
            output=""
            if item == "FAIL":
                search_output_file.write("FAIL")
            else:
                output = output + ' '.join([str(i[0]) + ',' + str(i[1]) for i in item])
                search_output_file.write(output)
            search_output_file.write('\n')

    def appendToHeap(self,child_node_cost,current_tuple,queue_replica,heap,parent,path_map,asearch):
        if current_tuple not in queue_replica:
            ItemCount = next(self.ItemCounter)
            if asearch==True:
                ItemEntry = [child_node_cost.f, ItemCount, current_tuple,child_node_cost.g]
            else:
                ItemEntry = [child_node_cost, ItemCount, current_tuple]
            queue_replica[current_tuple] = ItemEntry
            heapq.heappush(heap, ItemEntry)
            path_map[current_tuple] = parent
        else:
            Item = queue_replica[current_tuple]
            if asearch==True:
                existing_cost = Item[3]
                child_cost = child_node_cost.g
            else:
                existing_cost= Item[0]
                child_cost = child_node_cost
            if existing_cost > child_cost:
                ItemCount = next(self.ItemCounter)
                if asearch==True:
                    ItemEntry = [child_node_cost.f, ItemCount, current_tuple,child_node_cost.g]
                else:
                    ItemEntry = [child_node_cost, ItemCount, current_tuple]
                queue_replica[current_tuple] = ItemEntry
                heap = heapq.heapify(list(queue_replica.values()))
                path_map[current_tuple] = parent

    def popFromHeap(self,heap,queue_replica):
        if heap:
            current_cell = heapq.heappop(heap)
            del queue_replica[current_cell[2]]
            return current_cell
        else:
            return []

    def heuristic(self, start, end):
        dx = abs(start[1] - end[1])
        dy = abs(start[0] - end[0])
        dz = abs(self.grid[start[1]][start[0]] - self.grid[end[1]][end[0]])
        return round(math.sqrt(dx*dx + dy * dy + dz * dz))

        # return 10 * max(dx, dy) + (4) * min(dx, dy)


    
    def a(self):
        result =[]
        for n in range(0,self.no_of_targets):
            path_found= False
            target = tuple(self.target_array[n])
            start = (self.X,self.Y)
            ItemCount = next(self.ItemCounter)
            start_cost = Node(0,0,0)
            ItemEntry = [0, ItemCount, start,0]
            heap=[]
            path_map={}
            queue_replica={}
            explored={}
            queue_replica[start] = ItemEntry
            heapq.heappush(heap, ItemEntry)
            path_map[start]=start
            while heap:
                current_cell = self.popFromHeap(heap,queue_replica)
                explored[current_cell[2]] = current_cell[3]
                if current_cell[2][1] == target[1] and current_cell[2][0]==target[0]:
                    print(current_cell)
                    path_found=True
                    break
                
                for i in range(0,len(self.rows)):
                    child_y = current_cell[2][1] + self.rows[i]
                    child_x = current_cell[2][0] + self.columns[i]
                    cost = 0
                    if i < 4:
                        cost = 10
                    else:
                        cost = 14
                    co_ordinates = current_cell[2]
                    if child_y>=0 and child_x>=0 and child_y < self.H and child_x < self.W and abs(self.grid[child_y][child_x] - self.grid[current_cell[2][1]][current_cell[2][0]])<=self.Z:
                        g_cost = current_cell[3] + cost + abs(self.grid[co_ordinates[1]][co_ordinates[0]] - self.grid[child_y][child_x])
                        h_cost = self.heuristic((child_x,child_y),target)
                        f_cost = g_cost + h_cost
                        child_node = Node(f_cost,g_cost,h_cost)
                        if (child_x,child_y) in explored:
                            if explored[(child_x,child_y)] > g_cost:
                                del explored[(child_x,child_y)]
                                self.appendToHeap(child_node,(child_x,child_y),queue_replica,heap,(current_cell[2][0],current_cell[2][1]),path_map,True)
                        else:
                                self.appendToHeap(child_node,(child_x,child_y),queue_replica,heap,(current_cell[2][0],current_cell[2][1]),path_map,True)
                        # print(child_x,child_y)
                        # print(heap,"heap after child put")
                        # print(path_map,"path_map")

            if path_found:
                path = []
                current_node = target 
                while current_node != start:
                    path.append(current_node)
                    current_node = path_map[current_node]
                path.append(start)
                result.append(path[::-1])
            else:
                result.append("FAIL")
        return result

    def bfs(self):
        result = []
        count = 0
        start = (self.X,self.Y)
        visited = set()
        prev_cells = {}
        path_found = False
        queue = deque()
        if start in self.target_array:
            count +=1
            prev_cells[start] = start
            if count == self.no_of_targets:
                path_found = True
        queue.append(start)
        while queue and not path_found:
            current_cell = queue.popleft()
            visited.add(current_cell)
            for j in range(0,len(self.rows)):
                child_y = current_cell[1]+self.rows[j]
                child_x = current_cell[0]+self.columns[j]
                if child_y>=0 and child_x>=0 and child_y < self.H and child_x < self.W and (child_x,child_y) not in queue and (child_x,child_y) not in visited and abs(self.grid[child_y][child_x] - self.grid[current_cell[1]][current_cell[0]])<=self.Z:                    
                    prev_cells[(child_x,child_y)] = current_cell
                    if [child_x,child_y] in self.target_array:
                        count+=1
                        if count == self.no_of_targets:
                            path_found = True
                            break
                    queue.append((child_x,child_y))
        for n in range(0, self.no_of_targets):
            path = []
            current_node = tuple(self.target_array[n])
            if current_node in prev_cells:
                while current_node != start:
                    path.append(current_node)
                    current_node = prev_cells[current_node]
                path.append(start)
                result.append(path[::-1])
            else:
                result.append('FAIL')
        return result

    def pushtoHeap(self,path_cost,child_x,child_y,queue_replica,heap,bool_val):
        ItemCount = next(self.ItemCounter)
        ItemEntry = [path_cost, ItemCount, (child_x,child_y)]
        queue_replica[(child_x,child_y)] = ItemEntry
        if bool_val == True:
            heap = heapq.heapify(list(queue_replica.values()))
        else:
            heapq.heappush(heap, ItemEntry)



    def ucs(self):
        result=[]
        for n in range(0,self.no_of_targets):
            path_found= False
            target = tuple(self.target_array[n])
            start = (self.X,self.Y)
            if start[1] == target[1] and start[0]==target[0]:
                result.append([(target[1],target[0])])
                path_found = True
            queue_replica={}
            explored=set()
            path_map={}
            heap=[]
            self.pushtoHeap(0,start[0],start[1],queue_replica,heap,False)
            while heap:
                current_cell=self.popFromHeap(heap,queue_replica)
                if current_cell[2][1] == target[1] and current_cell[2][0]==target[0]:
                    path_found = True
                    break
                explored.add(current_cell[2])
                for i in range(0,len(self.rows)):
                    child_y = current_cell[2][1] + self.rows[i]
                    child_x = current_cell[2][0] + self.columns[i]
                    cost = 0
                    if i < 4:
                        cost = 10
                    else:
                        cost = 14
                    path_cost = current_cell[0] + cost
                    if child_y>=0 and child_x>=0 and child_y < self.H and child_x < self.W and abs(self.grid[child_y][child_x] - self.grid[current_cell[2][1]][current_cell[2][0]])<=self.Z:
                        if (child_x,child_y) not in queue_replica and (child_x,child_y) not in explored:
                            self.pushtoHeap(path_cost,child_x,child_y,queue_replica,heap,False)
                            path_map[(child_x,child_y)] = (current_cell[2][0],current_cell[2][1])

                        elif (child_x,child_y) in queue_replica:
                            Item = queue_replica[(child_x,child_y)]
                            existing_cost= Item[0]
                            if existing_cost > path_cost:
                                self.pushtoHeap(path_cost,child_x,child_y,queue_replica,heap,True)
                                path_map[(child_x,child_y)] = (current_cell[2][0],current_cell[2][1])

            if path_found:
                path = []
                current_node = target 
                while current_node != start:
                    path.append(current_node)
                    current_node = path_map[current_node]
                path.append(start)
                result.append(path[::-1])
            else:
                result.append("FAIL")
        return result
    
if __name__=="__main__": 

    with open(os.path.join(sys.path[0], "input.txt"), "r") as search_input_file:
        data=""
        if search_input_file.mode == "r":
            data = search_input_file.read().splitlines()
            with open(os.path.join(sys.path[0], "output.txt"), "w+") as search_output_file:
                search = Search()
                search.prepare_input(data,search_output_file)
