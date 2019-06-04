#!/usr/bin/env python2
import cv2
import numpy as np
import math

notwalkable =[]
class Basic_map:
    def __init__(self):
        self.Color_HSV = {}
        # hsv 3 low , hsv 3 high
        self.Color_HSV['RED'] = [126, 134, 0, 255, 255, 255]
        self.Color_HSV['BLUE'] = [91, 161, 0, 124, 255, 255]
        self.x = 0
        self.y = 0
        return
    def find(self,img, color): #find specific color points, in this case start, end_nodes
        lowerBound = np.array(self.Color_HSV[color][:3])
        upperBound = np.array(self.Color_HSV[color][3:6])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, lowerBound, upperBound)
        #ret, thr = cv2.threshold(color_mask, 127, 255, 0)
        contours, hierarchy= cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            for i in range(len(contours)):
                # Get area value
                area = cv2.contourArea(contours[i])
                if area > 1:  # minimum  area
                    rect = cv2.minAreaRect(contours[i])
                    (self.x, self.y), (w, h), angle = cv2.minAreaRect(contours[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    return round(self.x), round(self.y), color_mask
                else:
                    return 0, 0, 0

    def obstacle_load(self,img): # make base_map that has obstacle pixel data
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, base_map = cv2.threshold(img_gray, 22, 255, 1)
        return base_map

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.G = 0
        self.F = 0
        self.H = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(map, start, end):
    list = []
    list = map
    start_node = Node(None, start)
    start_node.G = start_node.H = start_node.F = 0
    end_node = Node(None, end)
    end_node.G = end_node.H = end_node.F = 0

    open_list =[]
    closed_list = []

    open_list.append(start_node)
    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list): #this means you compare current node with open_list node.
            if item.F < current_node.F: #if openlist item's F is small than current node's F, you changes openlist.
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path=[]
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]


        children =[]
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: #adjacent node
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1] ) #Get adjacent node position
            if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map)-1]) -1) or node_position[1] < 0: #check if node is out side end of map. like [0,-1]. there is no -1 array number.
                continue
            if map[node_position[1]][node_position[0]] != 0: # Make sure walkable terrain
                print("not walkable place = ", (node_position[0], node_position[1]))
                notgood = [node_position[0],node_position[1]]
                notwalkable.append(notgood)
                continue
            new_node = Node(current_node, node_position)  # this means, make current node as a parent and adjacent position as node position.
            children.append(new_node) #put this new nodes in the children !!!! By for moon, this will put all the adjacent node in the children

        for child in children: #if child in children has already in the closed_list
            for closed_child in closed_list:
                if child == closed_child: #so it child is already in the closed_list, ignore them
                    continue
            child.G = current_node.G + 1 #adjacent node(child) get +1 from current node's G score
            child.H = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.F = child.G + child.H
            #print("child.g", child.G, "child.h", child.H, "child.f", child.F)
            for open_node in open_list:
                if child == open_node and child.G > open_node.G:
                    continue
            open_list.append(child)

def main():
    img = cv2.imread('/home/benlee/catkin_ws/src/find_path/script/map.png', cv2.IMREAD_COLOR)
    start_point = Basic_map()
    end_point = Basic_map()
    obstacle_map = Basic_map()
    start_x, start_y, start_img = start_point.find(img, 'BLUE')
    start = (start_x, start_y+10) 
    end_x, end_y, end_img = end_point.find(img, 'RED')
    end = (end_x, end_y-10) #
    print("start",start,"end", end)
    capstone_map = obstacle_map.obstacle_load(img)
    #a = np.argwhere(capstone_map == 255)
    path = astar(capstone_map, start, end)
    if len(path)>0:
        for i in range(0, len(path)):
            print("Path :", path[i])
            cv2.circle(img, path[i], 1, (0,0,255), 1)
    print(notwalkable)
    '''if len(notwalkable)>0:
        for i in range(0, len(notwalkable)):
            cv2.circle(img, (notwalkable[i][0],notwalkable[i][1]), 3, (10,30,155), 1)'''
    cv2.imshow('start_img', start_img)
    cv2.imshow('end_img', end_img)
    cv2.imshow('obstacle', capstone_map)
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()