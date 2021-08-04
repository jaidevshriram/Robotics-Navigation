import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import random
from matplotlib.patches import Arc
import sys
import copy
import shapely.geometry
import shapely.ops
import descartes

from obstacles import *
from map import map as mapper
from kinematics import nonholonomic

# Coordinates of the spherical obstacles defined as global variables
obstacle_1_x=15
obstacle_1_y=3
obstacle_2_x=4
obstacle_2_y=5
obstacle_rs=1
        
# Function to populate the environment with pre-decided coordinates of the obstacles
def environment():
    plt.axis([-1, 21, -1, 21])
                
    global obstacle_1_x
    global obstacle_1_y
    global obstacle_2_x
    global obstacle_2_y
    global obstacle_rs

    obs1 = plt.Rectangle((4,9),1,12, fc='b', ec='k')
    plt.gca().add_patch(obs1)

    obs2 = plt.Rectangle((15,9),1,12, fc='b', ec='k')
    plt.gca().add_patch(obs2)

    obs3 = plt.Rectangle((9.5,-1),1,12, fc='b', ec='k')
    plt.gca().add_patch(obs3)

    obs = plt.Circle((obstacle_1_x, obstacle_1_y), radius=obstacle_rs, fc='b', ec='k')
    plt.gca().add_patch(obs)

    obs = plt.Circle((obstacle_2_x, obstacle_2_y), radius=obstacle_rs, fc='b', ec='k')
    plt.gca().add_patch(obs)

#Function to check if the node generated lies in the obstacle space or not
def obstacle_collision(x,y):
    global obstacle_1_x
    global obstacle_1_y
    global obstacle_2_x
    global obstacle_2_y
    global obstacle_rs
    c=0                  #counter variable to  check if node lies in obstacle. If it lies, the variable becomes 1.
    if ((4-0.8) < x < (4+1+0.8) and (9-0.8) < y < (9+0.8+11.8)):
        c=1

    if ((15-0.8) < x < (15+1+0.8) and (9-0.8) < y < (9+0.8+11.8)):
        c=1
         
    if ((9.5-0.8) < x < (9.5+1+0.8) and -0.8 < y < (0.8+11.8)):
        c=1 
            
    if ((x-obstacle_1_x)**2 + (y-obstacle_1_y)**2)**0.5 < (obstacle_rs+0.8):
        c=1
            
    if ((x-obstacle_2_x)**2 + (y-obstacle_2_y)**2)**0.5 < (obstacle_rs+0.8):
        c=1
        
    return c    
    
def LSL(current_x,current_y,current_th,goal_x,goal_y,goal_th,r_min):
    pc1_x=current_x+r_min*math.cos(math.radians(current_th)-(np.pi/2)) #first circle centre point
    pc1_y=current_y+r_min*math.sin(math.radians(current_th)-(np.pi/2))

    pc2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(np.pi/2))  #second circle centre point
    pc2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(np.pi/2))
    theta = (np.arctan2(pc2_x-pc1_x, pc2_y-pc1_y))

    pt1_x=pc1_x+r_min*math.cos(theta)  #tangent point on first circle
    pt1_y=pc1_y-r_min*math.sin(theta)

    pt2_x=pc2_x+r_min*math.cos(theta)  #tangent point on second circle
    pt2_y=pc2_y-r_min*math.sin(theta)

    S = ((pc2_x-pc1_x)**2 + (pc2_y-pc1_y)**2)**0.5       #Distance between centres

    Ti = np.arctan2(current_x-pc1_x,current_y-pc1_y) - np.arctan2(pt1_x-pc1_x,pt1_y-pc1_y) #First Turning angle

    Tf = np.arctan2(pt2_x - pc2_x,pt2_y- pc2_y) - np.arctan2(goal_x - pc2_x,goal_y- pc2_y) #Second Turning angle

    if Ti<0:
        Ti+=2*np.pi                       #changing first turning angle to positive
    if Tf<0:
        Tf+=2*np.pi                       #changing second turning angle to positive

    total_dist = Ti*r_min + Tf*r_min + S  #Total distance from current to Goal

    ans=[Ti,Tf, total_dist,S,pt1_x,pt1_y,pt2_x,pt2_y,pc1_x,pc1_y,pc2_x,pc2_y,"LL"]
    #    0   1.     2.     3.  4.    5.    6.    7.    8.    9.    10.  11.  12.

    return ans

def RSR(current_x,current_y,current_th,goal_x,goal_y,goal_th,r_min):
    pc1_x=current_x+r_min*math.cos(math.radians(current_th)-(np.pi/2))        #first circle centre point
    pc1_y=current_y+r_min*math.sin(math.radians(current_th)-(np.pi/2))

    pc2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(np.pi/2))             #second circle centre point
    pc2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(np.pi/2))

    theta = (np.arctan2(pc2_x-pc1_x, pc2_y-pc1_y))

    pt1_x=pc1_x-r_min*math.cos(theta) #tangent point on first circle
    pt1_y=pc1_y+r_min*math.sin(theta)

    pt2_x=pc2_x-r_min*math.cos(theta) #tangent point on second circle
    pt2_y=pc2_y+r_min*math.sin(theta)

    S = ((pc2_x-pc1_x)**2 + (pc2_y-pc1_y)**2)**0.5     #Distance between centres

    Ti = np.arctan2(pt1_x-pc1_x,pt1_y-pc1_y) - np.arctan2(current_x-pc1_x,current_y-pc1_y) #First Turning angle

    Tf = np.arctan2(goal_x - pc2_x,goal_y- pc2_y) - np.arctan2(pt2_x - pc2_x,pt2_y- pc2_y) #Second Turning angle
    
    if Ti<0:
        Ti+=2*np.pi              #changing first turning angle to positive
    if Tf<0:
        Tf+=2*np.pi              #changing second turning angle to positive

    total_dist = Ti*r_min + Tf*r_min + S  #Total distance from current to Goal

    #Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,pt1_x,pt1_y,pt2_x,pt2_y,pc1_x,pc1_y,pc2_x,pc2_y,"RR"]
    #    0   1.     2.     3.  4.    5.    6.    7.    8.    9.    10.  11.  12.
    
    return ans

def LSR(current_x,current_y,current_th,goal_x,goal_y,goal_th,r_min):
    pc1_x=current_x-r_min*math.cos(math.radians(current_th)-(np.pi/2))    #first circle centre point
    pc1_y=current_y-r_min*math.sin(math.radians(current_th)-(np.pi/2))

    pc2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(np.pi/2))       #second circle centre point
    pc2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(np.pi/2))

    S1 = ((pc2_x-pc1_x)**2 + (pc2_y-pc1_y)**2)**0.5 #Distance between centres

    if(S1**2 - ((2*r_min)**2))<0:        #checking if value to be square rooted is negative or not
        return None
    
    S = (S1**2 - ((2*r_min)**2))**0.5     #Distance of straight path between tangent points

    theta = (np.arctan2((pc2_x-pc1_x), pc2_y-pc1_y)) - np.arctan2(r_min,S/2.0)

    pt1_x=pc1_x+r_min*math.cos(theta)    #tangent point on first circle
    pt1_y=pc1_y-r_min*math.sin(theta)

    pt2_x=pc2_x-r_min*math.cos(theta)    #tangent point on second circle
    pt2_y=pc2_y+r_min*math.sin(theta)

    Ti = np.arctan2(current_x-pc1_x,current_y-pc1_y) - np.arctan2(pt1_x-pc1_x,pt1_y-pc1_y)    #First Turning angle

    Tf = np.arctan2(goal_x - pc2_x,goal_y- pc2_y) - np.arctan2(pt2_x - pc2_x,pt2_y- pc2_y)    #Second Turning angle

    if Ti<0:
        Ti+=2*np.pi       #changing first turning angle to positive
    if Tf<0:
        Tf+=2*np.pi       #changing second turning angle to positive

    total_dist = Ti*r_min + Tf*r_min + S  #Total distance from current to Goal

    #Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,pt1_x,pt1_y,pt2_x,pt2_y,pc1_x,pc1_y,pc2_x,pc2_y,"LR"]
    #    0   1.    2.      3.  4.    5.    6.    7.    8.    9.    10.  11.   12.
    
    return ans

def RSL(current_x,current_y,current_th,goal_x,goal_y,goal_th,r_min):
    pc1_x=current_x+r_min*math.cos(math.radians(current_th)-(np.pi/2))     #first circle centre point
    pc1_y=current_y+r_min*math.sin(math.radians(current_th)-(np.pi/2))

    pc2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(np.pi/2))           #second circle centre point
    pc2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(np.pi/2))

    S1 = ((pc2_x-pc1_x)**2 + (pc2_y-pc1_y)**2)**0.5          #Distance between centres

    if(S1**2 - ((2*r_min)**2))<0:                            #checking if value to be square rooted is negative or not
        return None
    
    S = (S1**2 - ((2*r_min)**2))**0.5                        #Distance of straight path between tangent points

    theta =   + (np.arctan2((pc2_x-pc1_x), pc2_y-pc1_y)) + np.arctan2(r_min,S/2.0)

    pt1_x=pc1_x-r_min*math.cos(theta) #tangent point on first circle
    pt1_y=pc1_y+r_min*math.sin(theta)

    pt2_x=pc2_x+r_min*math.cos(theta) #tangent point on second circle
    pt2_y=pc2_y-r_min*math.sin(theta)
    
    Ti = np.arctan2(pt1_x-pc1_x,pt1_y-pc1_y) - np.arctan2(current_x-pc1_x,current_y-pc1_y)    #First Turning angle

    Tf = np.arctan2(pt2_x - pc2_x,pt2_y- pc2_y) - np.arctan2(goal_x - pc2_x,goal_y- pc2_y)    #Second Turning angle

    if Ti<0:
        Ti+=2*np.pi                #changing first turning angle to positive
    if Tf<0:
        Tf+=2*np.pi                #changing second turning angle to positive

    total_dist = Ti*r_min + Tf*r_min + S             #Total distance from current to Goal

    #Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,pt1_x,pt1_y,pt2_x,pt2_y,pc1_x,pc1_y,pc2_x,pc2_y,"RL"]
    #    0   1.     2.     3.  4.    5.    6.    7.    8.   9.    10.    11.  12.
    
    return ans

#Finding nearest node to the randomly generated node in the tree
def findNearest(sample,tree):
    min_d = sys.maxsize
    ind=0
    min_node=tree[0]
    for indd,node in enumerate(tree):
        dis = ((node.x-sample.x)**2 + (node.y-sample.y)**2)**0.5
        if min_d>dis:
            min_d=dis
            min_node=node
            ind=indd
    return min_node,ind

class RRT:
    def __init__(self, map, initistate, endstate):
        self.initistate = initistate
        self.endstate = endstate
        self.map.create_graph(initstate, endpos)

#Function to compute RRT
def RRT(start_x,start_y,start_theta,goal_x,goal_y,goal_theta):  
    global obstacle_1_x
    global obstacle_1_y

    global obstacle_2_x
    global obstacle_2_y

    global obstacle_rs
    
    current_x=start_x             #start x-coordinate is assigned to current x-coordinate
    current_y=start_y             #start y-coordinate is assigned to current y-coordinate
    current_th=start_theta        #current angle in Degrees

    r_min=0.2
    step=1                    #growth factor to limit the growth if node in the obstacle

    goal_x=goal_x                #goal x-coordinate
    goal_y=goal_y                #goal y-coordinate
    goal_th=goal_theta           #goal angle in Degrees

    start_circle = plt.Circle((current_x, current_y), radius=0.4, fc='c')               #Plotting start circle
    plt.gca().add_patch(start_circle)

    goal_circle = plt.Circle((goal_x, goal_y), radius=0.4, fc='y')                      #Plotting goal circle
    plt.gca().add_patch(goal_circle)
    
    plt.title("Dubin's car")                                  #adding title to the plot
    plt.legend((start_circle, goal_circle),('Start Node','Goal Node'), loc = 'lower left', bbox_to_anchor =(1.0,0.9))      #adding legends
    plt.gca().set_aspect('equal', adjustable='box')

    tree = [Node(current_x,current_y)]                  #adding the first node to the tree
    
    j=0
    initial=[]
    theta_end=None
    iter = 0
    while True:
        j+=1

        sample = Node(random.uniform(0,20),random.uniform(0,20))               #generating random node in the sample space
        nearestNode,ind = findNearest(sample,tree)                             #finding the nearest node from the tree
        newNode = copy.deepcopy(nearestNode)
        
        theta = (np.arctan2(sample.y-nearestNode.y, sample.x-nearestNode.x))      #angle between sample node and nearest node
        newNode.x+=step*math.cos(theta)
        newNode.y+=step*math.sin(theta)

        theta_start = theta
        theta_end = (np.arctan2(newNode.y-goal_y, newNode.x-goal_x))

        paths=[LSL,RSR,LSR,RSL]                                               #possible paths that the car can take
        minS=sys.maxsize
        ans=[]
        count=0
        curve=""
        ans_count=1
        for i in paths:
            path_ans=i(nearestNode.x,nearestNode.y,theta_start,newNode.x,newNode.y,theta_end,r_min)
            count+=1
            if path_ans:
                if minS>path_ans[2]:
                    ans=path_ans
                    minS=path_ans[3]
                    ans_count=count
                    curve=path_ans[12]
        
        if obstacle_collision(newNode.x,newNode.y) == 0:                       #checking if node is in collision space
            plt.plot([ans[4],ans[6]],[ans[5],ans[7]],'-g')

            start_angle1 = np.arctan2(nearestNode.y - ans[9], nearestNode.x - ans[8]) * 180 / np.pi
            end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi

            if curve[0]=="L":
                arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1, color='g')
                newNode.coord1 = ans[8]
                newNode.coord2 = ans[9]
                newNode.theta11 = start_angle1
                newNode.theta12 = end_angle1 
            else:
                arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1, color='g')
                newNode.coord1 = ans[8]
                newNode.coord2 = ans[9]
                newNode.theta11 = end_angle1
                newNode.theta12 = start_angle1
            plt.gca().add_patch(arc1)                                         #plotting the first curve

            start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
            end_angle2 = np.arctan2(newNode.y - ans[11], newNode.x - ans[10]) * 180 / np.pi

            if curve[1]=="L":
                arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2, color='g')
                newNode.coord3 = ans[10]
                newNode.coord4 = ans[11]
                newNode.theta13 = start_angle2
                newNode.theta14 = end_angle2
            else:
                arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2, color='g')
                newNode.coord3 = ans[10]
                newNode.coord4 = ans[11]
                newNode.theta13 = end_angle2
                newNode.theta14 = start_angle2

            plt.gca().add_patch(arc2)                                       #plotting the ending curve
            plt.pause(0.01)

            plt.gca().set_aspect('equal')
            plt.title('Non-Holo')
            plt.savefig(f"./out/{iter}.png")
            iter += 1

            # plt.clf()

            newNode.p=ind                                                  #storing the index of nearest node as parent in the new node
            newNode.pt1_x=ans[4]
            newNode.pt1_y=ans[5]
            newNode.pt2_x=ans[6]
            newNode.pt2_y=ans[7]
            newNode.S=ans[3]
            newNode.curve=curve
            newNode.Ti=ans[0]
            if j==1:
                initial.append([ans[4],ans[5],ans[6],ans[7],ans[3],curve,ans[0]])
            tree.append(newNode)
            
            if ((newNode.x-goal_x)**2 + (newNode.y-goal_y)**2)**0.5 < step:       #stopping the loop if the node is very close to goal
                break
    
    #These set of statements are executed for the last node to the goal node
    paths=[LSL,RSR,LSR,RSL] 
    minS=sys.maxsize
    ans=[]
    count=0
    ans_count=1
    for i in paths:
        path_ans=i(newNode.x,newNode.y,theta_end,goal_x,goal_y,goal_th,r_min)
        count+=1
        if path_ans:
            if minS>path_ans[3]:                          #computing best possible path taht teh car can take
                ans=path_ans
                minS=path_ans[3]
                ans_count=count
                curve=path_ans[12]

    goalNode = copy.deepcopy(nearestNode)
    
    goalNode.x=goal_x
    goalNode.y=goal_y
    goalNode.pt1_x=ans[4]
    goalNode.pt1_y=ans[5]
    goalNode.pt2_x=ans[6]
    goalNode.pt2_y=ans[7]
    goalNode.S=ans[3]
    goalNode.curve=curve
    goalNode.Ti=ans[0]

    plt.plot([ans[4],ans[6]],[ans[5],ans[7]])

    start_angle1 = np.arctan2(newNode.y - ans[9], newNode.x - ans[8]) * 180 / np.pi
    end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi
    
    if curve[0]=="L":
        arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1, color='g')
        goalNode.coord1 = ans[8]
        goalNode.coord2 = ans[9]
        goalNode.theta11 = start_angle1
        goalNode.theta12 = end_angle1 
    else:
        arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1, color='g')
        goalNode.coord1 = ans[8]
        goalNode.coord2 = ans[9]
        goalNode.theta11 = end_angle1
        goalNode.theta12 = start_angle1
    plt.gca().add_patch(arc1)

    start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
    end_angle2 = np.arctan2(goal_y - ans[11], goal_x - ans[10]) * 180 / np.pi
    
    if curve[1]=="L":
        arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2, color='g')
        goalNode.coord3 = ans[10]
        goalNode.coord4 = ans[11]
        goalNode.theta13 = start_angle2
        goalNode.theta14 = end_angle2
    else:
        arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2, color='g')
        goalNode.coord3 = ans[10]
        goalNode.coord4 = ans[11]
        goalNode.theta13 = end_angle2
        goalNode.theta14 = start_angle2

    plt.gca().add_patch(arc2)

    plt.gca().set_aspect('equal')

    #Storing the nodes from the tree in the variable called path
    path=[[newNode.pt1_x,newNode.pt1_y,newNode.pt2_x,newNode.pt2_y,newNode.S,newNode.curve,newNode.Ti,newNode.coord1,newNode.coord2,newNode.coord3,newNode.coord4,newNode.theta11,newNode.theta12,newNode.theta13,newNode.theta14]]
    while True:
        if not newNode.p:
            break
        path.append([tree[newNode.p].pt1_x,tree[newNode.p].pt1_y,tree[newNode.p].pt2_x,tree[newNode.p].pt2_y,tree[newNode.p].S,tree[newNode.p].curve,tree[newNode.p].Ti,tree[newNode.p].coord1,tree[newNode.p].coord2,tree[newNode.p].coord3,tree[newNode.p].coord4,tree[newNode.p].theta11,tree[newNode.p].theta12,tree[newNode.p].theta13,tree[newNode.p].theta14])
        newNode=tree[newNode.p]

    def arc_to_shapely(start_angle, end_angle, centerx, centery, radius):
        theta = np.radians(np.linspace(start_angle, end_angle, 100000))
        x = centerx + radius * np.cos(theta)
        y = centery + radius * np.sin(theta)

        return shapely.geometry.LineString(np.column_stack([x, y]))

    def get_angle(a, b, c, d):
        line1 = np.array([a-c, b-d])
        line2 = np.array([1, 0])

        line_1_norm = np.sqrt(np.sum(line1 ** 2))
        line_2_norm = np.sqrt(np.sum(line2 ** 2))

        return np.arccos((line1[0])/(line_1_norm * line_2_norm))

    def compute_R(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])

    def compute_translations(R):
        o = R @ np.array([1, 0]).T
        l = R @ np.array([1, 0.025]).T
        r = R @ np.array([1, -0.025]).T

        return (r-o)[:], (l-o)[:]

    # plt.show()
    plt.clf()

    lines = []

#Plotting the path that the car has to take from start to goal in red       
    plt.plot([goalNode.pt1_x, goalNode.pt2_x], [goalNode.pt1_y, goalNode.pt2_y],'r', linewidth=1, alpha=0.5)
    lines.append(shapely.geometry.LineString([(goalNode.pt1_x, goalNode.pt1_y), (goalNode.pt2_x, goalNode.pt2_y)]))

    theta = get_angle(goalNode.pt2_x, goalNode.pt2_y, goalNode.pt1_x, goalNode.pt1_y)
    R = compute_R(theta)
    r, l = compute_translations(R)

    print(r, l, theta)

    plt.plot([goalNode.pt1_x + r[0], goalNode.pt2_x + r[0]], [goalNode.pt1_y + r[1], goalNode.pt2_y + r[1]], 'black', linewidth=1)
    plt.plot([goalNode.pt1_x + l[0], goalNode.pt2_x + l[0]], [goalNode.pt1_y + l[1], goalNode.pt2_y + l[1]], 'black', linewidth=1)

    c1 = 'black'
    c2 = 'black'

    arcf1 = patches.Arc((goalNode.coord1, goalNode.coord2), 2*r_min,2*r_min, theta1=goalNode.theta11, theta2=goalNode.theta12, color='r', linewidth=1)
    plt.gca().add_patch(arcf1)

    arcf1 = patches.Arc((goalNode.coord1, goalNode.coord2), 2*r_min,2*r_min, theta1=goalNode.theta11, theta2=goalNode.theta12, color='r', linewidth=1)
    # lines.append(arc_to_shapely(goalNode.theta11, goalNode.theta12, goalNode.coord1, goalNode.coord2, 2*r_min))

    arcf1 = patches.Arc((goalNode.coord1, goalNode.coord2), 2*r_min + 0.04, 2*r_min + 0.04, theta1=goalNode.theta11, theta2=goalNode.theta12, color=c1, linewidth=1, alpha=0.5)
    plt.gca().add_patch(arcf1)

    arcf1 = patches.Arc((goalNode.coord1, goalNode.coord2), 2*r_min - 0.04, 2*r_min - 0.04, theta1=goalNode.theta11, theta2=goalNode.theta12, color=c2, linewidth=1, alpha=0.5)
    plt.gca().add_patch(arcf1)

    arcf2 = patches.Arc((goalNode.coord3, goalNode.coord4), 2*r_min,2*r_min,theta1=goalNode.theta13, theta2=goalNode.theta14, color='r', linewidth=1)
    plt.gca().add_patch(arcf2)
    # lines.append(arc_to_shapely(goalNode.theta13, goalNode.theta14, goalNode.coord3, goalNode.coord4, 2*r_min))

    arcf2 = patches.Arc((goalNode.coord3, goalNode.coord4), 2*r_min + 0.04 ,2*r_min + 0.04,theta1=goalNode.theta13, theta2=goalNode.theta14, color=c1, linewidth=1, alpha=0.5)
    plt.gca().add_patch(arcf2)

    arcf2 = patches.Arc((goalNode.coord3, goalNode.coord4), 2*r_min - 0.04,2*r_min- 0.04,theta1=goalNode.theta13, theta2=goalNode.theta14, color=c2, linewidth=1, alpha=0.5)
    plt.gca().add_patch(arcf2)

    # last_center = (goalNode.coord3, goalNode.coord4)

    for i in path:

        plt.plot([i[0],i[2]], [i[1],i[3]],'r', linewidth=1, alpha=0.5)
        # lines.append(shapely.geometry.LineString([(i[0], i[1]), (i[2], i[3])]))

        theta = get_angle(i[0], i[1], i[2], i[3])
        R = compute_R(theta)
        r, l = compute_translations(R)

        print(r, l, theta)

        plt.plot([i[0] + r[0], i[2] + r[0]], [i[1] + r[1], i[3] + r[1]], 'black', linewidth=1)
        plt.plot([i[0] + l[0], i[2] + l[0]], [i[1] + l[1], i[3] + l[1]], 'black', linewidth=1)

        arcf1 = patches.Arc((i[7], i[8]), 2*r_min,2*r_min,theta1=i[11], theta2=i[12], color='r', linewidth=1)
        plt.gca().add_patch(arcf1)

        # lines.append(arc_to_shapely(i[11], i[12], i[7], i[8], 2*r_min))

        arcf1 = patches.Arc((i[7], i[8]), 2*r_min + 0.04, 2*r_min + 0.04, theta1=i[11], theta2=i[12], color=c1, linewidth=1)
        plt.gca().add_patch(arcf1)

        arcf1 = patches.Arc((i[7], i[8]), 2*r_min - 0.04, 2*r_min - 0.04, theta1=i[11], theta2=i[12], color=c2, linewidth=1)
        plt.gca().add_patch(arcf1)

        # last_center = (i[7], i[8]) 

        arcf2 = patches.Arc((i[9], i[10]), 2*r_min,2*r_min,theta1=i[13], theta2=i[14], color='r', linewidth=1)
        plt.gca().add_patch(arcf2)

        # lines.append(arc_to_shapely(i[13], i[14], i[9], i[10], 2*r_min))

        arcf2 = patches.Arc((i[9], i[10]), 2*r_min + 0.04 ,2*r_min + 0.04,theta1=i[13], theta2=i[14], color=c1, linewidth=1)
        plt.gca().add_patch(arcf2)

        arcf2 = patches.Arc((i[9], i[10]), 2*r_min - 0.04,2*r_min- 0.04,theta1=i[13], theta2=i[14], color=c2, linewidth=1)
        plt.gca().add_patch(arcf2)

    plt.gca().set_aspect('equal')
    # plt.axis([-1, 8, -1, 8])
    plt.title('Non-Holo Wheel Trajectory')
    plt.savefig(f"./out/wheel.png")

if __name__ == "__main__":
    location=True
    counter = 1
    counter1 = 1
    while location:
        if counter == 1:
            # st_x,st_y,st_th = input('Enter start coordinates - "x y theta" (range of coordinates 0-20, range of theta 0-360)\n').split()
            st_x,st_y,st_th = 0, 0, 0 
            start_x=float(st_x)
            start_y=float(st_y)
            start_theta=float(st_th)
            counter = obstacle_collision(start_x,start_y)

        if counter1 == 1:
            # gl_x,gl_y,gl_th=input('Enter goal coordinates - "x y theta" (range of coordinates 0-20, range of theta 0-360)\n').split()
            gl_x,gl_y,gl_th = 0, 8, 0
            goal_x=float(gl_x)
            goal_y=float(gl_y)
            goal_theta=float(gl_th)
            counter1 = obstacle_collision(goal_x,goal_y)
        
        if counter == 0 and counter1 == 0:
            location = False
            print('The start and goal coordinates are obstacle free!\n')
        if counter == 0 and counter1 == 1:
            location = True
            print('The goal coordinates lie in an obstacle region. Enter goal coordinates again.\n')
        if counter == 1 and counter1 == 0:
            location = True
            print('The start coordinates lie in an obstacle region. Enter start coordinates again.\n')
        if counter == 1 and counter1 == 1:
            location = True
            print('The start and goal coordinates both lie in obstacles. Enter again both.\n')

    environment()    
    RRT(start_x,start_y,start_theta,goal_x,goal_y,goal_theta)