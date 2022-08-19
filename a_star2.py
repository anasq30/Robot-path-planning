import numpy as np
import heapq
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class Priority_queue():
    def __init__(self):
        self.queue = []

    def is_empty(self):
        return not self.queue

    def put(self,item,priority):
        return heapq.heappush(self.queue,(priority,item))

    def get(self):
        return heapq.heappop(self.queue)[1]

    def __str__(self):
        return str(self.queue)



class A_star():
    def __init__(self):
        self.pq = Priority_queue()

        self.map = np.array([
                    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                    [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                    [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
                    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
                    [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
                    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
                    [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
                    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
                    [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
                    [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
                    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])

        #self.offsets = {'d1':(-1,1),'right':(0,1),'d2':(1,1),'down':(-1,0),'inf':(self.inf,self.inf),'up':(1,0),'d3':(-1,-1),'left':(0,-1),'d4':(1,-1)}
        self.offsets = {'right':(0,1),'down':(-1,0),'up':(1,0),'left':(0,-1)}
        self.predecessor = {}
        self.start = (0,0)
        self.goal = (0,0)
        self.path = []
        self.robot_pose_subscriber = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.robot_pose_cb)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.robot_pose = Odometry()
        self.cmd_vel = Twist()
        self.reached = False
        self.index = 0
        self.is_rotation = True
        self.goalx,self.goaly=rospy.get_param("goalx"),rospy.get_param("goaly")
        self.goal = (round(self.goalx+8.0),round(10-self.goaly))

    def robot_pose_cb(self, msg):
        self.robot_pose = msg
        

    def get_robot_position(self):
        return self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y 

    def get_heading(self):
        heading = tf.transformations.euler_from_quaternion([
            self.robot_pose.pose.pose.orientation.x,
            self.robot_pose.pose.pose.orientation.y,
            self.robot_pose.pose.pose.orientation.z,
            self.robot_pose.pose.pose.orientation.w])[2]
        return heading    

    def drive_robot(self,pth):
        if not self.reached:
            
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = 0
            
            robot_pose_x, robot_pose_y = self.get_robot_position()
            #print(type(self.path))
            pth = self.get_path()
            self.goaly=10.0-pth[self.index][1]
            self.goalx=pth[self.index][0]-8.0          
            
            

            heading_error = math.atan2(self.goaly-robot_pose_y, self.goalx-robot_pose_x) - self.get_heading()
            if self.is_rotation:
                if math.fabs(heading_error) > math.radians(6):
                    if heading_error < 0:
                        heading_error += math.pi * 2
                    elif heading_error > math.pi * 2:
                        heading_error -= math.pi * 2
                    if heading_error > math.pi:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = -0.75
                        self.vel_pub.publish(self.cmd_vel)
                    else:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = 0.75
                        self.vel_pub.publish(self.cmd_vel)
                else:
                    self.is_rotation=False
            else:
                error=math.sqrt((self.goalx-robot_pose_x)**2+(self.goaly-robot_pose_y)**2)
                if error> 0.5:
                    self.cmd_vel.linear.x = 0.75
                    self.cmd_vel.angular.z = 0.0
                    self.vel_pub.publish(self.cmd_vel)
                else:
                    self.is_rotation=True
                    if self.index+1<len(pth):
                        self.index+=1
                    else:
                        self.reached=True

    def heuristic(self,p1,p2):
        x1,y1 = p1
        x2,y2 = p2

        dist = abs(x1-x2) + abs(y1-y2)
        return dist

    def astar(self):
        self.pq.put(self.start,0)
        self.predecessor = {self.start:None}
        g_value = {self.start:0}
        

        while not self.pq.is_empty():
            current_cell = self.pq.get()
            #close_li.append(current_cell)
            if current_cell == self.goal:
                return self.get_path()
    
            for direction in ['right','down','up','left']:
                row_offset,col_offset = self.offsets[direction]
                neighbor = (current_cell[0] + row_offset, current_cell[1] + col_offset)
                print("reached")
                if self.is_legal(neighbor) and neighbor not in g_value :
                    new_cost = g_value[current_cell] +1
                    g_value[neighbor] = new_cost
                    f_value = new_cost + self.heuristic(self.goal,neighbor)
                    self.pq.put(neighbor,f_value)
                    self.predecessor[neighbor] = current_cell
        return None


    
    def get_path(self):
        self.path = []
        current = self.goal
        while current != self.start:
            self.path.append(current)
            current = self.predecessor[current]
        self.path.append(self.start)
        self.path.reverse()
        '''
        shifted_path = []
        for i,point in enumerate(self.path):
            shifted_path.append((point[0]+1, point[1]+1))
        '''
        return self.path
                        

    def is_legal(self,pos):
        i,j = pos
        print(pos)
        num_rows = len(self.map)
        num_cols = len(self.map[0])
        return 0 <= i < num_rows and 0 <= j < num_cols and self.map[i][j]!=1

    

if __name__=="__main__":
    rospy.init_node("a_star", anonymous=False)
    aa = A_star()
    aa.start = (1,12)
    aa.goal = (13,1)
    x = aa.astar()
    print(x)
    plt.rcParams["figure.figsize"] = [7.0,3.0]
    plt.rcParams["figure.autolayout"] = True
    '''
    for i,j in x:
        plt.plot(i,j,"r*")
        plt.axis([0,20,20,0])

    for i,j in x:
        plt.text(i,j+0.5, '({}, {})'.format(i,j))

    plt.show()
    '''
    for i in x:
        a = 1
        aa.map[i[0]+a,i[1]+a]=10
    plt.imshow(aa.map)
    plt.show()
    while not rospy.is_shutdown():

            if aa.reached==True:
                if rospy.has_param("/goalx") and rospy.has_param("/goaly"):

                    print("reached")
            else:
                aa.drive_robot(x)

    






    
