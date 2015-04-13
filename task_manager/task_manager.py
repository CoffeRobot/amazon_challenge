__author__ = 'alessandro'

import json
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class TaskManager:

    TASK_COMPLETED = 1
    TASK_CANCELLED = 2
    TASK_POSTPONED = 3

    def __init__(self):
        self.objects_list = []
        self.data = []
        self.bins = {}
        self.task_list = []
        self.pub = rospy.Publisher('amazon_next_task', String, queue_size = 0)
        rospy.init_node('amazon_task_scheduler', anonymous=True)
        rospy.Subscriber('/amazon_task_update', Int32, self.receive_update)
        self.pub_rate = rospy.Rate(30)

    def add_bin_content(self, bin_id):
        content = self.data['bin_contents']
        self.bins[bin_id] = content[bin_id]

    def init_taks_list(self):
        tasks = self.data['work_order']
        for task in tasks:
            bin = task['bin']
            item = task['item']
            t = [len(self.bins[bin]), item, bin]
            self.task_list.append(t)

        self.task_list.sort(key=lambda tup: tup[0])

        #next_task = self.task_list[0]
        #next_task_msg = '[' + next_task[2] + "," + next_task[1] + ']'
        #print(next_task_msg)

    def receive_update(self,msg):
        value = msg.data

        if len(self.task_list) < 1
            return

        if value == TASK_COMPLETED:
            self.task_list.pop(0)
        elif value == TASK_CANCELLED:
            self.task_list.pop(0)
        elif value == TASK_POSTPONED:
            task = self.task_list.pop(0)
            self.task_list.append(task)

        print('Task list updated')

    def read_json_list(self, file_path):

        json_data = open(file_path)
        self.data = json.load(json_data)
        json_data.close()

        self.add_bin_content('bin_A')
        self.add_bin_content('bin_B')
        self.add_bin_content('bin_C')
        self.add_bin_content('bin_D')
        self.add_bin_content('bin_E')
        self.add_bin_content('bin_F')
        self.add_bin_content('bin_G')
        self.add_bin_content('bin_H')
        self.add_bin_content('bin_I')
        self.add_bin_content('bin_J')
        self.add_bin_content('bin_K')
        self.add_bin_content('bin_L')

        self.init_taks_list()

    def publish_next_task(self):
        while not rospy.is_shutdown():
            next_task_msg = '[empty,empty]'
            if len(self.task_list) > 0:
                next_task = self.task_list[0]
                next_task_msg = '[' + next_task[2] + "," + next_task[1] + ']'
            self.pub.publish(next_task_msg)
            self.rate.sleep()


task_manager = TaskManager()
task_manager.read_json_list('/home/alessandro/catkin_ws/src/amazon_challenge/data/example.json')
task_manager.publish_next_task()



