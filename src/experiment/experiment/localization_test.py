import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import time
import csv
import os

"""
29/7/2026 Localization test that records the confidence and pixel position of the detected object
saves in csv file
"""

class LocalizationTest(Node):
    def __init__(self):
        super().__init__('localization_test')
        self.pose_subscriber = self.create_subscription(PoseStamped,'pose_0',self.pose_callback,1)
        self.conf_subscriber = self.create_subscription(Float32, 'conf_0',self.conf_callback, 1)

        self.conf_record = []
        self.pose_pixel_xy = []
        self.pose_count = 0
        self.not_detect_count = 0

    def conf_callback(self,msg:Float32):
        self.conf_record.append(msg.data)
        # self.get_logger().info(f'in conf callback{msg.data}')
        return

    def pose_callback(self, msg: PoseStamped):
        self.pose_pixel_xy.append([msg.pose.position.x,msg.pose.position.y])
        self.pose_count +=1
        if msg.header.frame_id == 'No objects detected':
            self.not_detect_count += 1
        return
        

    def log_to_csv(self,filename:str,pose:list):
        path = os.getcwd() + '/' + filename
        file_exists = os.path.exists(path)
        mode = 'a' if  file_exists else 'w'
        with open(filename, mode, newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            if not(file_exists):
                writer.writerow(['x', 'y', 'confidence','pixel_x','pixel_y','frames', 'not_detect_count'])

            conf, pose_pixel = pad_list(self.conf_record, self.pose_pixel_xy)
            for conf_i in range(len(conf)):
                writer.writerow([pose[0], pose[1],conf[conf_i],pose_pixel[conf_i][0],pose_pixel[conf_i][1],self.pose_count,self.not_detect_count])

        self.get_logger().info('Data logged to CSV file')

def pad_list(list1:list, list2:list[list], pad_value=0) -> list:
    max_length = max(len(list1), len(list2))
    padded_list1 = list1 + ['pad'] * (max_length - len(list1))
    padded_list2 = list2 + [['pad','pad']] * (max_length - len(list2))
    # print(f'{padded_list2}')
    return padded_list1, padded_list2

def main(args=None):
    rclpy.init(args=args)
    localization_test = LocalizationTest()
    duration = 10.0
    poses = [[1.5,0],[2,0],[2.5,0],[3,0],[3.56,0],[1.5,0.5],[2,0.5],[2.5,0.5],[3,0.5],[3.56,0.5],
             [1.5,1],[2,1],[2.5,1],[3,1],[3.56,1],[1.5,1.5],[2,1.5],[2.5,1.5],[3,1.5],[3.56,1.5],
             [1.5,2],[2,2],[2.5,2],[3,2],[3.56,2],[1.5,2.5],[2,2.5],[2.5,2.5],[3,2.5],[3.56,2.5],
             [1.5,3],[2,3],[2.5,3],[3,3],[3.56,3],[1.5,3.5],[2,3.5],[2.5,3.5],[3,3.5],[3.56,3.5],
             [1.5,4],[2,4],[2.5,4],[3,4],[3.56,4],[1.5,4.5],[2,4.5],[2.5,4.5],[3,4.5],[3.56,4.5],
             [1.5,5],[2,5],[2.5,5],[3,5],[3.56,5],[1.5,5.5],[2,5.5],[2.5,5.5],[3,5.5],[3.56,5.5],
             [1.5,6],[2,6],[2.5,6],[3,6],[3.56,6],[1.5,6.5],[2,6.5],[2.5,6.5],[3,6.5],[3.56,6.5],
             [1.5,7],[2,7],[2.5,7],[3,7],[3.56,7],[1.5,7.5],[2,7.5],[2.5,7.5],[3,7.5],[3.56,7.5],
             [1.5,8],[2,8],[2.5,8],[3,8],[3.56,8]]


#     poses = [[1.5,0],[2.5,0],[3.56,0],[1.5,1],[2.5,1],[3.56,1],[1.5,2],[2.5,2],
#              [3.56,2],[1.5,3],[2.5,3],[3.56,3],[1.5,4],[2.5,4],[3.56,4],[1.5,5],
#              [2.5,5],[3.56,5],[1.5,6],[2.5,6],[3.56,6],[1.5,7],[2.5,7],[3.56,7],[1.5,8],[2.5,8],[3.56,8]
# ]

    while rclpy.ok():

        for pose in poses:
        
            key = input(f'Press Enter to start gathering data for pose {pose} or type "q" to quit: ')
            localization_test.get_logger().info(f'Gathering data for {duration} seconds')
            if key == 'q':
                localization_test.destroy_node()
                rclpy.shutdown()
                break
            elapse = 0.0
            count = 0
            localization_test.conf_record.clear()
            localization_test.pose_pixel_xy.clear()
            localization_test.pose_count = 0
            localization_test.not_detect_count = 0
            start_time = time.time()
            while elapse < duration:
                # count += 1
                # localization_test.get_logger().info(f'in elapse loop {count}')
                elapse = time.time()- start_time
                
                rclpy.spin_once(localization_test, timeout_sec=0.02) #fun fact, without timeout_sec, it will block till 1 message is received.
            
            localization_test.log_to_csv('localization_test.csv', pose)


    
    


if __name__ == '__main__':
    main()