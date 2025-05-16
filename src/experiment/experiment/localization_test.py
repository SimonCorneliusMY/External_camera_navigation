import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import time
import csv
import os

class LocalizationTest(Node):
    def __init__(self):
        super().__init__('localization_test')
        self.pose_subscriber = self.create_subscription(PoseStamped,'pose_0',self.pose_callback,10)
        self.conf_subscriber = self.create_subscription(Float32, 'conf_0',self.conf_callback, 10)

        self.conf_record = []
        # self.pose_pixel_x = []
        # self.pose_pixel_y = []
        self.pose_pixel_xy = []
        self.pose_count = 0
        self.not_detect_count = 0

    def conf_callback(self,msg:Float32):
        self.conf_record.append(msg.data)
        # self.get_logger().info(f'in conf callback{msg.data}')
        return

    def pose_callback(self, msg: PoseStamped):
        # self.get_logger().info('in pose callback')
        # Process the incoming pose message
        # self.pose_pixel_x.append(msg.pose.position.x)
        # self.pose_pixel_y.append(msg.pose.position.y) 
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

def pad_list(list1:list, list2:list[list], pad_value=0) -> list:
    max_length = max(len(list1), len(list2))
    padded_list1 = list1 + ['pad'] * (max_length - len(list1))
    padded_list2 = list2 + [['pad','pad']] * (max_length - len(list2))
    print(f'{padded_list2}')
    return padded_list1, padded_list2

def main(args=None):
    rclpy.init(args=args)
    localization_test = LocalizationTest()
    duration = 10.0


    while rclpy.ok():
        
        pose = (list(input("Key in xy pose or q to quit...").split()))
        if pose[0] == 'q':
            break
        elapse = 0.0
        count = 0
        localization_test.conf_record.clear()
        localization_test.pose_pixel_xy.clear()
        localization_test.pose_count = 0
        localization_test.not_detect_count = 0
        start_time = time.time()
        while elapse < duration:
            count += 1
            localization_test.get_logger().info(f'in elapse loop {count}')
            elapse = time.time()- start_time
            
            rclpy.spin_once(localization_test, timeout_sec=0.02) #fun fact, without timeout_sec, it will block till 1 message is received.
        
        localization_test.log_to_csv('localization_test.csv', pose)


    localization_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()