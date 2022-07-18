from tomlkit import boolean
import numpy as np
import argparse
import datetime
import os

import sys
import termios
import atexit
from select import select

import open3d as o3
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import ros_numpy
from sensor_msgs.msg import Image, PointCloud2
import message_filters

class KBHit:
    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''
        if os.name == 'nt':
            pass
        else:
            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)
            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)
    def set_normal_term(self):
        ''' Resets to normal terminal.  On Windows this is a no-op.
        '''
        if os.name == 'nt':
            pass
        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
            Should not be called in the same program as getarrow().
        '''
        s = ''
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        else:
            return sys.stdin.read(1)
    def getarrow(self):
        ''' Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        '''
        if os.name == 'nt':
            msvcrt.getch() # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]
        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]
        return vals.index(ord(c.decode('utf-8')))
    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        if os.name == 'nt':
            return msvcrt.kbhit()
        else:
            dr,dw,de = select([sys.stdin], [], [], 0)
            return dr != []

class Capture_data():
    def __init__(self):
        self.args = arg_parse()

        self.bridge = CvBridge()
        self.kb = KBHit()

        os.makedirs(self.args.save_dir)

        self.num = 0
        self.stream_captrue = False

        sub_data_list = []
        if self.args.ignore_rgb == False:   sub_data_list.append(message_filters.Subscriber(self.args.rgb_topic, Image))
        if self.args.ignore_depth == False: sub_data_list.append(message_filters.Subscriber(self.args.depth_topic, Image))
        if self.args.ignore_depth == False: sub_data_list.append(message_filters.Subscriber(self.args.pcd_topic, PointCloud2))

        if len(sub_data_list) < 1:
            print('No received data list')
            exit()
        ts = message_filters.ApproximateTimeSynchronizer(sub_data_list, 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback_func)

    def capture_data(self, datalist):
        for data in datalist:        
            if data._type == 'sensor_msgs/Image':
                if data.encoding == "rgb8": 
                    img = ros_numpy.numpify(data)
                    if self.args.bgr_order == False:
                        img = img[..., (2,1,0)]
                    data_type = 'rgb'
                elif data.encoding == "16UC1":
                    img = ros_numpy.numpify(data)
                    data_type = 'dep'
                if img.any() == None: continue
                save_str = self.args.save_dir + '/' + str(self.num) + '_' + data_type + '.png'
                cv2.imwrite(save_str, img)

            elif data._type =='sensor_msgs/PointCloud2':
                pcd = ros_numpy.numpify(data)
                o3_pcd = o3.geometry.PointCloud()
                
                pcd_points = np.zeros((pcd.shape[0], pcd.shape[1], 3))
                pcd_points[..., 0] = pcd['x']
                pcd_points[..., 1] = pcd['y']
                pcd_points[..., 2] = pcd['z']
                o3_pcd.points = o3.utility.Vector3dVector(pcd_points.reshape(-1,3))
                if pcd['rgb'] is not None:
                    pcd = ros_numpy.point_cloud2.split_rgb_field(pcd)
                    pcd_colors = np.zeros((pcd.shape[0], pcd.shape[1], 3))
                    pcd_colors[..., 0] = pcd['r']/255
                    pcd_colors[..., 1] = pcd['g']/255
                    pcd_colors[..., 2] = pcd['b']/255
                    o3_pcd.colors = o3.utility.Vector3dVector(pcd_colors.reshape(-1,3))
                save_str = self.args.save_dir + '/' + str(self.num) + '.ply'
                o3.io.write_point_cloud(save_str, o3_pcd)
            
        self.num += 1

    def callback_func(self, *datalist):
        num_data = len(datalist)
        
        if num_data < 1: 
            print ("No received data!")
            return
        
        if self.stream_captrue == True:
            self.capture_data(datalist)
            print(f'Capture data - {self.num} -')

        if self.args.video_mode == False:
            if self.kb.kbhit():
                c = self.kb.getch()
                if c == 's':
                    self.capture_data(datalist)
                    print(f'Capture data - {self.num} -')
                elif c == 'r':
                    self.num = 0
                    print(f'Reset Number')
                elif c == 'q':
                    sys.exit()
        else:
            if self.kb.kbhit():
                c = self.kb.getch()
                if c == 's':
                    self.stream_captrue = True
                elif c == 'p': # pause
                    self.stream_captrue = False
                elif c == 'r':
                    self.num = 0
                    print(f'Reset Number')
                elif c == 'q':
                    sys.exit()


def arg_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_mode", action='store_true')
    parser.add_argument("--rgb_topic", default="/camera/color/image_rect_color", type=str)
    parser.add_argument("--depth_topic", default="/camera/depth/image_rect_raw", type=str)
    parser.add_argument("--pcd_topic", default="/camera/depth_registered/points", type=str)
    parser.add_argument("--bgr_order", action='store_true')
    parser.add_argument("--ignore_rgb", action='store_true')
    parser.add_argument("--ignore_depth", action='store_true')
    parser.add_argument("--ignore_pcd", action='store_true')
    parser.add_argument("--save_dir", default=str(datetime.datetime.now()), type=str)
    return parser.parse_args()

def main():

    rospy.init_node('capture_data')
    cd = Capture_data()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()