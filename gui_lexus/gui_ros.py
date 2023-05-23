import sys
import os
import rclpy
import rclpy.node as Node

import json

class JSONReader:
    def __init__(self, filename):
        self.filename = filename
        self.data = []
        self.readData(self.filename)
    
    def readData(self, filename):
        with open(filename, "r") as f:
            self.data = json.load(f)

def main(args=None):
    rclpy.init(args=args)
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    print(scriptDir)
    rclpy.shutdown()

if __name__ == '__main__':
    main()