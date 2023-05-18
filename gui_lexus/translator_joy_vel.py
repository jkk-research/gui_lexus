#!/usr/bin/env python
# joy to //lexus3/pacmod/accel_cmd brake and steer
# TODO: test and correct buttons


import rclpy
import pacmod3_msgs.msg as pac_msg
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rclpy.node import Node

class Translator(Node):
    def __init__(self):
        super().__init__('translator')
        self.get_logger().info("Started")
        self.sub1 = self.create_subscription(Joy, 'joy', self.callback, 10)
        self.sub2 = self.create_subscription(Bool, "/lexus3/pacmod/enabled", self.callbackEnabled, 10)
        self.sub1  
        self.sub2  
        
        self.pubA = self.create_publisher(pac_msg.SystemCmdFloat, "/lexus3/pacmod/accel_cmd", 10)
        self.pubB = self.create_publisher(pac_msg.SystemCmdFloat, "/lexus3/pacmod/brake_cmd", 10)
        self.pubS = self.create_publisher(pac_msg.SteeringCmd, "/lexus3/pacmod/steering_cmd", 10)
        self.pubTU = self.create_publisher(pac_msg.SystemCmdInt, "/lexus3/pacmod/turn_cmd", 10)
        self.pubHE = self.create_publisher(pac_msg.SystemCmdInt, "/lexus3/pacmod/headlight_cmd", 10)
        self.pubWI = self.create_publisher(pac_msg.SystemCmdInt, "/lexus3/pacmod/wiper_cmd", 10)
        self.pubSH = self.create_publisher(pac_msg.SystemCmdInt, "/lexus3/pacmod/shift_cmd", 10)
        self.pubHO = self.create_publisher(pac_msg.SystemCmdBool, "/lexus3/pacmod/horn_cmd", 10)
        self.pubF = self.create_publisher(Bool, "/lexus3/pacmod/enable", 10)
        #self.pubE = self.create_publisher(Bool, "/lexus3/pacmod/enabled", 10)

        self.last_published_time = self.get_clock().now()
        self.last_published = None
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joyInit = True
        self.autonomStatusChanged = True
        self.autonomStatus = True
        self.headlight = False
        self.pacmodst = True
        self.get_logger().info("joy translator")
    
    def timer_callback(self):
        if (self.pacmodst == False):
            self.autonomStatus = False
            #self.get_logger().info("Pacmod says off")

    def callbackEnabled(self, message):
        self.pacmodst = message.data
            

    def callback(self, message):
        accelCmd = pac_msg.SystemCmdFloat()
        brakeCmd = pac_msg.SystemCmdFloat()
        steerCmd = pac_msg.SteeringCmd()
        TUCmd = pac_msg.SystemCmdInt()
        HECmd = pac_msg.SystemCmdInt()
        WICmd = pac_msg.SystemCmdInt()
        SHCmd = pac_msg.SystemCmdInt()
        HOCmd = pac_msg.SystemCmdBool()
        accelCmd.header.stamp = self.get_clock().now().to_msg()
        brakeCmd.header.stamp = self.get_clock().now().to_msg()
        steerCmd.header.stamp = self.get_clock().now().to_msg()
        steerCmd.command = message.axes[0] * 5.2  # 6 for local 5.2 for laptop
        accelCmd.command = (message.axes[5] - 1) / -2
        brakeCmd.command = (message.axes[2] - 1) / -2
        if(self.autonomStatus == False):
            if(message.buttons[0]): # start A
                self.autonomStatusChanged = True
                self.autonomStatus = True
                self.get_logger().info("Autonomous mode on")
        TUCmd.command = 1
        if(message.buttons[1]): # stop B
            self.autonomStatusChanged = True
            self.autonomStatus = False
            self.get_logger().info("Autonomous mode off")
        elif(message.buttons[2]): # horn  X
            HOCmd.command = True
            #self.get_logger().info("Horn")
        elif(message.axes[4] > 0): # lights 
            TUCmd.command = 2
            #self.get_logger().info("Left")
        elif(message.axes[4] < 0): # lights 
            TUCmd.command = 0
            #self.get_logger().info("Right")
        #elif(message.axes[5] < 0): # lights 
        #    TUCmd.command = 3
        #    self.get_logger().info("Down")
        elif(message.axes[7] > 0): # lights
            if self.headlight: 
                self.headlight = False
                HECmd.command = 2     
            else:
                self.headlight = True
                HECmd.command = 0            
            self.get_logger().info("Up")
        if(self.autonomStatusChanged):
            accelCmd.clear_override = True
            brakeCmd.clear_override = True
            accelCmd.clear_override = True
            steerCmd.clear_override = True
            TUCmd.clear_override = True
            HECmd.clear_override = True
            WICmd.clear_override = True
            SHCmd.clear_override = True
            HOCmd.clear_override = True
        else:
            accelCmd.clear_override = False
            brakeCmd.clear_override = False
            accelCmd.clear_override = False
            steerCmd.clear_override = False
            TUCmd.clear_override = False
            HECmd.clear_override = False
            WICmd.clear_override = False
            SHCmd.clear_override = False
            HOCmd.clear_override = False
        #//lexus3/pacmod/enable.data
        status = Bool()
        status.data = self.autonomStatus
        brakeCmd.enable = self.autonomStatus
        accelCmd.enable = self.autonomStatus
        steerCmd.enable = self.autonomStatus
        TUCmd.enable = self.autonomStatus
        HECmd.enable = self.autonomStatus
        WICmd.enable = self.autonomStatus
        SHCmd.enable = self.autonomStatus
        HOCmd.enable = self.autonomStatus            
        if(self.joyInit):
            if(message.axes[1] == 0.0):
                accelCmd.command = 0.0 
            else:
                self.joyInit = False
        steerCmd.rotation_rate = 3.3
        accelCmd.header.frame_id = "pacmod"        
        ast = "----"
        if (self.autonomStatus):
            ast = "Auto  "
        else:
            ast = "Driver"
        self.get_logger().info("%s accel brake steer: %.1f %.1f %.1f" %(ast, accelCmd.command, brakeCmd.command, steerCmd.command))
        self.last_published = message
        self.pubA.publish(accelCmd)
        self.pubB.publish(brakeCmd)
        self.pubS.publish(steerCmd)
        self.pubTU.publish(TUCmd)
        self.pubHE.publish(HECmd)
        self.pubWI.publish(WICmd)
        self.pubSH.publish(SHCmd)
        self.pubHO.publish(HOCmd)
        self.pubF.publish(status)


def main(args=None):
    rclpy.init(args=args)

    translator_cmd_joy = Translator()

    rclpy.spin(translator_cmd_joy)

    translator_cmd_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()