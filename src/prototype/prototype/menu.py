import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import keyboard
from multiprocessing import Process
from subprocess import *
from time import sleep
import os

LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'

class LeaderPublisher(Node):

    def __init__(self):
        super().__init__('LeaderPublisher')

        self.publisher = self.create_publisher(String, 'test_pose', 10)
        
        self.title = "- Swarm Manager -\n"
        self.menu_items = ["Start Simulation (ID:0)", "Send Goal To Leader (ID:1)"]
        self.menu_calls = {'0':self.start_simulation, '1':self.send_message}
        self.formation_id = {'0':"triangle", '1':"diamond", '2':"landscape", '3':"vertical"}

        self.sleep_timer = 2

        self.simulation_started = True

        self.options_menu()

    def send_message(self):

        self.clear_all()
        print(self.title)
        user_input_x = 0
        user_input_y = 0
        user_input_formation = 0
        while True:
            user_input_x = input("Enter an X coordinate goal: ")
            try:
                user_input_x = float(user_input_x)
            except:
                print("! Please enter an integer value !")
                sleep(self.sleep_timer)
                self.clear_line(2)
                continue
            break
        
        while True:
            user_input_y = input("Enter a Y coordinate goal: ")
            try:
                user_input_y = float(user_input_y)
            except:
                print("! Please enter an integer value !")
                sleep(self.sleep_timer)
                self.clear_line(2)
                continue
            break

        while True:
            print("\nTriangle: 0\nDiamond (4 robot limit): 1\nHorizontal Line: 2\nVertical Line: 3")
            user_input_formation = input("Enter a formation ID: ")
            try:
                int(user_input_formation)
            except:
                print("! Please enter an integer value !")
                sleep(self.sleep_timer)
                self.clear_line(2)
                continue
            try:
                self.formation_id[user_input_formation]
            except:
                print("! Please enter the formation ID number !")
                sleep(self.sleep_timer)
                self.clear_line(2)
                continue
            break

        msg = {'0':{'x':user_input_x,'y':user_input_y, 'formation':self.formation_id[user_input_formation]}}
        string_msg = String()
        string_msg.data = json.dumps(msg)
        self.publisher.publish(string_msg)

        print("\nNew goal sent to leader\n\nReturning to menu...")
        sleep(self.sleep_timer)
        
    def clear_line(self, clear_total = 1):
        for i in range(clear_total):
            print(LINE_UP, end=LINE_CLEAR)
    
    def clear_all(self):
        os.system('clear')

    def start_simulation(self):
        world_name = ""
        while True:
            self.clear_all()
            print(self.title)
            print("Obstacle World: 0\nEmpty World: 1")
            world = input("\n\nEnter world ID: ")
            if world == "0":
                world_name = "obstacle_test"
            elif world == "1":
                world_name = "empty"
            else:
                print("! Please enter the world ID number !")
                sleep(self.sleep_timer)
                continue
            break

        call(["gnome-terminal","-x","ros2","launch","swarmbot","launch_sim_multi.launch.py","world:=./src/swarmbot/worlds/"+world_name+".world", "verbose:=true"], stdin=PIPE, stderr=PIPE, stdout=PIPE)
        self.simulation_started = True


    def options_menu(self):
        

        while True:
            self.clear_all()

            print(self.title)

            for item in self.menu_items:
                print(item)
            
            user_input = input("\nEnter Menu ID: ")

            try:
                int(user_input)
            except:
                print("! Please enter a menu ID number !")
                sleep(self.sleep_timer)
                continue

            try:
                if int(user_input) != 0 and not self.simulation_started:
                    print("! Please start the simulation before accessing this menu item !")
                    sleep(self.sleep_timer)
                    continue
                self.menu_calls[user_input]()
            except:
                print("! Please enter a valid menu ID !")
                sleep(self.sleep_timer)
        

def main(args=None):
    rclpy.init(args=args)

    publisher_node = LeaderPublisher()
    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    
    rclpy.shutdown()