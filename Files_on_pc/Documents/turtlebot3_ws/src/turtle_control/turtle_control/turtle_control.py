import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String, Int32
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor


kitchen_map = {
    '11': [0.06503505465284291, 0.8756929588729234, 0.0, 0.0, 0.0, 0.9999890282433566, 0.004684377536821601],
    '21': [0.05690508149047848, 0.858816331944444, 0.0, 0.0, 0.0, -0.023771004682411223, 0.9997174297452199],
    '12': [0.4027947850252832, -0.09547408696736888, 0.0, 0.0, 0.0, -0.9982622323764475, 0.058928052833870315],
    '22': [0.018723721014854613, -0.9480287575600368, 0.0, 0.0, 0.0, 0.11747369534592486, 0.9930759945249773],
    '13': [0.3274517369485671, -0.08004891797288488, 0.0, 0.0, 0.0, -0.09593089886463534, 0.9953879960312074],
    '23': [0.4936577695473061, -0.056662814011639195, 0.0, 0.0, 0.0, 0.9977838952864386, 0.06653794636913031]
}

table_map = {
    '11': [1.999624584634467, 0.7448253781145789, 0.0, 0.0, 0.0, 0.016039605015515452, 0.999871357260996],
    '21': [2.0152792743270753, 0.7365924439328901,  0.0, 0.0, 0.0, 0.9986852077861696, 0.05126261551164903],
    '12': [1.9938867894566579, -0.14450063811524144, 0.0, 0.0, 0.0, 0.10277319471696938, 0.9947048157356372],
    '22': [1.9776959542485837, -0.117392582421665, 0.0, 0.0, 0.0, -0.9969221562622443, 0.07839779559042007],
    '13': [1.8756654105755193, -1.1463137926491649, 0.0, 0.0, 0.0, -0.04424751995548917, 0.9990205988756131],
    '23': [1.9577547752489135, -1.1297988152076013, 0.0, 0.0, 0.0, -0.9930754036819531, 0.11747868999067845]
}

kitchen_dr_map = {
    '1': [0.3659018329199358, 0.8182915951253283, 0.0, 0.0, 0.0, 0.9961367749339762, 0.08781529265530397],
    '2': [0.3534979024531007, -0.9917606511081662, 0.0, 0.0, 0.0, -0.9982725920283256, 0.05875229361521215],
    '3': [0.25326470211453206, -0.0819821271752251, 0.0, 0.0, 0.0, -0.04243861694275646, 0.9990990760640238]
}

initial_map_co = [0.11292153874933289, -0.0693849554962082, 0.0, 0.0, 0.0, -0.03298367047425209, 0.9994558907135652]

id_to_food = { '1': 'Donut', '2': 'Egg', '3':'Chicken',
     '4': 'Peach', '5': 'Lemon',  '6': 'Orange',
    '7': 'Apple_juice', '8': 'Orange_juice', '9': 'Tomato_juice'
}


def map_to_destinations(code):
    kitchen_co1 = kitchen_map[code[1:3]]
    table_co1 = table_map[code[1] + code[3]]
    kitchen_dr1 = kitchen_dr_map[code[2]]
    food1 = code[0]
    if len(code)  > 4:
        kitchen_co2 = kitchen_map[code[5:7]]
        table_co2 = table_map[code[5] + code[7]]
        kitchen_dr2 = kitchen_dr_map[code[6]]
        food2 = code[4]
    else:
        kitchen_co2 = None
        table_co2 = None
        kitchen_dr2 = None
        food2 = None
    print("***********************************************")
    print(kitchen_co2)

    print(table_co2)
    return food1, kitchen_dr1, kitchen_co1, table_co1, food2, kitchen_dr2, kitchen_co2, table_co2


class RestaurantRobot(Node):
    def __init__(self):
        super().__init__('restaurant_robot')
        self.navigator = BasicNavigator()
        self.subscription_front = None
        self.subscription_back = None
        self.subscription_order = self.create_subscription(String, '/chatter', self.ui_code, 10)
        self.status_publisher = self.create_publisher(Int32, '/order_status', 10)
        self.update_status_publisher = self.create_publisher(String, '/order_status_update', 10)
        self.subscription_front = self.create_subscription(Bool, 'ir_front', self.ir_front_callback, 10)
        self.subscription_back = self.create_subscription(Bool, 'ir_back', self.ir_back_callback, 10)
        self.food_type = self.create_subscription(String, '/food_type', self.image_type, 10)
        self.subscription_emergency = self.create_subscription(String, '/emergency', self.emergency_callback, 10)
        self.timer = self.create_timer(2, self.timer_callback)
        self.front_food_picked_up = False
        self.back_food_picked_up = False
        self.front_food_detected = False
        self.back_food_detected = False
        self.front_food_placed = False
        self.back_food_placed = False
        self.kitchen_dr1 = None 
        self.kitchen_co1 = None 
        self.table_co1 = None 
        self.kitchen_dr2 = None 
        self.kitchen_co2 = None 
        self.table_co2 = None
        self.next_process_number = 0
        self.emergency_stop = 0
        self.food1 = None
        self.food2 = None
        self.navigator.waitUntilNav2Active()
        self.i = 0
    
    def emergency_callback(self, msg):
        if msg.data == "ee":  # Checking if the stop signal is received
            self.emergency_stop == 1
            self.navigator.cancelTask()  # Cancel all navigation tasks
            
            self.get_logger().info('Emergency stop activated!')
        
        elif msg.data[0] == '1':
            self.emergency_stop == 0
            self.go_to_kitchen(kitchen_map[msg.data[1:3]])
            
        elif msg.data[0] == '2':
            self.emergency_stop == 0
            self.go_to_kitchen_door(kitchen_dr_map[msg.data[1]])
            
        elif msg.data[0] == '3':
            self.emergency_stop == 0
            self.go_to_table(table_map[msg.data[1:3]])
            


    def send_updated_status(self, value):
        msg = String()
        msg.data = value
        self.update_status_publisher.publish(msg)

    def image_type(self, msg):
        if msg.data:
            # receving an encoded data from the robot terminal (food,code)
            if self.next_process_number == 1 and self.emergency_stop == 0:
                if msg.data[1] == '2': # Green/Yellow 

                    self.send_updated_status(f'Going to kitchen {self.trip_code[2]}; Got signal')
                    # Navigating to Kitchen 
                    print('**************kitchen_co1*******************')
                    self.go_to_kitchen(self.kitchen_co1)
                    print("kitchen_co1 = ", self.kitchen_co1)

                    self.send_updated_status(f'Wating to Kitchen {self.trip_code[2]}; Waiting for {id_to_food[self.trip_code[0]]}')

                    self.next_process_number = 2  # Object detection kd1
            
            elif self.next_process_number == 2 and self.emergency_stop == 0:
                if msg.data[0] == self.food1: # same type food, waiting for food detection
                    self.front_food_detected = True
                    
                    

            elif self.next_process_number == 3 and self.emergency_stop == 0:
                if msg.data[1] == '2': # Green/Yellow 
                    print('**************kitchen_co2*******************')

                    self.send_updated_status(f'Going to Kitchen {self.trip_code[6]}; Got signal')
                    self.go_to_kitchen(self.kitchen_co2)

                    self.send_updated_status(f'Waiting at Kitchen {self.trip_code[6]}; Waiting for {id_to_food[self.trip_code[4]]}')
                    print("kitchen_co2 = ", self.kitchen_co2)
                    self.next_process_number = 4  # Object detection kd1
                        
                    print('**************out_kitchen_co2*******************')
            
            elif self.next_process_number == 4 and self.emergency_stop == 0:
                if msg.data[0] == self.food2: 
                    self.back_food_detected = True
                    # Navigate to the table
                    

    def ir_subscription(self):
        if self.subscription_front is None:
            print("creating ir_front subscription")
            self.subscription_front = self.create_subscription(Bool, 'ir_front', self.ir_front_callback, 10)
        else:
            print("ir_front subscription already active")
        if self.subscription_back is None:
            print("creating ir_back subscription")
            self.subscription_back = self.create_subscription(Bool, 'ir_back', self.ir_back_callback, 10)
        else:
            print("ir_back subscription already active")
    
    def cleanup_subscriptions(self):
        if self.subscription_front:
            print("removed ir_front subscription")
            self.destroy_subscription(self.subscription_front)
            self.subscription_front = None
        if self.subscription_back:
            print("removed ir_back subscription")
            self.destroy_subscription(self.subscription_back)
            self.subscription_back = None
    
    def timer_callback(self):
        if self.next_process_number == 0:
            msg = Int32()
            msg.data = self.i 
            self.status_publisher.publish(msg)
            self.get_logger().info('Publishin: "%s"' %msg.data)
            self.i +=1



    def ir_front_callback(self, msg):
        print(" ir_front_call back")
        if msg.data and self.next_process_number == 2 and self.front_food_detected and self.emergency_stop == 0:
            self.front_food_placed = True
            if self.kitchen_co2:
                self.send_updated_status(f'Going to Kitchen {self.trip_code[6]} Door; Got {id_to_food[self.trip_code[0]]}')
                print('**************kitchen_dr2*******************')
                self.go_to_kitchen_door(self.kitchen_dr2)
                print("kitchen_dr2 = ", self.kitchen_dr2)

                self.send_updated_status(f'Wating to Kitchen {self.trip_code[6]} Door; Waiting for light')

                self.next_process_number = 3  # Light detection kd2
            else:
                print('**************out_of_kitchens*******************')

                # Navigate to the table
                print('**************going to table_co1*******************')
                self.send_updated_status(f'Going to Table {self.trip_code[3]}; Got {id_to_food[self.trip_code[0]]}')
                self.go_to_kitchen(self.table_co1)
                self.send_updated_status(f'Waiting at Table {self.trip_code[3]}; Waiting for {id_to_food[self.trip_code[0]]} to remove from front')
                print("table_co1 = ", self.table_co1)
                self.next_process_number = 5  # Front IR sensor

        if not msg.data and self.next_process_number == 5 and self.emergency_stop == 0:
            self.front_food_detected = False
            self.get_logger().info('Customer has taken the food.')

            self.next_process_number = 6  # To go to table two

            if self.table_co2:

                self.send_updated_status(f'Going at Table {self.trip_code[7]}; Front food removed')
                print('**************going to table_co2*******************')
                
                self.go_to_kitchen_door(self.table_co2)

                self.send_updated_status(f'Waiting at Table {self.trip_code[7]}; Waiting for {id_to_food[self.trip_code[4]]} to remove from back')
                print("table_co2 = ", self.table_co2)
                print("waiting for ir back subscriptions")
                self.next_process_number = 7  # Back IR sensor
            else:

                self.send_updated_status(f'Front food removed; Going home')
                self.go_to_kitchen_door(initial_map_co)
                self.next_process_number = 0 # Take next order

    def ir_back_callback(self, msg):
        print(" ir_back_call back")
        # Handle back IR sensor if necessary, logging for now
        if not msg.data and self.next_process_number == 7 and self.emergency_stop == 0:
            self.back_food_detected = False
            self.get_logger().info("Customer has taken the food.")

            self.send_updated_status(f'Back food is removed; Going home')
            self.go_to_kitchen_door(initial_map_co)
            self.next_process_number = 0 # Take next order
        
        if msg.data and self.next_process_number == 4 and self.back_food_detected and self.emergency_stop == 0:
            print('**************going to table_co1*******************')
            self.back_food_placed = True

            self.send_updated_status(f'Going to Table {self.trip_code[3]}; Got {id_to_food[self.trip_code[4]]}')
            self.go_to_kitchen(self.table_co1)

            self.send_updated_status(f'Waiting at Table {self.trip_code[3]}; Waiting for {id_to_food[self.trip_code[0]]} to remove from front')
            print("table_co1 = ", self.table_co1)

            # print("waiting for ir front subscriptions")

            self.next_process_number = 5  # Front IR sensor

    def go_to_kitchen_door(self, kitchen_dr):
        kitchen_dr_pose = self.set_goal_pose(*kitchen_dr)
        print('**************kitchen_dr going to pose*******************')
        self.navigator.goToPose(kitchen_dr_pose)
        time.sleep(20)  # Sleep for 2 minutes
        # led signal
        print('**************waiting for food to prepre *******************')

    def go_to_kitchen(self, kitchen_co):
        kitchen_pose = self.set_goal_pose(*kitchen_co)
        print('**************kitchen_co going to pose*******************')
        self.navigator.goToPose(kitchen_pose)
        time.sleep(10)  # Sleep for 2 minutes
        # object detection
        print('**************waiting for the order food iten*******************')

    def go_to_table(self, table_co):
        print('**************started  navigation to table_co*******************')
        table_pose = self.set_goal_pose(*table_co)
        print('**************table_co going to pose*******************')
        self.navigator.goToPose(table_pose)
        # When goal is reached, manage IR subscriptions
        print('**************creating ir subscribtion*******************')
        # self.ir_subscription()

    # Adjust the callback for navigation completion to manage sensor subscriptions
    def navigation_completion_callback(self):
        if not self.front_food_picked_up:
            self.ir_subscription()  # Ensure IR sensors are subscribed to once goal is reached
        else:
            self.cleanup_subscriptions()  # Clean up subscriptions if food is picked up


    def ui_code(self, msg):
        if msg.data and self.next_process_number == 0:
            # decoded information
            self.trip_code = msg.data.split(' ')[2]
            self.food1, self.kitchen_dr1, self.kitchen_co1, self.table_co1, self.food2, self.kitchen_dr2, self.kitchen_co2, self.table_co2 = map_to_destinations(msg.data.split(' ')[2])
            
            # Navigating to Kitchen door
            print('**************kitchen_dr1*******************')
            self.send_updated_status(f'Going to Kitchen {self.trip_code[2]} Door; Moving')
            self.go_to_kitchen_door(self.kitchen_dr1)
            self.send_updated_status(f'Wating to Kitchen {self.trip_code[2]} Door; Waiting for light')
            print("kitchen_dr1 = ", self.kitchen_dr1)
            
            self.next_process_number = 1  # Light detection kd1
            
            


    def set_goal_pose(self, x, y, z, ox, oy, oz, ow):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = ox
        goal_pose.pose.orientation.y = oy
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = ow
        return goal_pose


def main(args=None):
    rclpy.init(args=args)
    restaurant_robot = RestaurantRobot()
    # restaurant_robot.ir_subscription()
    executor = MultiThreadedExecutor()
    # rclpy.spin(fibonacci_action_server, executor)
    rclpy.spin(restaurant_robot, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()