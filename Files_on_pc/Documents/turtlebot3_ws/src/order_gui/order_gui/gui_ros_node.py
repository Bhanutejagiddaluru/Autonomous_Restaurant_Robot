import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32

from PyQt5 import QtGui


from .order_gui import Ui_Form

import json



# Define the kitchens and their food items in lowercase
kitchen_items = {
    'K1': {'donut', 'egg', 'chicken'},
    'K2': {'peach', 'lemon', 'orange'},
    'K3': {'applejuice', 'orangejuice', 'tomatojuice'}
}

# Maps and encodings
kitchen_map = {'K1': '1', 'K2': '2', 'K3': '3'}
table_map = {'table_1': '1', 'table_2': '2', 'table_3': '3'}
orientation_map = {'front': '1', 'back': '2'}
item_code_map = {
    'donut': '1', 'egg': '2', 'chicken': '3',
    'peach': '4', 'lemon': '5', 'orange': '6',
    'applejuice': '7', 'orangejuice': '8', 'tomatojuice': '9'
}

trip_list = []
order_number = 0
last_order = 0
item_in_last_order = 0


def process_orders(orders_json):
    # orders = json.loads(orders_json)
    global order_number

    order = orders_json
    
    table_number = table_map[order['table_name']]
    for index, food in enumerate(order['food_list']):
        item_name = next(iter(food)).lower()
        orientation = orientation_map[food['Orientation']]
        kitchen_number = get_kitchen_for_item(item_name)
        if kitchen_number:
            food_code = item_code_map[item_name]
            combined_code = food_code + orientation + kitchen_number + table_number
            trip_list.append([order_number, index,item_name, combined_code])

    order_number +=1
    
    return 


def get_kitchen_for_item(item):
    for kitchen, items in kitchen_items.items():
        if item in items:
            return kitchen_map[kitchen]
    return None

class MyGuiNode(Node):
    def __init__(self, ui):
        super().__init__("ros2_hmi_node")
        self.ui = ui

        # Define our GUI button callbacks here
        # self.ui.order.clicked.connect(self.publisher_ros2_callback)

        self.ui.order.clicked.connect(self.order_confirm)


        self.ui.table1.clicked.connect(self.table1)
        self.ui.table2.clicked.connect(self.table2)
        self.ui.table3.clicked.connect(self.table3)


        self.ui.chicken.clicked.connect(self.chicken)
        self.ui.egg.clicked.connect(self.egg)
        self.ui.pizza.clicked.connect(self.donut)

        self.ui.orange.clicked.connect(self.orange)
        self.ui.lemon.clicked.connect(self.lemon)
        self.ui.peach.clicked.connect(self.peach)

        self.ui.applejuice.clicked.connect(self.applejuice)
        self.ui.orangejuice.clicked.connect(self.orangejuice)
        self.ui.tomatojuice.clicked.connect(self.tomatojuice)

        # Define ROS2 publishers and subscribers here
        self._btn_publisher = self.create_publisher(String, '/chatter', 10)
        self._subscriber    = self.create_subscription(Int32, '/order_status', self.subscriber_callback, 1)

        # Define internal member variables
        self._count         = 0
        self._current_selection = {}
        self._ordered_food = []
        self._kitchen = ''
        self._current_selection['food_list'] = []
        self._current_selection['table_name'] =''

        # Sanity check
        self.get_logger().info("Node set up properly.")
        self.direction = True

    def table1(self):
        self._current_selection['table_name'] = 'table_1'
        self.ui.selectedfood.setText(json.dumps(self._current_selection))

    def table2(self):
        self._current_selection['table_name'] = 'table_2'
        self.ui.selectedfood.setText(json.dumps(self._current_selection))

    def table3(self):
        self._current_selection['table_name'] = 'table_3'
        self.ui.selectedfood.setText(json.dumps(self._current_selection))


    def chicken(self):
        food_selectiion = {f'chicken': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)

        self.ui.selectedfood.setText(json.dumps(self._current_selection))

        self.direction = not self.direction


    def egg(self):
        food_selectiion = {f'egg': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def donut(self):
        food_selectiion = {f'donut': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def orange(self):
        food_selectiion = {f'orange': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def lemon(self):
        food_selectiion = {f'lemon': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def peach(self):
        food_selectiion = {f'peach': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def applejuice(self):
        food_selectiion = {f'applejuice': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def orangejuice(self):
        food_selectiion = {f'orangejuice': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def tomatojuice(self):
        food_selectiion = {f'tomatojuice': False}
        food_selectiion['Orientation'] = 'front' if self.direction else 'back'
        self._current_selection['food_list'].append(food_selectiion)
        self.ui.selectedfood.setText(json.dumps(self._current_selection))
        self.direction = not self.direction

    def order_confirm(self):

        if len(self._current_selection['food_list'] ) and self._current_selection['table_name'] != '':
            # orders_json = json.dumps(self._current_selection)


            result = process_orders(self._current_selection)
            
            self._ordered_food.append(self._current_selection.copy())

            self.ui.orderededfood.setText(json.dumps(self._ordered_food))

            self._current_selection['food_list'] = []
            self._current_selection['table_name']  = ''
            
            self.ui.selectedfood.setText(json.dumps(self._current_selection))


    # Define the member functions here
    def publisher_ros2_callback(self):
        """
            This function is executed when we click the button
            on the GUI. So, this button should publish a message
            on the /chatter topic. This is what we will implement
            in the function body.
        """

        if len(trip_list):

            # This part is standard ROS2 publisher code 
            msg = String()
            msg.data  = trip_list.pop(0) #json.dumps(self._current_selection)
            self._btn_publisher.publish(msg)
            self.get_logger().info(f"[Publisher] I sent: {msg.data}")

    
    def subscriber_callback(self, msg):
        """
            The objective of this function is to set the text of the
            particular textLabel element when there is new data being
            received. Check the Qt documentation for more things you 
            can do to display the data.
        """
        global last_order, item_in_last_order
        temp = msg.data
        self.get_logger().info(f"[Subscriber] I heard: {temp}")
        
        if item_in_last_order and len(trip_list)>last_order-1:
            self._ordered_food[trip_list[last_order-1][0]]['food_list'][trip_list[last_order-1][1]][trip_list[last_order-1][2]] = True

            if item_in_last_order ==2:
                self._ordered_food[trip_list[last_order-2][0]]['food_list'][trip_list[last_order-2][1]][trip_list[last_order-2][2]] = True
            
            
            
            self.ui.orderededfood.setText(json.dumps(self._ordered_food))
        
        # self.ui.label_subscriber.setText(f"{msg.data}")

        # self.publisher_ros2_callback()

        item_in_last_order = 0

        if len(trip_list)>last_order:
            self._count +=1
            trip_code = trip_list[last_order][3]
            last_order +=1
            item_in_last_order=1

            if len(trip_list)>last_order:
                trip_code = trip_code+ trip_list[last_order][3]
                last_order +=1

                item_in_last_order = 2

            msg = String()
            msg.data  = f"Trip {self._count}: {trip_code}" #json.dumps(self._current_selection)
            self._btn_publisher.publish(msg)
            self.get_logger().info(f"[Publisher] I sent: {msg.data}")

            

            



