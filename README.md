# A Simple Publisher and Subscriber

## 1 Create a package

Run the package creation command by going to ros2 ws/src:

```
ros2 pkg create --build-type ament_python py_pubsub
```
Your terminal will display a message confirming the creation of your package py pubsub and all of its necessary files and folders.

## 2 Write the publisher node

You can download the example talker code by going to ros2 ws/src/py pubsub/py pubsub and typing the following command:
```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/19.PNG)

The __init.py file will now be followed by a new one called publisher member function.py.
Open the file in your preferred text editor.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
## 2.1 Add dependencies

Return one level up to the setup.py, setup.cfg, and package.xml files that have been produced for you in the ros2 ws/src/py pubsub directory.

Open package.xml in your text editor, and make sure the description>, maintainer>, and license> tags are full.
```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

After the lines above that correspond to the import declarations for your node, add the following dependencies:
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/20.PNG)

This states that rclpy and std msgs are necessary for the package's code to run.

Make care to save the file.

## 2.2 Add an entry point

Take a look at setup.py. Make sure to check your package.xml one more time to make sure the maintainer, maintainer email, description, and license columns match:
```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Add the next line to the entry points field between the console scripts brackets:
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/21.PNG)

Remember to save.

## 2.3 Check setup.cfg

The following details should be included by default in the setup.cfg file:
```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
Tell setuptools to put your executables in the lib directory so that ros2 run knows where to find them.

You could create your package right now, source the local setup files, and launch it if you wanted to see the full system in operation. Let's first, though, establish the subscriber node.

## 3 Write the subscriber node

Returning to ros2 ws/src/py pubsub/py pubsub will allow you to create the following node. The following code should be entered into your terminal:

```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
Now, the directory must include the following files:

```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

Now open your text editor and navigate to subscriber member function.py.
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3.1 Add an entry point

Reopen setup.py and put the publisher's entry point below the subscriber node's entry point. Now, the entry points field ought to read:
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/22.PNG)

Your pub/sub system should be functional when the file has been saved.

## 4 Build and Run

On your ROS 2 system, the rclpy and std msgs packages are presumably already installed. The ideal technique is to run rosdep in the workspace's root directory (ros2 ws) before building to see if any dependencies are missing:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select py_pubsub
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/23.PNG)

Navigate to ros2 ws in a new terminal, then source the setup files:
```
. install/setup.bash
```

Now run the talker node:

```
ros2 run py_pubsub talker
```
The terminal should start sending the following info messages in 0.5 seconds:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/24.PNG)

In 0.5 seconds, the terminal should begin transmitting the following informational messages:

```
ros2 run py_pubsub listener
```
The listener will start writing messages to the console starting at the publisher's current message count as follows:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/25.PNG)

Ctrl+C will stop the nodes from rotating in each terminal.

# A Simple Service and Client 

## 1 Create a package
Run the package creation command by going to ros2 ws/src:
```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

You will see confirmation from your terminal that your package py srvcli and all of its necessary files and folders have been created.

![image](https://github.com/thapapradeep884/IMAGE/blob/main/26.PNG)

## 1.1 Update (package.xml)

Because you used the —dependencies option when generating the package, you don't need to manually add dependencies to package.xml.

But as always, don't forget to include the description, license information, and the name and email of the maintainer in package.xml.
```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/27.PNG)

## 1.2 Update (setup.py)

The following details should be added to the setup.py file's description, maintainer, maintainer email, and license fields:
```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/28.PNG)


## 2 Write the service node

In the ros2 ws/src/py srvcli/py srvcli directory, make a new file called service member function.py, and then paste the following code inside:

```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.1 Add an entry point

For the ros2 run command to be able to run your node, the entry point must be added to setup.py (located in the ros2 ws/src/py srvcli directory).

In between the "console scripts" brackets, the following line should be added:

```
'service = py_srvcli.service_member_function:main',
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/29.PNG)

## 3 Write the client node

In the ros2 ws/src/py srvcli/py srvcli directory, make a new file called client member function.py, and then paste the following code inside:

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 3.1 Add an entry point
Similar to how the service node needs an entry point, the client node also needs one.

The entry points column in your setup.py file must be formatted as follows:
```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/30.PNG)

## 4 Build and Run

Running rosdep in the workspace's root directory (ros2 ws) is a good idea to see if any dependencies are missing before building:

```
rosdep install -i --from-path src --rosdistro foxy -y
```
Go back to ros2 ws, the workspace's root, and create your new package:

```
colcon build --packages-select py_srvcli
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/31.PNG)

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```
Run the service node right now:
```
ros2 run py_srvcli service
```
The node will hold off until the client makes a request.

Re-source the setup files from ros2 ws in a new terminal. The client node, any two integers, and a space between them.

```
ros2 run py_srvcli client 2 3
```

If you chose options 2 and 3 as an illustration, the customer would receive a response similar to this:

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/32.PNG)

You should return to the terminal where your service node is running. As you can see, it published the following log statements after receiving the request:

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/33.PNG)

Ctrl+C will stop the nodes from rotating in each terminal.

# Creating custom msg and srv files

## 1 Create a new package

Navigate into ros2_ws/src and run the package creation command:

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

Your terminal will display a message confirming the establishment of your package tutorial interfaces and every file and folder it needs. Due to the fact that pure Python packages cannot currently generate.msg or.srv files, it should be stated that this is a CMake package. A custom interface that you create in a CMake package can be used by a Python node, which will be covered in the last section.

It's best practice to keep.msg and.srv files separated in different places within a package. The directories should be made in the ros2 ws/src/tutorial interfaces.

```
mkdir msg

mkdir srv
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/35.png)

## 2 Create custom definitions

## 2.1 msg definition 

In the tutorial interfaces/msg directory that you just created, make a new file named Num.msg, and then add a single line of code describing the data structure in Num.msg:

```
int64 num
```
This custom message sends a single value, the 64-bit integer num.

In the directory you just created for the tutorial interfaces/msg, make a new file called Sphere.msg and put the following information in it:

```
geometry_msgs/Point center
float64 radius
```
A message from a different message package—in this case, geometry msgs/Point—is used in this custom message.

## 2.2 srv definition

In the instructional interfaces/srv directory that you just created, add a new file with the name AddThreeInts.srv with the following request and response structure:

```
int64 a
int64 b
int64 c
---
int64 sum
```
This custom service accepts the three integers a, b, and c and provides the answer's integer sum.

![image](https://github.com/thapapradeep884/IMAGE/blob/main/36.png)

## 3 CMakeLists.txt

The interfaces you defined can be used in C++ and Python by adding the following lines to CMakeLists.txt to convert them into language-specific code:

```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

## 4 package.xml

To package.xml, these lines should be added.

```
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5 Build the tutorial_interfaces package

Now that all of the pieces of your custom interfaces package are in place, you can build it. Run the following command in the workspace's root directory (/ros2 ws):


```
colcon build --packages-select tutorial_interfaces
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/37.png)

Now, other ROS 2 applications will be able to locate the interfaces.

## 6 Confirm msg and srv creation

To source it in a new terminal, issue the following command from within your workspace (ros2 ws):

```
. install/setup.bash
```

You can now check that your interface creation was successful by using the ros2 interface show command:

```
ros2 interface show tutorial_interfaces/msg/Num
```
should return:
```
int64 num
```
And
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
should return:
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```
And
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
should return:

```
int64 a
int64 b
int64 c
---
int64 sum
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/38.png)
## 7 Test the new interfaces

For this stage, you can make use of the packages you created earlier. A few simple adjustments to the nodes, CMakeLists, and package files will enable you to use your new interfaces.

## 7.1 Testing Num.msg with pub/sub

Making a few little tweaks to the publisher/subscriber package created in a previous tutorial (in C++ or Python) will allow you to observe Num.msg in action. You'll be changing the default string message to a numerical one, which will result in a little change in the output.

Publisher:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml:

Add the following line:

```
<exec_depend>tutorial_interfaces</exec_depend>
```

Create the package after making the aforementioned adjustments and saving your work.

```
colcon build --packages-select py_pubsub
```
On Windows:

```
colcon build --merge-install --packages-select py_pubsub
```

Then open two fresh terminals, run source ros2 ws in each, and do the following:

```
ros2 run py_pubsub talker

```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/39.png)

```
ros2 run py_pubsub listener

```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/40.png)

As opposed to the text it previously published as Num.msg only relays an integer, the talker should only be broadcasting integer values:

```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

## 7.2 Testing AddThreeInts.srv with service/client

AddThreeInts.srv can be used by making a few small changes to the service/client package created in a previous tutorial (in C++ or Python). Because you'll be changing from the initial two integer request srv to a three integer request srv, the result will be drastically different.


Service:

```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Client:

```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml:

Add the following line:

```
<exec_depend>tutorial_interfaces</exec_depend>
```

Create the package after making the aforementioned adjustments and saving your work.

```
colcon build --packages-select py_srvcli
```

On Windows:

```
colcon build --merge-install --packages-select py_srvcli
```

then launch two fresh terminals, enter source ros2 ws into each, and execute:

```
ros2 run py_srvcli service
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/41.png)

```
ros2 run py_srvcli client 2 3 1
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/42.png)




















