## Week 6, Sudip Dhungana

***Contents***
- Managing Dependencies with rosdep
- Creating an action
- Writing an action server and client
- Composing multiple nodes in a single process
- Creating a launch file
- Integrating launch files into ROS2 packages
- Using Substitutions
- Using Event Handlers

### 1.Managing Dependencies with rosdep

**What is rosdep?**

_`rosdep` is ROS's dependency management utility that can work with ROS packages and external libraries and `rosdep` is a command-line utility for identifying and installiing dependencies to build or install a package._

**Use of rosdep tool**

```
sudo rosdep init
rosdep update
```

The given command is used to initialize rosdep and update it to get the latest index.

After that, we can run `rosdep install` to install dependencies. (All)

In order to install dependencies in the root of the workspace with directory `src`, 

```
rosdep install --from-paths src -y --ignore-src
```

### 2.Creating an action

In order to create an action, we should have ROS2 and colcon installed which we already installed previosly. 

Firstly, we have to source the ROS2 installation before doing other tasks.

**2.1. Defining an action**

Actions are defined in `.action` files and is made up of three message definitions: **Request**, **Result**, and **Feedback** which are separated by `---`.

Here , lets take a situation of defining a new action **"Fibonacci"** for computing _Fibonacci sequence_. 

An `action` directory is created in `action_tutorials_interfaces` package:

```
cd action_tutorials_interfaces
mkdir action
```

After that, within `action` directory, we created a file named: `Fibonacci.action` with the following contents:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**2.2. Building an action**

We must pass the definition to the rosidl code to use the new `Fibonacci action` in our code. For that purpose, we should add the following lines to our `CMakeLists.txt` before the `ament_package()` line in the `action_tutorials_interfaces`.

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

And the following dependencies are added to `package.xml`:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

After all these, we will be able to build the package containing the `Fibonacci` action definition as follows:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```

### 3.Writing an action server and client

**3.1. Writing an Action Server**

Here, were create a new file named: `fibonacci_action_server.py` in the home directory and added the following code:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

Once done, we **save** the file and run the action server:

```
python3 fibonacci_action_server.py
```

After that, in another terminal, we use the command line interface to send a goal as follows:
```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

**3.1.1. Publishing Feedback**

_We can make our action server publish feedback for action clients by calling the goal handle's **publsih_feedback()** method._

After replacing the `sequence` variable with a feedback message to store the sequence. After every update of the feedback message in the for-loop, we publish the feedback message:

```python
import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```


After restarting the action server, it is confirmed that feedback is now published by using the command line tool with the `--feedback` option:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

**3.2. Writing an action client**

Similarly, we also scope the action client to a new file named `fibonacci_action_client.py` and the following codes are added to the file:

```python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
    
```

After this step, we can run the action server built earlier using following code:

```
python3 fibonacci_action_server.py
```

And in another terminal, we run the action client:

```
python3 fibonacci_action_client.py
```

The action server executes the goal and we are able to see the messages successfully. 

The action client start up and quickly finish but we don't get any feedback. 


**3.2.1.Getting a result**

We need to set a goal handle for the goal we sent and hence here is the complete code for this:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
    
```

With the action server running in a separate terminal, if we run Fibonacci action client, we will be able to see logged messages for the goal being accepted and the final result.

```
python3 fibonacci_action_client.py
``` 

**3.2.2.Getting feedback**

Once, action client is able to send the goals, here is the complete code for getting some feedbacks about the goals we send from the action server:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
    
```

### 4.Composing multiple nodes in a single process

**4.1. To discover avilable components**

_In order to check the available components in the workspace, we run the following commands._

```
ros2 component types
```

The terminal returns all the available components:

![image](https://user-images.githubusercontent.com/113494159/196020894-c021cc1d-42ba-49bc-8841-a2d11a100985.png)

**4.2. Run-time composition using ROS services with a publisher and subscriber**

Firstly, we start the component container in one terminal :

```
ros2 run rclcpp_components component_container
```

In order to verify that the container is running via `ros2` command line tools, we run following command in the second terminal which will show a name of the component as an output.

```
ros2 component list
```

After this step, in the second terminal, we load the talker component:

```
ros2 component load /ComponentManager composition composition::Talker
```

This command will return the unique ID of the loaded component as well as the name of the node:

![image](https://user-images.githubusercontent.com/113494159/196021077-03a72c40-9ed6-421a-ab57-db6edeb20712.png)

![image](https://user-images.githubusercontent.com/113494159/196021068-fa4ffe58-7d70-42e3-9d06-d3ecff8eddfd.png)

After this, we run following code in the second terminal in order to load the listener component:

```
ros2 component load /ComponentManager composition composition::Listener
```

The output is shown in the images above.


Finally we can run the `ros2` command line utility to inspect the state of the container:

```
ros2 component list
```

We can see the result as follows:

```
/ComponentManager
   1  /talker
   2  /listener
```


**4.3. Run-time composition using ROS services with a server and client**

It is very similar steps to what we did using talker and listener. 

In the first terminal, we run:

```
ros2 run rclcpp_components component_container
```

and after that, in the second terminal, we run following commands to see **server** and **client** source code:

```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

We can see the output of the above commands as in the following images:

![image](https://user-images.githubusercontent.com/113494159/196021291-7cb081bf-ab97-4ebb-ac7d-1d02c01846bf.png)

![image](https://user-images.githubusercontent.com/113494159/196021325-1b981551-06f5-444e-a93e-9d5944c0a569.png)

**4.4. Compile-time composition using ROS services**

By using this demonstration, it shows that the same shared libraries can be reused to compile a single executable running multiple components. 

The executable contains all four components from above : `talker`, `listener`, `server`, and `client`.

In one terminal, 

```
ros2 run composition manual_composition
```

![image](https://user-images.githubusercontent.com/113494159/196021396-73d71513-9154-481f-a9a2-5fe903ea492b.png)


**4.5. Run-time composition using dlopen**

This demonstration shows an alternative to run-time composition by creating a generic container process an explicity passing the libraries to load without using ROS interfaces. The process will open each library and create one instance of each "rclcpp::Node" class in the library source code.


```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

![image](https://user-images.githubusercontent.com/113494159/196021502-a99852ad-36b2-47ea-904d-137d69055275.png)


**4.6. Composition using launch actions**

While the command line tools are helpful for troubleshooting and diagnosing component setups, starting a group of components at once is frequently more practical. We can make use of `ros2` launch's functionality to automate this process.

```
ros2 launch composition composition_demo.launch.py
```

![image](https://user-images.githubusercontent.com/113494159/196021525-b673e259-283a-407f-9cdf-ebf00c3796a3.png)


### 5.Creating a launch file

In order to create a launch file, we use the `rqt_graph` and `turtlesim` packages which we have already installed previously.

**5.1. Setup**

We need to create a new directory to store the launch files:

```
mkdir launch
```

**5.2. Writing the launch file**

In the newly created directory, we created a new file named `turtlesim_mimic_launch.py` and paste the following codes into the file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

**5.3. ros2 launch**

In order to run the launch file created, we enter into the earlier created directory and run the following commands:

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```

The output is shown in the image below:

![image](https://user-images.githubusercontent.com/113494159/196021738-0643f343-1071-4658-921c-95768bf41bd9.png)

Two turtlesim windows are opened with the above messages:

![image](https://user-images.githubusercontent.com/113494159/196021763-2c35af4d-817b-48fc-81fc-c39d2e618ec1.png)

_In order to see the sytem in action, we open a new terminal and run the `ros2 topic pub` command on `/turtlesim1/turtle1/cmd_vel` topic to get the first turtle moving._

![image](https://user-images.githubusercontent.com/113494159/196021849-9b6d5512-284c-4d28-a696-afa934e750f8.png)

**Both the turtle move in the same path.**


**5.4. Introspect the system with rqt_graph**

Open the new terminal without closing the system and we run `rqt_graph`.

```
rqt_graph
```

![image](https://user-images.githubusercontent.com/113494159/196022052-e8d67387-577a-41e9-916b-c4ac211548a6.png)


### 6.Integrating launch files into ROS2 packages

**6.1. Creating a package**

Firstly, we create a workspace for the package:

```
mkdir -p launch_ws/src
cd launch_ws/src
```

and create a python package:

```
ros2 pkg create py_launch_example --build-type ament_python
```


**6.2. Creating the structure to hold launch files **

After creating the packages, it should be looking as follows for python package:

![image](https://user-images.githubusercontent.com/113494159/196022501-78722f24-c268-4341-a589-706e7336f8eb.png)

And in order to colcon to launch files, we need to inform Python's setup tools of our launch files using the `data_files` parameter of `setup`.

Inside, `setup.py` file, we input the following codes:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

**6.3. Writing the launch file**

Inside the `launch` directory, we create a new launch file named `my_script_launch.py`.
Here, the launch file should define the `generate_launch_description()` function which returns a `launch.LaunchDescription() to be used by the `ros2 launch` verb.

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```

**6.4. Building and running the launch file**

We go to top-level of the workspace and build the file using:

```
colcon build
```

Once the build is successful, we should be able to run the launch file as follows:

```
ros2 launch py_launch_example my_script_launch.py
```

### 7.Using Substitutions

**7.1. Creating and Setting up the package**

We create a new package of build_type `ament_python` named `launch_tutorial` :

```
ros2 pkg create launch_tutorial --build-type ament_python
```

and inside of that package, we create a directory called `launch`.

```
mkdir launch_tutorial/launch
```

After that, we edit the `setup.py` file and add in changes so that launch file will be installed successfully. 

```python
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

**7.2. Parent Launch File**

After the above steps, we created a launch file named : `example_main.launch.py` in the launch folder of the `launch_tutorial` directory with the following codes in it:

```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

**7.3. Substitutions example launch file**

A new file is created in the same folder: `example_substitutions.launch.py`

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

**7.4. Building the package**

We run the build command in the root of the workspace. 

```
colcon build
```

**7.5. Launching Example**

Now we are able to run the `example_main.launch.py` file using the `ros2 launch` command. 

```
ros2 launch launch_tutorial example_main.launch.py
```

![image](https://user-images.githubusercontent.com/113494159/196023691-55605922-f633-4f1a-b057-47543e52db44.png)

A turtlesim node is started with a blue background. After that second turtle is spawned and the background color is changed to purple and pink respectively.

### 8.Using Event Handlers

**8.1. Event handler example launch file**

We created a new file named: `example_event_handlers.launch.py` in the same directory.. i.e. inside `launch` folder of `launch_tutorial` package.

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

**8.2. Building and Running the Command**

After adding the file, we go back to the root of the workspace and run the build command there.
```
colcon build
```

After building, it is important to source the package and run the following codes for the output:

ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200


![image](https://user-images.githubusercontent.com/113494159/196024114-0e8025ee-186e-4b6c-bc8a-5f518d23b78d.png)

It start a turtlesim node with a blue background and spawn the second turtle. After that, it change the background color to purple and then to pink.
If the turtlesim window is closed, it shutdown the launch file. 
