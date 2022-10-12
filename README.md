# Week6,  Sudip Dhungana 12194823

#Creating an Action

1.1 Prerequisites
1.2 Tasks

1.21 Defining an Action
**Creating an action Directory in action_tutorials_interfaces:**

```
cd action_tutorials_interfaces
mkdir action
```

##After that, a file is created named: Fibonacci.action with the following contents within the action directory.
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

![image](https://user-images.githubusercontent.com/113494159/194979723-6f51f5e2-c5a3-4335-8a10-fbaa38d93377.png)

1.22 Building an Action
*Before using the new Fibonacci action type in our code, it is necessary to add following lines to our CMakeLists.txt before ament_package() line in the action_tutorials_interfaces:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

##Additionally, we have to add the required dependencies to our package.xml as follows:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

*After all these, now we are able to build the package containing the Fibonacci action definition*
```
cd ~/ros2_ws
colcon build
```
![image](https://user-images.githubusercontent.com/113494159/194981121-5d3cb189-76c7-4ad0-bfda-314854be7781.png)


***Writing an action server and client in Python***

1. Writing an action server

![image](https://user-images.githubusercontent.com/113494159/194982477-4be78dbb-c0d3-4cf7-a351-0400ad095d15.png)
![image](https://user-images.githubusercontent.com/113494159/194982544-ccbc00dc-b762-4677-9743-fd7f559f68b9.png)
![image](https://user-images.githubusercontent.com/113494159/194984025-5acece74-7427-49cd-89a1-9f1ca1fbd58d.png)
![image](https://user-images.githubusercontent.com/113494159/194984461-5d52b8b7-18c9-46bf-9c37-814a58786adc.png)

2. Writing an action client

![image](https://user-images.githubusercontent.com/113494159/195436223-e91ed4cc-77af-4a66-a41a-4cf524324047.png)




