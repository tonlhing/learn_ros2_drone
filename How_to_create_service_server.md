To create a Service Server in ROS, you'll first need to define a service file and then write the server code that will handle requests based on that service. Below are the steps to create a service server in your `drone_trajectories` package.

### **Step 1: Define the Service File**

1. **Create the Service File:**
   - Navigate to the `srv` directory in your package:
     ```bash
     cd ~/catkin_ws/src/drone_trajectories/srv
     ```
   - Create a new service file, for example, `ComputeTrajectory.srv`:
     ```bash
     touch ComputeTrajectory.srv
     ```
   - Edit the `ComputeTrajectory.srv` file to define the input and output of the service:
     ```bash
     nano ComputeTrajectory.srv
     ```

   **Example Content for `ComputeTrajectory.srv`:**
   ```plaintext
   float64 x
   float64 y
   float64 z
   ---
   float64 distance
   ```

   In this example:
   - The service takes three `float64` values (`x`, `y`, `z`) as input.
   - It returns a single `float64` value (`distance`).

2. **Update `CMakeLists.txt` and `package.xml`:**
   - Open `CMakeLists.txt` and add the following lines to ensure the service is generated:
     ```cmake
     find_package(catkin REQUIRED COMPONENTS
       rospy
       std_msgs
       message_generation
     )

     add_service_files(
       FILES
       ComputeTrajectory.srv
     )

     generate_messages(
       DEPENDENCIES
       std_msgs
     )

     catkin_package(
       CATKIN_DEPENDS message_runtime
     )
     ```

   - In `package.xml`, make sure you have the following dependencies:
     ```xml
     <build_depend>message_generation</build_depend>
     <exec_depend>message_runtime</exec_depend>
     ```

3. **Rebuild Your Package:**
   - Navigate back to your workspace and build the package:
     ```bash
     cd ~/catkin_ws
     catkin_make
     ```

### **Step 2: Write the Service Server Code**

1. **Create the Service Server Script:**
   - Navigate to the `src` directory:
     ```bash
     cd ~/catkin_ws/src/drone_trajectories/src
     ```
   - Create a Python script for your service server, for example, `compute_trajectory_server.py`:
     ```bash
     touch compute_trajectory_server.py
     chmod +x compute_trajectory_server.py
     ```

2. **Write the Server Code:**
   - Edit the `compute_trajectory_server.py` file:
     ```bash
     nano compute_trajectory_server.py
     ```

   **Example Content for `compute_trajectory_server.py`:**
   ```python
   #!/usr/bin/env python

   import rospy
   from drone_trajectories.srv import ComputeTrajectory, ComputeTrajectoryResponse
   import math

   def handle_compute_trajectory(req):
       rospy.loginfo("Computing trajectory for points: x=%f, y=%f, z=%f", req.x, req.y, req.z)
       distance = math.sqrt(req.x ** 2 + req.y ** 2 + req.z ** 2)
       return ComputeTrajectoryResponse(distance)

   def compute_trajectory_server():
       rospy.init_node('compute_trajectory_server')
       s = rospy.Service('compute_trajectory', ComputeTrajectory, handle_compute_trajectory)
       rospy.loginfo("Ready to compute trajectory.")
       rospy.spin()

   if __name__ == "__main__":
       compute_trajectory_server()
   ```

   **Explanation:**
   - `handle_compute_trajectory`: This function handles incoming service requests. It calculates the Euclidean distance from the origin to the point `(x, y, z)` and returns this as the response.
   - `compute_trajectory_server`: This function initializes the ROS node and sets up the service server, ready to handle requests.

### **Step 3: Test the Service Server**

1. **Run the Service Server:**
   - Source your workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```
   - Run the service server node:
     ```bash
     rosrun drone_trajectories compute_trajectory_server.py
     ```

2. **Call the Service:**
   - In a new terminal, source your workspace again and call the service:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     rosservice call /compute_trajectory 3.0 4.0 5.0
     ```
   - This should return the computed distance, which would be the Euclidean distance for the point `(3.0, 4.0, 5.0)`.

### **Conclusion:**
You've now created a service server in ROS that computes the distance of a point from the origin. The server is running inside your `drone_trajectories` package, and it can handle requests defined by the `ComputeTrajectory.srv` service file.
