To create a new ROS package called `drone_trajectories` and set up the necessary folders (`launch`, `src`, and `srv`), you can follow these steps:

### **Step 1: Set Up Your ROS Workspace**
First, make sure you have a ROS workspace set up. If you don’t have one, you can create one as follows:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### **Step 2: Create the New Package**
Navigate to your workspace's `src` directory and create the new package `drone_trajectories`. We'll assume the package depends on `rospy` and `std_msgs` (you can add more dependencies if needed).

```bash
cd ~/catkin_ws/src
catkin_create_pkg drone_trajectories rospy std_msgs
```

This command will create a new package with a basic `CMakeLists.txt` and `package.xml` file.

### **Step 3: Create the Required Folders**
Now, navigate into the `drone_trajectories` package and create the necessary folders: `launch`, `src`, and `srv`.

```bash
cd ~/catkin_ws/src/drone_trajectories
mkdir launch src srv
```

### **Step 4: Verify the Structure**
You can verify that the structure is correct by listing the contents of the `drone_trajectories` directory:

```bash
ls -R
```

You should see something like this:

```bash
.:
CMakeLists.txt  launch  package.xml  src  srv

./launch:
# (This folder is currently empty)

./src:
# (This folder is currently empty)

./srv:
# (This folder is currently empty)
```

### **Step 5: Build the Package**
Finally, go back to the root of your workspace and build your package:

```bash
cd ~/catkin_ws
catkin_make
```

### **Step 6: Source Your Workspace**
After building, source your workspace to make sure ROS recognizes the new package:

```bash
source devel/setup.bash
```

Now, your `drone_trajectories` package is set up with the `launch`, `src`, and `srv` folders, ready for you to add launch files, source code, and service definitions as needed.
