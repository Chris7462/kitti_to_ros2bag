# KITTI to ROS2 Bag
As I couldn't find any package on GitHub for directly converting raw KITTI data into ROS2 bag, I decided to create this repository.

The current version of this repository only supports the conversion of KITTI raw data into ROS2 bag. Contributions are welcome.

## How to use it?
1. Download the [raw data](https://www.cvlibs.net/datasets/kitti/raw_data.php) from KITTI website and unzip the downloaded file.
2. clone this repo under your **{ros2_workspace}/src** folder.
    ```bash
    git clone https://github.com/Chris7462/kitti_to_ros2bag.git
    ```
3. Modify the **kitti_to_ros2bag.yaml** file under *params* folder.
    * *kitti_path*: indicates the data path
    * *dirs*: designates the directories to be written into the bag
    * *output_bag_name*: specifies the output bag name

    For example:

    ```yaml
    kitti_path: "/data/kitti/raw/2011_09_29_drive_0071_sync/"
    dirs: ["image_00", "image_01", "image_02", "image_03", "oxts", "velodyne_points"]
    output_bag_name: "2011_09_29_drive_0071_sync_bag"
    ```
4. Build the package
    ```bash
    colcon build --symlink-install --packages-select kitti_to_ros2bag
    ```
5. Launch the package
    ```bash
    source ./install/setup.bash
    ros2 launch kitti_to_ros2bag kitti_to_ros2bag_launch.py
    ```
    That's it. You have the file *kitti_2011_09_29_drive_0071_sync_bag* that contains your data.

# References
1. https://github.com/umtclskn/ros2_kitti_publishers
2. https://github.com/tomas789/kitti2bag
