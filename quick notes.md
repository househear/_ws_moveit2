1. how to build into DEBUG mode
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select db_test



2. source

. install/setup.bash


3. launch db_test
ros2 launch db_test db_server.launch.py
ros2 run rock_rhino_process_controller rock_rhino_process_controller


Errors:
[component_container-2] 2022-04-13 15:34:00.365 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7414: open_and_lock_file failed -> Function open_port_internal

# targeted process
    1. scinario: default 3 tag images present in 3d space (task 1)
    2. image processor detected tags and send it to db (task 2)
    3. webots driver monitor the acitve tags in db, if exit, moveit2 to it
    4. robot apporached all tags, stop

# process controller
A B C

A: runtask/runjob
B: module (db, img, wb, mvit)
C: task type
    db: query/add/delete
# 4/13 to construct db and acess db from process controller
--- detectedTag data type
    int64 tag_id   
    geometry_msgs/Vector3 position
    geometry_msgs/Vector3 posture

--- query db
    void query_all() //print out all records in db;
    void add_record() //add an record into db;
    void delete_record() //delete an record into db;


