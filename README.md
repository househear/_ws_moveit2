# _ws_moveit2
webots +ros2 + moveit2


# Install webots from source following (linux):

https://github.com/cyberbotics/webots/wiki/Linux-installation/

install directory: /home/housebear/webots


# Install webots_ros2 from source:
https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install

webots_ros2_control is not built due to unknown error


housebear@yoga-thinkpad:~/_ws_moveit2$ ros2 node list
WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
/armed_robots_abb
/webots_driver
/webots_driver
housebear@yoga-thinkpad:~/_ws_moveit2$ ros2 action list
/abb/follow_joint_trajectory
/ur/follow_joint_trajectory
housebear@yoga-thinkpad:~/_ws_moveit2$ ros2 service list
/add_two_ints
/armed_robots_abb/describe_parameters
/armed_robots_abb/get_parameter_types
/armed_robots_abb/get_parameters
/armed_robots_abb/list_parameters
/armed_robots_abb/set_parameters
/armed_robots_abb/set_parameters_atomically
/robot_state_publisher/set_parameters
/step
/webots_driver/describe_parameters
/webots_driver/get_parameter_types
/webots_driver/get_parameters
/webots_driver/list_parameters
/webots_driver/set_parameters
/webots_driver/set_parameters_atomically

housebear@yoga-thinkpad:~/_ws_moveit2$ ros2 topic list
/clock
/joint_states
/parameter_events
/rosout



https://cyberbotics.com/doc/reference/robot
https://cyberbotics.com/doc/guide/starting-webots#on-linux-and-macos

    • selfCollision: setting this field to TRUE will enable the detection of collisions within the robot and apply the corresponding contact forces, so that the robot limbs cannot cross each other (provided that they have a Physics node). This is useful for complex articulated robots for which the controller doesn't prevent inner collisions. Enabling self collision is, however, likely to decrease the simulation speed, as more collisions will be generated during the simulation. Note that only collisions between non-consecutive solids will be detected. For consecutive solids, e.g., two solids attached to each other with a joint, no collision detection is performed, even if the self collision is enabled. The reason is that this type of collision detection is usually not wanted by the user, because a very accurate design of the bounding objects of the solids would be required. To prevent two consecutive solid nodes from penetrating each other, the minStop and maxStop fields of the corresponding joint node should be adjusted accordingly. Here is an example of a robot leg with self collision enabled:
Thigh (solid)
  |
Knee (joint)
  |
Leg (solid)
  |
Ankle (joint)
  |
Foot (solid)
In this example, no collision is detected between the "Thigh" and the "Leg" solids because they are consecutive, e.g., directly joined by the "Knee". In the same way no collision detection takes place between the "Leg" and the "Foot" solids because they are also consecutive, e.g., directly joined by the "Ankle". However, collisions may be detected between the "Thigh" and the "Foot" solids, because they are non-consecutive, e.g., they are attached to each other through an intermediate solid ("Leg"). In such an example, it is probably a good idea to set minStop and maxStop values for the "Knee" and "Ankle" joints.
    • showWindow: defines whether the robot window should be shown at the startup of the controller.
    • window: defines the path of the robot window controller plugin used to display the robot window. If the window field is empty, the default generic robot window is loaded. The search algorithm works as following: Let $(VALUE) be the value of the window field, let $(EXT) be the shared library file extension of the OS (".so", ".dll" or ".dylib"), let $(PREFIX) be the shared library file prefix of the OS ("" on windows and "lib" on other OS), let $(PROJECT) be the current project path, let $(WEBOTS) be the webots installation path, and let $(...) be a recursive search, then the first existing file will be used as absolute path:
$(PROJECT)/plugins/robot_windows/$(VALUE)/$(PREFIX)$(VALUE)$(EXT)
$(WEBOTS)/resources/$(...)/plugins/robot_windows/$(VALUE)/$(PREFIX)$(VALUE)$(EXT)
    • remoteControl: defines the path of the remote-control controller plugin used to remote control the real robot. The search algorithm is identical to the one used for the window field, except that the subdirectory of plugins is remote_controls rather than robot_windows.

g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 1/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 1/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 1/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 1/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 1/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0

Floor contact WB, 4 wheels
8 Robots space contact
8 unknown space conatct
5 gemos unknown contacts

BoundingObject Finger1 is a group
BoundingObject Finger2 is a group
Box+Cylinder is a group




g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/1
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 1 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 1; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 0/0; FR = g1/g2 0/0; KB = g1/g2 0/0; Rob = g1/g2 0/0
g1 is a space 0; g2 is a space 0 ; WB = g1/g2 1/0; FR = g1/g2 0/1; KB = g1/g2 0/0; Rob = g1/g2 0/0

      dWebotsConsolePrintf("g1 is a space %d; g2 is a space %d ; WB = g1/g2 %d/%d; FR = g1/g2 %d/%d; KB = g1/g2 %d/%d; Rob = g1/g2 %d/%d; WL1 = g1/g2 %d/%d; WL2  = g1/g2 %d/%d; WL3  = g1/g2 %d/%d; WL4  = g1/g2 %d/%d\n", 
      dGeomIsSpace(g1), dGeomIsSpace(g2),
      dAreGeomsSame(g1, WB_geom),(dAreGeomsSame(g2, WB_geom)),
      dAreGeomsSame(g1, FR_geom),(dAreGeomsSame(g2, FR_geom)),
      dAreGeomsSame(g1, KB_geom),(dAreGeomsSame(g2,KB_geom)),
      dAreGeomsSame(g1, Rob_geom),(dAreGeomsSame(g2,Rob_geom)),
      dAreGeomsSame(g1, WHEEL1_geom),(dAreGeomsSame(g2,WHEEL1_geom)),
      dAreGeomsSame(g1, WHEEL2_geom),(dAreGeomsSame(g2,WHEEL2_geom)),
      dAreGeomsSame(g1, WHEEL3_geom),(dAreGeomsSame(g2,WHEEL3_geom)),
      dAreGeomsSame(g1, WHEEL4_geom),(dAreGeomsSame(g2,WHEEL4_geom)));










1. File→Import 3D model. 模型将会以solid节点显示在数结构的最末端
2. 在该solid下面的boundingObject Mesh中添加Mesh用于作碰撞检测边界（可以是与第一步导入的同一个mesh。也可以是分辨率更低的mesh）；
3. 添加physics节点


建立一个静态基准(STATIC_BASE), 将需要测试的物体加入到该基准下的children中



/*
 * File:
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdlib.h>
static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE

/* The Geoms used in our plugin. */
static dGeomID robot_geom;

/* The Bodies used in our plugin. */
static dBodyID robot_body;
/*
 * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 * To associate this plugin with the world follow these steps:
 *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 *  2. Then hit the [Select] button at the bottom of the Scene Tree
 *  3. In the list choose the name of this plugin (same as this file without the extention)
 *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 *  5. Then reload the world: the plugin should now load and execute with the current simulation
 */

void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);
  robot_geom = dWebotsGetGeomFromDEF("TARGET");
  /*
   * Get ODE object from the .wbt model, e.g.
   *   dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");
   *   dBodyID body2 = dWebotsGetBodyFromDEF("MY_SOLID");
   *   dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SOLID");
   * If an object is not found in the .wbt world, the function returns NULL.
   * Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
   *
   * This function is also often used to add joints to the simulation, e.g.
   *   dWorldID world = dBodyGetWorld(body1);
   *   pthread_mutex_lock(&mutex);
   *   dJointID joint = dJointCreateBall(world, 0);
   *   dJointAttach(joint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
  if (robot_geom)
    robot_body = dGeomGetBody(dSpaceGetGeom((dSpaceID)robot_geom, 0));
  dWebotsConsolePrintf("Initialization: ROBOT = %p %p\n", robot_geom, robot_body);

}

void webots_physics_step() {

  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {

  if (dAreGeomsSame(g1, robot_geom) || dAreGeomsSame(g2, robot_geom))
  {
  
        dWebotsConsolePrintf("@%g s: suzande colide", dWebotsGetTime() / 1000);
  
  }
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise.
   * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   pthread_mutex_lock(&mutex);
   *   dJointCreateContact(world, contact_joint_group, &contact[i])
   *   dJointAttach(contact_joint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
  return 0;
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
  pthread_mutex_destroy(&mutex);
}




/*
* File:
* Date:
* Description:
* Author:
* Modifications:
*/

#include <ode/ode.h>
#include <plugins/physics.h>

static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE
static dGeomID SUZANNE_geom;
static dGeomID A_geom;
static dGeomID B_geom;
static dGeomID C_geom;
static dGeomID D_geom;
static dGeomID E_geom;
static dGeomID F_geom;
/*
* Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
* To associate this plugin with the world follow these steps:
* 1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
* 2. Then hit the [Select] button at the bottom of the Scene Tree
* 3. In the list choose the name of this plugin (same as this file without the extention)
* 4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
* 5. Then reload the world: the plugin should now load and execute with the current simulation
*///

void webots_physics_init() {
pthread_mutex_init(&mutex, NULL);
SUZANNE_geom = dWebotsGetGeomFromDEF("SUZANNE");
A_geom = dWebotsGetGeomFromDEF("A_LINK");
B_geom = dWebotsGetGeomFromDEF("B_LINK");
C_geom = dWebotsGetGeomFromDEF("C_LINK");
D_geom = dWebotsGetGeomFromDEF("D_LINK");
E_geom = dWebotsGetGeomFromDEF("E_LINK");
F_geom = dWebotsGetGeomFromDEF("F_LINK");

dWebotsConsolePrintf("IS A SPACE: suzanne/B/C/D/E/F: %d/%d/%d/%d/%d/%d \n", dGeomIsSpace(SUZANNE_geom), dGeomIsSpace(B_geom), dGeomIsSpace(C_geom), dGeomIsSpace(D_geom), dGeomIsSpace(E_geom), dGeomIsSpace(F_geom));

/*
* Get ODE object from the .wbt model, e.g.
* dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");LLL
* dBodyID body2 = dWebotsGetBodyFromDEF("MY_SOLID");
* dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SOLID");
* If an object is not found in the .wbt world, the function returns NULL.
* Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
*
* This function is also often used to add joints to the simulation, e.g.
* dWorldID world = dBodyGetWorld(body1);
* pthread_mutex_lock(&mutex);
* dJointID joint = dJointCreateBall(world, 0);
* dJointAttach(joint, body1, body2);
* pthread_mutex_unlock(&mutex);;
* ...
*/
}

void webots_physics_step() {
/*
* Do here what needs to be done at every time step, e.g. add forces to bodies
* dBodyAddForce(body1, f[0], f[1], f[2]);
* ...
*/
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {

/* First we check if the collision is involving the FINGER1_geom. */
if ((dAreGeomsSame(g1, SUZANNE_geom) || dAreGeomsSame(g2, SUZANNE_geom)))
{

if (((dAreGeomsSame(g1, F_geom) || dAreGeomsSame(g2, F_geom))) ||
((dAreGeomsSame(g1, E_geom) || dAreGeomsSame(g2, E_geom))) ||
((dAreGeomsSame(g1, D_geom) || dAreGeomsSame(g2, D_geom))) ||
((dAreGeomsSame(g1, C_geom) || dAreGeomsSame(g2, C_geom))))
{

dWebotsConsolePrintf("@ %g s: F/E/D/C: %d/%d/%d/%d \n", dWebotsGetTime() / 1000,
((dAreGeomsSame(g1, F_geom) || dAreGeomsSame(g2, F_geom))),
((dAreGeomsSame(g1, E_geom) || dAreGeomsSame(g2, E_geom))),
((dAreGeomsSame(g1, D_geom) || dAreGeomsSame(g2, D_geom))),
((dAreGeomsSame(g1, C_geom) || dAreGeomsSame(g2, C_geom))));

}

}

/*
* This function needs to be implemented if you want to overide Webots collision detection.
* It must return 1 if the collision was handled and 0 otherwise.
* Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
* n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
* dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
* dWorldID world = dBodyGetWorld(body1);
* ...
* pthread_mutex_lock(&mutex);;
* dJointCreateContact(world, contact_joint_group, &contact[i])
* dJointAttach(contact_joint, body1, body2);
* pthread_mutex_unlock(&mutex);\ * .....
*/
return 0;
}

void webots_physics_cleanup() {
/*
* Here you need to free any memory you allocated in above, close files, etc.
* You do not need to free any ODE object, they will be freed by Webots.
*/
pthread_mutex_destroy(&mutex);
}


May 15 2021
How we want it to work?
1. Move the robot roughly to where it can see a 2D label image;
2. Type start in console, the camera takes one image;
3. Image is shown in Display;
4. Image is used to calculate the 3D position of the label;
5. 3D Points is shown in 3D scene;
6. Determine the first approaching point;
7. Robot moves to the first approaching point;
8.If approachable, send the 3D point to physical robot and trigger the 3D scanner to scan;
9. Take a 2D label image and go to step 2 (if in single step mode, wait for start, if in continuous mode, go forward)








main controller
1. Design and respond to console commands

void initial environment();
system_mode = single/continuous
print_status = true
initial camera capturing

print help message

while ()
{ switch command:

idle: waiting for command
capture3Dpose: use the current image for calculating 3D pose of the labels present in the image. If system_mode = single, jump to idle, if = continuous, jump to planpath
update_database: use the latest poses to refresh the database;
plan_path: determine the next point based on the 3Dpoints datebase (approaching_point = ..)
approach_point: robot move to approaching_point


inquiry_status: 

}

Capturing video image in robsim, the image processing is conducted in ImgController robot

supervisor_tdraw_tail
ImgController (robot controller): 
1. generate tracking points after triggering the image capturing
2. update three point groups

three groups for tracking points:
1. current tracking points
2. un-reached point
3. reached point


PhysicsController (Physics plug-in)
process controller
1.trigger image capturing to ImgController
2.
use switch status to control the robot motion
idle: 
approaching point:





5/20:
Three groups are added for grouping points:
Current_P:
Waiting_P:
Reached_P:
视觉引导模块（RobPlan）

仿真模块（RobSIm）

导入被测物体CAD
碰撞报错
定义三种点集

集成视觉引导模块（RobPlan）与仿真模块（RobSIm）
1. 标签摄像机的采集，实时显示及分析

5/24:
A full scanning process will end when all labels have been detected and reached;


The supervisor controller will loop until the given number of labels are hit;

int nb_labels_in_total; //number of labels in total, needs to be manually set
int  nb_reached_labels; // number of labels that has been reached by robot;
bool run_continuous = true; // 


While (nb_reached_labels < nb_labels_in_total) 
{

keyboard_get_key actions;
switch(state):

state: 



int nb_reached_labels  = nb_p_in_Reached; //read number of points in Reached_P_Group
};


InverseKinematics (controller)

watching the group- current tracking point, if available, approaching it. 


5/25
Found inverse kinematic tool Moveit in ROS

https://ros-planning.github.io/moveit_tutorials/

concept of catkin
http://wiki.ros.org/catkin


robot models

https://moveit.ros.org/robots/





wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
GAZEBO_MAJOR_VERSION=9 ROS_DISTRO=noetic . /tmp/dependencies.sh
echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install


source devel/setup.bash

HOW TO
1. find a file 
find . -name MotionPlanRequest.h

2. open current explore
xdg-open .

roslaunch moveit_tutorials move_group_interface_tutorial.launch



tcp_ip_communication:
----topics subscription
/image_processor/tag_detected: when a tag is detected, draw it in webots UI

  <software name>rock rhino</software name>
  <version>0.0.1</version>
  <description>
  rock rhino is an armed robot development platform that is guided by machine vision and conduct wide variety of autonomous tasks.
 </description>
  <maintainer email="yding@uestc.edu.cn">Yaoyu Ding</maintainer>
  <license>Apache License 2.0</license>

MANUAL:
Command Definition:
Diag Mode: diagnose individual packages when unexpected issues present.
Prod Mode: production mode that run the process normally, it is categorized into two sub mode :continuous/step. The continuous mode will run the process continuously with no stop. The step mode will stop at each end of process status and wait for a command to proceed. There are mainly 6 process statuses 

image_processor/detecting_tag 
database/updating_tags
moveit2/planning_trajectory
webots_driver/simulating_trajectory
tcpip_service/updating_webots_UI
The package rock_rhino_process_controller configures the process properties and controls the switch between process status and send the tasks to other rock_rhino packages.
Command input: type keyboard “c” to accept any command:
system_mode check
system_mode switch

AUTO MODE:

image_processor/detecting_tag:
rock_rhino_process_controller sends the action request (detect tag in the live image) to rock_rhino_image_processor, rock_rhino_image_processor capturing the live image, calculating tag info, and reserving the result for updating the database. Then send action done back to rock_rhino_process_controller
/rock_rhino_process_controller/
database/updating_tags
rock_rhino_process_controller sends the action request (update the database) to rock_rhino_database,  rock_rhino_database uses the reserved tag in rock_rhino_image_processor to update the tag database and refresh the three groups of tags:
{approaching point, waiting points, approached points}
approaching point is the one on the top of the waiting points queue, which is being approached or is the one to be approached when approaching request is captured.
Waiting points is a queue that contains all detected but not approached yet tags. New tag will be appended to this queue. A FIFO rule is adopted to determine the approaching sequence
approached points is a queue that store all approached points.

moveit2/planning_trajectory
webots_driver/simulating_trajectory
tcpip_service/updating_webots_UI


main start: 
1. the image processor starts capturing and detecting (assume the default robot position could detect tag, otherwise need to manually move robot to capture the initial tag, or add tag in the scene to initialize the process)
broadcast detected tag via:
/rock_rhino_image_processor/detected_tag
message: rock_rhino_image_processor: a new tag is detected and broadcasted
2.  the database is listening the topic /rock_rhino_image_processor/detected_tag
once a new tag is broadcast, it updates the database by inserting the new tag into database. 
topic: /rock_rhino_image_processor/detected_tag
publisher:  rock_rhino_image_processor
subscriber:   rock_rhino_database
message: rock_rhino_database: obtained new tag from image processor, inserted into database.
The database updates watch dog will do two things:
a. send the updated tag to webots UI via  rock_rhino_tcpip_service. Two steps are implemented: first, send update_tag_plot to rock_rhino_tcpip:
topic: /rock_rhino_database/update_tag_plot
publisher: rock_rhino_database
subscriber:  rock_rhino_tcpip_service
message: rock_rhino_database: send request to rock_rhino_tcpip_service to update tag plot in webots UI
Then, rock_rhino_tcpip will send the updated tag info to webots _superman via data package with structure:
{
action type: add/delete/change
point_info: translation&posture
}
message: rock_rhino_tcpip_service: send request to webots _superman for tag update.

B. if process is in auto, send the first element in FIFO queue to rock_rhino_moveit2
 /rock_rhino_moveit2/request_move_action
topic: /rock_rhino_database/request_move_action
publisher: rock_rhino_database
subscriber:   rock_rhino_moveit2
message: rock_rhino_database: send request to rock_rhino_moveit2 to plan the trajectory for approaching the targeted point.
3. webot _superman is listening via tcpip service, it will parse the package and respond to defined action types.
message: webots _superman: a new tag is inserted, the current point is updated.

4. the rock_rhino_moveit2 is listening the move_action from database, and is planing the trajectory for approaching the targeted point.
topic: /rock_rhino_database/request_move_action
publisher: rock_rhino_database
subscriber:   rock_rhino_moveit2
message: rock_rhino_moveit2: is plan the trajectory for approaching the targeted point.
If it succeeds, the trajectory is sent to webot_drive for move simulation, if it failed, error is sent to process controller and the auto is suspended

5. The rock_rhino_webots_driver simulates the planned trajectory from moveit2 and\

topic: /the rock_rhino_moveit2/following_joint_controller
publisher: rock_rhino_moveit2
subscriber:  the rock_rhino_webot_driver
message: rock_rhino_webot_driver:  is simulating the planned trajectory from moveit2
once the targeted point is approached, a. webots_driver send the info to database to change the type of current point (from current to approached); 
topic: /the rock_rhino_process_controller/change_process_status
publisher:  rock_rhino_webot_driver
subscriber:   the rock_rhino_process_controller
message: rock_rhino_webot_driver:  simulation of planned trajectory finished, set the process status to: ready_for_motion

b. webots_driver send the process_status to be 
ready_for_motion. If the database has waiting_points, then it jumps to step 4 and loop


MANUAL MODE:
Create a component library to handle tcpip connection and requests (it is actually an client). The service is in webots _superman.


PACKAGES:
rules for packages:
enable individual function test

rock_rhino_tcpip_service:
Create a component library to handle tcpip connection and requests (it is actually an client). The service is in webots _superman.

srv_connect_tcpip = create_service<example_interfaces::srv::AddTwoInts>("tcpip/connect_tcpip", handle_connect_tcpip);
srv_send_request = create_service<example_interfaces::srv::AddTwoInts>("tcpip/send_request", handle_send_request);
srv_close_tcpip = create_service<example_interfaces::srv::AddTwoInts>("tcpip/close_tcpip", handle_close_tcpip);

Individual Test:
1. open webots in its local folder by cmd:  ./webots
2. open khepera_tcpip.wbt
3. run the tcpip node by cmd: ros2 run rock_rhino_tcpip_service tcpip_service
4. start clients by cmd : ros2 run rock_rhino_tcpip_service add_two_ints_client 1 1.
type “a” to connect tcpip, typ “b” to test different operation on robot, type “c” to close tcpip
Integrated Test:
1. run the tcpip node by cmd: ros2 run rock_rhino_tcpip_service tcpip_service
2. start clients by cmd : ros2 run rock_rhino_tcpip_service add_two_ints_client 1 1. and send a to connect (after the webots file is opened, otherwise it will fail to connect).


rock_rhino_image_processor:
This package handles image processing and broadcast processed image/info. The image source is from topic 'camera/image_raw', and the detected tag information is broadcast by topic 'image_processor/detected_tag'.

Individual Test:
1. launch ros2 launch rock_rhino_image_processor ar_detection_launch.py
2. open rqt to check the detected tag

rock_rhino_database:
This package serves as database and maintain the tag info, controls the next approaching 3D pose. Before the usage of the database, the database server needs to be started by: mongodb_server_node. Then a task specified server db_server_node start to
a.  handle the incoming tags and
b. selfmain the db and 
c. deal the request from the database
this command starts a node /mongodb
Individual Test:
1. run  ros2 launch db_test db_server.launch..py

main code:

Updates: db_test replaced rock_rhino_database
ros2 run db_test test_demo
when a new tag is added into db, send the request to tcpip server to update ui points
pos
rock_rhino_webot_driver:
This package serves as interface of webots and communicate with external ROS2.
  Run demo to check out two robot are controlled by its corresponding actions:
ros2 launch rock_rhino_webot_driver armed_robots.launch.py
notice: there is no need to turn on nodes( static_tf and robot_state_publisher) in webots driver.


rock_rhino_moveit2:
This package utilizes moveit2 package to plan the trajectory of armed robot for approaching a potion with a given posture. It is expected to avoid collision in the planning scene.
Individual Test:
One can test and debug moveit function by running --- ros2 launch rock_rhino_moveit2 start_move_group.launch.py ---. (the webots window will pop if run --- ros2 launch rock_rhino_moveit2 run_move_group.launch.py --- )The rviz pops up, click “add” and add motion planning. One can practice planning for the armed robot in the rviz 
1. Manually drag the robot and hit “plan & excute”
2. use program to command path planning by running: --- ros2 launch rock_rhino_moveit2 run_ompl_demo.launch.py ---


rock_rhino_process_controller:
This package act as a supervisor for all the actions/communication between task, it also handles the configuration of the process mode and provide interface for terminal command.
Run demo to send order though client to other packages:
ros2 run rock_rhino_process_controller rock_rhino_process_controller
















Examine the code:
Before the node is spinned, start a new thread to capture keyboard input in a while loop 
_thread.start_new_thread(process_status_controller_test, (process_controller,1))

a class Process_Controller is defined for handling the following tasks:
parse
maintain process_status
predefine job which contains task (image )
command parameter_1 parameter_2…
set system_mode diag
get system_mode diag
runjob conti
runjob step
runtask detect_tag

rock_rhino_main:
This package launches all related package in order and run the entire project.
1. open the environment by ros2 launch rock_rhino_main armed_robots.launch.py
webots: launch webots

2. run the tcpip node by cmd: ros2 run rock_rhino_tcpip_service tcpip_service. Start clients by cmd : ros2 run rock_rhino_tcpip_service add_two_ints_client 1 1. and send a to connect (after the webots file is opened, otherwise it will fail to connect).
TODO: auto connection in tcpip package initial phase
initialize the tcpip automatically when launch the main. It is done by set a sleep(30) in the rock_rhino_tcpip_service.add_two_ints_client_async.cpp, which send the initial request to connect. The waiting time is fixed for now, it needs to optimized by monitoring the webots launch process.9/2
9/6: construct the DetectedTag.msg, and filter the tag by compared the detected tag with the last three tags (assumed the maximum amount of tag present in a image is <=3), if it has been recorded, then do not broadcast. This filter only avoids the multiple detection of the same tag in the current image and save the broadcast traffic. When the tag is revisited, it needs to be filtered in the database. DB keeps listening image_processor/detected_tag and updating the tags. If the tag  existed in DB, then do not insert.
image processor and db are self-maintain processes which are launched in main. To trigger the motion, comments needs to be sent in process controller. The process controller sends job “approach a tag” which is consist of the functional block:
a. inquiry db if tag exist; run  <ros2 run rock_rhino_process_controller rock_rhino_process_controller> type runtask inquiry
b. if a is yes, then send request to moveit for plan (obtain tag info from db, do plan, send ack to pctr). 
c. drive webots using planned trajectory.
d. drive real robot
if the job runs in continuous mode, it loops back to a and continue, if the job runs in step, it waits after step d is finished, until get command to approach the next tag.
Action request from process controller will do turn on/off updating (default is on), query db...

Job driven process: when a job starts, the process controller firstly check the availability of the tag in db, if yes, then send approval to moveit2 to plan the trajectory. Moviet2 requests the tag info from db and do planning.

TODO: dabase package is watching the newly detected tag, if there is new tag coming, send b type command to add points in webots ui
issues: launch process_controller in error, only can run the exe, not sure why.





RO2 driver available robots
https://www.youtube.com/watch?v=GVEpat-wlUA
RO2 driver for UR robot
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy


ABB introduces a new ROS driver for its robots
https://www.therobotreport.com/abb-introduced-new-ros-driver-robots/

F0306 自动化驾车装置
检测装备
科学问题： 另外之后 ，medical diference with LITH cutting, why you study this, speciality
need-- problem- to study- current solution 
end on measuremnt.
laser measure not laser cutting
–
obtract- not mature – meaning of meaure ,status , memeasurenet note sthet, value.
Equipment- not buyin ,,say you are not redy
no patent fee any longer
graduate 3-4, for
2.

intro for battery is too much, process , then proess weak,
 insert at end of programen
---
ar tag paper
https://www.researchgate.net/figure/Yaw-rotations-Z-axis-of-ARTag-marker-clockwise-and-counter-clockwise-for-10-20-30_fig9_317696465

ros.spin definition
https://blog.csdn.net/weixin_40215443/article/details/103793316