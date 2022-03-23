# Turtlebot3 Navigations

Turtlebot3 package for autonomous navigation that uses the namespace `tb3`.

Tested on ROS Melodic Gazebo.

## Dealing with namespaces

When using namespaces and/or multiple robots, separate `yaml` files need to be created for each robot (that uses `move_base`) to reflect the new namespaced topics. These are (some of) the locations which need to be updated

- [global_costmap_params](./param/global_costmap_params.yaml)
- [local_costmap_params](./param/global_costmap_params.yaml)

To enable manual navigation via RVIZ, ensure the topics for the following are in the right namespace in your `rviz` config file:

- **Visualization Manager** :

  - `move_base_simple` in **Tools** -> `rviz/SetGoal`
  - `move_base/NavfnROS/plan` in **Display** -> `rviz/Path`

## Autonomous Mapping

The `explore_lite` launch file performs autonomous mapping of closed spaces. It uses the [explore_lite](http://wiki.ros.org/explore_lite) package, which can be installed easily using

```bash
sudo apt-get install ros-<DISTRO>-explore-lite
```

or from source(which is what I did) as follows

```bash
cd catkin_ws
git clone --single-branch --branch <ROS_distro>-devel https://github.com/hrnr/m-explore.git
```

This works, but is not perfect; could use more fine tuning to work in complex worlds.

## Localization

Use the `global_localization` and `nomotion_update`/`request_nomotion_update` services for localization. Refer to this [tutorial][1] for more info






[1]:http://wiki.ros.org/Robots/TIAGo/Tutorials/Navigation/Localization
