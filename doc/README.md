# HBBA Module Concepts
The HBBA module manages the global robot behavior by activating and deactivating some specific behavior modules depending on the current motivations. The module generates a launch file from many HBBA configuration YAML files. The generator is a Python script named `hbba_synth`. The generated file contains all perception, behavior and motivation nodes. The generator adds filtering and arbitration nodes to the generated launch file. The Python script also adds remap clauses to connect filter and arbitration inputs and outputs to the perception, behavior and motivation nodes. The generated launch file also contains defined strategies, initial desires and the arbitration node configuration.

# Project Structure
One way to structure a HBBA project is to create a package for the HBBA configuration and another one containing the bring-up launch files. The first package contains the HBBA YAML files and the associated launch files of perception, behavior and motivation nodes. The second one contains the launch files of all other nodes.

# HBBA Configuration Package Setup
The package must have a folder named `hbba_cfg` which contains all HBBA configuration files and a folder named `launch` which contains all associated launch files.

The `CMakeLists.txt` file must contain these elements:
- `find_package(catkin REQUIRED COMPONENTS hbba_synth)`
- `add_hbba_cfg(configuration_name hbba_cfg/entry_point_configuration_filename.yaml)`
- `install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/ FILES_MATCHING PATTERN "*.launch" PATTERN ".svn" EXCLUDE)`
- `install(DIRECTORY hbba_cfg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/hbba_cfg/ FILES_MATCHING PATTERN "*.yaml" PATTERN ".svn" EXCLUDE)`

If you want to choose where the HBBA base nodes are executed, you need to add `EXTERNAL NO_EVENT_GENERATORS` flags to the `add_hbba_cfg` command. Then you need to add a launch clause in the entry-point configuration file. The included launch file should look like this.

```xml
<launch>
  <arg name="hbba_machine" default="head"/>

  <group ns="hbba">
    <node machine="$(arg hbba_machine)" name="iw" pkg="iw" type="iw" output="screen"/>
    <node machine="$(arg hbba_machine)" name="iw_translator" pkg="iw_translator" type="iw_translator_node" output="screen"/>
    <node machine="$(arg hbba_machine)" name="iw_observer" pkg="iw_observer" type="iw_observer_node" output="screen"/>
    <node machine="$(arg hbba_machine)" name="events_generator" pkg="iw_tools" type="events_generator"/>
    <node machine="$(arg hbba_machine)" name="goto_events_observer" pkg="iw_event_generators" type="goto_events_observer">
      <rosparam>
        goal_eps_d: 0.20
        goal_eps_a: 0.10
      </rosparam>
    </node>

    <rosparam>
      solver_model:
        solver_log:  false
        solver_sa:   false
        time_limit:   1000
        max_p:        true
    </rosparam>
  </group>
</launch>
```

# API

- [Entry-point HBBA configuration file](entry-point-config-file.md)
- [Launch Files](launch-files.md)
- [Remap Clauses](remap-clauses.md)
- [Resources](resources.md)
- [Includes](includes.md)
  - [Perceptions](includes-perceptions.md)
  - [Behaviors](includes-behaviors.md)
  - [Motivations](includes-motivations.md)
  - [Strategies](includes-strategies.md)
- [Initial Desires](initial-desires.md)
- [`custom_bringup` and `custom_bringdn` Built-in Functions](js-functions.md)
