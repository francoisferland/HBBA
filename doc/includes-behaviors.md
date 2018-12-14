# Behaviors
These clauses define behavior modules. The input topics are filtered by default. The output topics are prioritized by arbitration nodes.

Notes:
- The output topics can be renamed by global remap clauses.
- The specified launch file must not contain any remap clauses.
- The topic name must not contain "/" if the topic is filtered.
- The arbitration nodes use the priority to select the proper messages. The messages from the highest priority behavior are chosen.

## Syntax
```yaml
  - procmodule:
    name: node_name
    machine: filters/arbitration_execution_machine_name
    launch:
      pkg: package_name
      path: launch_file_relative\_path
      args:
        arg_name_1: value_1
        arg_name_2: value_2
        ...
    priority: relative_value
    input:
      - node_and_global_topic_name
      - node_topic_name: {src: global_topic_name}
      - node_and_global_topic_name: {filtered: true/false}
      - node_topic_name: {src: global_topic_name, filtered: true/false}
      ...
    output:
      - node_and_global_topic_name
      ...
    services:
      - node_and_global_service_name
      ...
```

## Types
- `name`: string
- `machine`: string (optional)
- `launch`: dictionary
  - `pkg`: string
  - `path`: string
  - `args`: dictionary (optional)
    - `arg_name_X`: string
    - `value_X`: string
- `priority`: integer
- `input`: string and/or dictionary array (optional)
  - `src`: string (optional)
  - `filtered`: boolean (optional)
- `output`: string array (optional)
- `services`: string array (optional)

## Example
```yaml
  - remap: {from: avsm_dock, to: /avsm/dock}
  - behavior:
    name: dock
    machine: base
    launch: {pkg: 'agewell_hbba_cfg', path: 'launch/agewell_bhvr_dock.launch'}
    priority: 30
    input:
      - battery: {src: input/odometry/robotBattery, filtered: false}
      - objectsStamped: {src: perception/dockstation/detection, filtered: false}
      - camera_info: {src: nav_camera_node/camera_info, filtered: false}
      - odom: {src: input/odometry/odom, filtered: false}
      - reset_state: {src: /dock/reset_state, filtered: false}
    output:
      - output_auto_cmd_vel
      - avsm_dock
```
