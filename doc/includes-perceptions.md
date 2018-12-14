# Perceptions
These clauses define perception modules. The input topics are filtered by default.

Notes:
- The output topics can be renamed by global remap clauses.
- The specified launch file must not contain any remap clauses.
- The topic name must not contain "/" if the topic is filtered.

## Syntax
```yaml
  - procmodule:
    name: node_name
    machine: filters_execution_machine_name
    launch:
      pkg: package_name
      path: launch_file_relative_path
      args:
        arg_name_1: value_1
        arg_name_2: value_2
        ...
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
- `input`: string and/or dictionary array (optional)
  - `src`: string (optional)
  - `filtered`: boolean (optional)
- `output`: string array (optional)
- `services`: string array (optional)

## Example
```yaml
  - remap: {from: cmd_vel, to: /nav/auto/cmd_vel}
  - procmodule:
    name: move_base
    machine: base
    launch: {pkg: 'agewell_hbba_cfg', path: 'launch/agewell_proc_move_base.launch'}
    input: 
      - odom: {src: input/odometry/odom, filtered: false}
      - map: {filtered: false}
      - scan: {src: kinect2/scan, filtered: false}
      - move_base_simple/goal: {filtered: false}
    output:
      - cmd_vel
      - move_base/GlobalPlanner/plan
      - move_base/status
```
