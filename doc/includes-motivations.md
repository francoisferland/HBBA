# Motivations
These clauses define motivation modules. The input and output topics are not filtered.

Notes:
- The input and output topics can be renamed by global remap clauses.
- The specified launch file must not contain any remap clauses.
- The topic name must not contain "/".

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
      ...
    output:
      - node_and_global_topic_name
      ...
```

## Types
- `name`: string
- `launch`: dictionary
  - `pkg`: string
  - `path`: string
  - `args`: dictionary (optional)
    - `arg_name_X`: string
    - `value_X`: string
- `input`: string array (optional)
- `output`: string array (optional)

## Example
```yaml
  - remap: {from: motv_manual_cmd, to: /manual/cmd}
  - remap: {from: motv_goto_goal, to: /move_base_simple/goal}
  - motivation:
    name: manual_cmd_motivations
    launch: {pkg: 'agewell_hbba_cfg', path: 'launch/agewell_motv_manual_cmd.launch'}
    input:
      - motv_manual_cmd
      - motv_goto_goal
```
