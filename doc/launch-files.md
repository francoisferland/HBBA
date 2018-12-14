# Launch Files
These clauses add launch file include clauses at the beginning of the generated launch file. The include clause file attribute contains the specified path.

## Syntax
```yaml
  - launch:
    pkg: package_name
    path: launch_file_relative_path
    args:
      arg_name_1: value_1
      arg_name_2: value_2
      ...
```

## Types
- `pkg`: string
- `path`: string
- `args`: dictionary (optional)
  - `arg_name_X`: string
  - `value_X`: string

## Example
```yaml
  - launch: {pkg: 'beam', path: 'launch/beam_bringup_head.launch'}
```
