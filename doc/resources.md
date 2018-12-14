# Resources
This clause defines the robot available resources that the HBBA module uses to optimize the activated strategies. The resources can be used to make sure that devices are used by strategies one at a time when the motivation nodes don't handle this constraint.

## Syntax
```yaml
  - resources:
    resource_name_1: quantity_1
    resource_name_2: quantity_2
    ...
```

## Types
- `resource_name_X`: string
- `quantity`: floating-point number

## Example
```yaml
  - resources:
    speaker: 1
    mic: 1
    base_controller: 1
```
