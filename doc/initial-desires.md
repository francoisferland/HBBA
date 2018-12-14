# Initial Desires
These clauses define desires added at the system startup.

## Syntax
```yaml
  - desire:
    id: desire_id
    type: desire_class
    utility: desire_utility
    intensity: desire_intensity
    params: desire_parameters
    security: true_or_false
```

## Types
- `id`: string
- `type`: string
- `utility`: integer
- `intensity`: floating-point number
- `params`: JSON string (optional)
- `security`: boolean (optional)

## Example
```yaml
  - desire:
    id: slam_static
    type: SLAM
    utility: 1
    intensity: 1.0
    security: False
```
