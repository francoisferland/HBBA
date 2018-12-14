# Remap Clauses
These clauses rename global topics or a global services.

## Syntax
```yaml
  - remap: {from: initial_name, to: new_name}
```

## Types
- `from`: string
- `to`: string

## Example
```yaml
  - remap: {from: output_manual_cmd_vel, to: /output/manual/cmd_vel}
```
