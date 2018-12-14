# Includes
These clauses append the content of other configuration files to the current one.

Notes:
- The specified configuration file must be in the package `hbba_cfg` folder.

## Syntax
```yaml
  - include: {pkg: package_name, file: filename}
```

## Types
- `pkg`: string
- `file`: string

## Example
```yaml
  - include: {pkg: 'hbba_synth', file: 'common.yaml'}
```
