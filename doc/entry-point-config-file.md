# Entry-Point Configuration File

This file should contain:
- Global launch clauses
- Global remap clauses
- Resource definition
- Includes clauses
- Initial desire definition

## Example

```yaml
hbba_conf:

- launch: {pkg: 'beam', path: 'launch/beam_bringup_head.launch'}
- launch: {pkg: 'agewell_hbba_cfg', path: 'launch/agewell_base_nodes.launch'}

# Global remaps:
- remap: {from: output_auto_cmd_vel, to: /output/auto/cmd_vel}

# Available resources:
- resources:
    speaker: 1
    mic: 1
    base_controller: 1

# Common includes:
- include: {pkg: 'hbba_synth', file: 'common.yaml'}

# Proc modules:
- include: {pkg: 'agewell_hbba_cfg', file: 'agewell_proc_manual_cmd.yaml'}

# Behaviors:
- include: {pkg: 'agewell_hbba_cfg', file: 'agewell_bhvr_goto.yaml'}

# Motivations:
- include: {pkg: 'agewell_hbba_cfg', file: 'agewell_motv_manual_cmd.yaml'}

# Strategies:
- include: {pkg: 'agewell_hbba_cfg', file: 'agewell_strategies.yaml'}

# Initial desires:
- desire:
    id: slam_static
    type: SLAM
    utility: 1
    intensity: 1.0
    security: False
```


