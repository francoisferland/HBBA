# Custom Scripts
These clauses are used to define JavaScript functions that can be called in the `custom_bringup` and `custom_bringdn` fields.

## Syntax
```yaml
  - custom_script: javascript_code
```

## Types
- `custom_script`: string

## Example
```yaml
  - custom_script: "function enableBase() {pubBoolean('/base/enable', true);}"
```

# Stragegies
These clauses define strategies achieving desires. The JavaScript code in the `custom_bringup` and `custom_bringdn` fields get access to the desire parameters with the global variable named `params`.

## Syntax
```yaml
  - strat:
    name: stragegy_name
    class: desire_class
    utility: strategy_utility
    costs:
      resource_name_1: required_quantity
      resource_name_2: required_quantity
      ...
    dependencies:
      desire_name_1: minimal_required_utility
      ...
    modules:
      - perception_node_name_that_must_be_activated_1:
        - input_topic_name_1: filter_rate
        - input_topic_name_2: filter_rate
        ...
      - perception_node_name_that_must_be_activated_2:
        - input_topic_name_1: filter_rate
        - input_topic_name_2: filter_rate
        ...
      - behavior_node_name_that_must_be_activated_1
      - behavior_node_name_that_must_be_activated_2
      ...
    custom_bringup: javascript_code_that_is_executed_on_the_strategy_activation
    custom_bringdn: javascript_code_that_is_executed_on_the_strategy_deactivation
```

## Types
- `name`: string
- `class`: string
- `utility`: integer
- `costs`: dictionary (optional)
  - `resource_name_X`: string
  - required_quantity: floating-point number
- `dependencies`: dictionary (optional)
  - `desire_name_X`: string
  - `minimal_required_utility`: integer
- `modules`: string and/or dictionary array (optional)
  - `perception_node_name_that_must_be_activated_X`: string
    - `input_topic_name_X`: string
    - `filter_rate`: integer
  - `behavior_node_name_that_must_be_activated_X`: string
- `custom_bringup`: string (optional)
- `custom_bringdn`: string (optional)

## Example
```yaml
  - strat:
    name: ActionSelectionMobileBase
    class: ActionSelectionMobileBase
    utility: 1
    custom_bringup: "enableActionSelectionMobileBase();"
    custom_bringdn: "disableActionSelectionMobileBase();"

  - strat:
    name: GoTo
    class: GoTo
    utility: 1
    costs:
        base_controller: 1
    dependencies: {ActionSelectionMobileBase: 1}
    modules:
        - goto
    custom_bringup: "pubNavGoal('/g', params.frame, params.x, params.y, params.t);"
```
