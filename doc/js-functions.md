# `custom_bringup` and `custom_bringdn` Built-in Functions

## `pubEmpty(topic)`

### Types
- `topic`: string

## `pubBoolean(topic, value)`

### Types
- `topic`: string
- `value`: boolean

## `pubInt8(topic, value)`

### Types
- `topic`: string
- `value`: integer

## `pubInt16(topic, value)`

### Types
- `topic`: string
- `value`: integer

## `pubInt32(topic, value)`

### Types
- `topic`: string
- `value`: integer

## `pubInt64(topic, value)`

### Types
- `topic`: string
- `value`: integer

## `pubFloat32(topic, value)`

### Types
- `topic`: string
- `value`: floating-point number

## `pubFloat64(topic, value)`

### Types
- `topic`: string
- `value`: floating-point number

## `pubString(topic, value)`

### Types
- `topic`: string
- `value`: string

## `pubNavGoal(topic, frame_id, x, y, yaw)`

### Types
- `topic`: string
- `frame_id`: string
- `x`: floating-point number
- `y`: floating-point number
- `yaw`: floating-point number

## `pubDuration(topic, seconds)`

### Types
- `topic`: string
- `seconds`: floating-point number

## `pubDuration(topic, seconds, nanoseconds)`

### Types
- `topic`: string
- `seconds`: integer
- `nanoseconds`: integer

## `call_eval(service, source)`

### Types
- `service`: string
- `source`: string

## `call_empty(service)`

### Types
- `service`: string

## `call_boolean(service, value)`

### Types
- `service`: string
- `value`: boolean

## `call_update_rate(service, rate)`

### Types
- `service`: string
- `rate`: floating-point number

## `sys(cmd)`

### Types
- `cmd`: string
