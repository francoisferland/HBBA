# `custom_bringup` and `custom_bringdn` Built-in Functions

## `pubEmpty(topic)`
This function publishes a message of type `std_msgs/Empty` on the specified topic.

### Types
- `topic`: string

## `pubBoolean(topic, value)`
This function publishes a message of type `std_msgs/Bool` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: boolean

## `pubInt8(topic, value)`
This function publishes a message of type `std_msgs/Int8` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: integer

## `pubInt16(topic, value)`
This function publishes a message of type `std_msgs/Int16` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: integer

## `pubInt32(topic, value)`
This function publishes a message of type `std_msgs/Int32` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: integer

## `pubInt64(topic, value)`
This function publishes a message of type `std_msgs/Int64` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: integer

## `pubFloat32(topic, value)`
This function publishes a message of type `std_msgs/Float32` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: floating-point number

## `pubFloat64(topic, value)`
This function publishes a message of type `std_msgs/Float64` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: floating-point number

## `pubString(topic, value)`
This function publishes a message of type `std_msgs/String` containing the specified value on the specified topic.

### Types
- `topic`: string
- `value`: string

## `pubNavGoal(topic, frame_id, x, y, yaw)`
This function publishes a message of type `geometry_msgs/PoseStamped` containing the specified values on the specified topic.

### Types
- `topic`: string
- `frame_id`: string
- `x`: floating-point number
- `y`: floating-point number
- `yaw`: floating-point number

## `pubDuration(topic, seconds)`
This function publishes a message of type `std_msgs/Duration` containing the specified value on the specified topic.

### Types
- `topic`: string
- `seconds`: floating-point number

## `pubDuration(topic, seconds, nanoseconds)`
This function publishes a message of type `std_msgs/Duration` containing the specified values on the specified topic.

### Types
- `topic`: string
- `seconds`: integer
- `nanoseconds`: integer

## `call_eval(service, source)`
This function calls the specified service of type `hbba_msgs/EvalScript`. The request contains the specified JavaScript code.

### Types
- `service`: string
- `source`: string

## `call_empty(service)`
This function calls the specified service of type `std_srvs/Empty`.

### Types
- `service`: string

## `call_boolean(service, value)`
This function calls the specified service of type `hbba_msgs/Boolean`. The request contains the specified value.

### Types
- `service`: string
- `value`: boolean

## `call_update_rate(service, rate)`
This function calls the specified service of type `hbba_msgs/UpdateRate`. The request contains the specified value.

### Types
- `service`: string
- `rate`: floating-point number

## `sys(cmd)`
This function executes the specified system command.

### Types
- `cmd`: string
