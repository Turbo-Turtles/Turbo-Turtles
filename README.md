# Motor Driver

A node that can be accessed universally if another node wants to control the motors. \
Later, an intelligence node will get all the information of the "input" nodes and then use this node.

Every other node can access it via the topic `/motor_driver`.

### Inputs

- speed
- turn

### Outouts

- linear (x, y, z)
- angular (x, y, z)
