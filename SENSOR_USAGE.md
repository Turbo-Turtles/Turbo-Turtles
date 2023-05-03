# Sensor usage for each mission

The specific tasks of the TurtleBot's sensors in each mission are described below.

### Base mission
<ins>camera detection</ins>
- lane markings
	- yellow -> left
	- white -> right

### Traffic mission
<ins>camera detection</ins>
- traffic sign
	- red -> top
	- orange/yellow -> mid
	- green -> bottom

### Intersection mission
<ins>camera detection</ins>
- traffic sign
	- left arrow
	- right arrow

### Construction mission
<ins>LiDAR detection</ins>
- obstacle
- maybe mapping for better pathing

### Parking mission
<ins>camera detection</ins>
- traffic sign
	- left arrow
- lane marking
	- parking slots (e.g. dash lines)
- other bot?

<ins>LiDAR detection</ins>
- obstacle / other bot

### Level Cross mission
<ins>camera detection</ins>
- bark
	- up -> open
	- down -> closed

### Tunnel mission
<ins>LiDAR detection</ins>
- obstacles
- maybe mapping for better pathing
	- exit is in oposite corner of entry

---
#### <ins>Occuring Signs:</ins>
- traffic light
  - shape: rectangle
  - states: red, orange/yellow, green
- section signs
  - shape: triangle
  - states: intersection, construction site, tunnel
- parking sign
  - shape: rectangle
  - states: -
- arrow signs
  - shape: circle
  - states: left, right
- bark
  - shape: multiple red and white rectangles
  - states: closed, open


## Conclusion

As we can see, it comes down to traffic sign and lane marking detection with the camera and obstacle detection and mapping with the LiDAR.
