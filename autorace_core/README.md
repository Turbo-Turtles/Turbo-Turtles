# Autorace Core

The central point to start all nodes and services.

Here is a state machine defined to control all processes and avoid interferences between them. \
But it is unfortunatley not entirely tested, because not all the nodes are working and fully integrated into the system.

### Not finshed!

If you came to get a completely working solution for the autorace 2020 track, you are at the wrong place. \
But you can learn from it!

### Tips

- instead of a slam approach with the lidar, you should stick to little prerecorded snippets/maps and use amcl, because if the bot hits something even once, the entire slam map is ruined and not usable.
- remember to train your image recognition with the correct image resolution
- do not create a giant one-file lane_detection, rather more specific smaller ones
And the last and most important tip:
- if you work in a team, take care so everyone is working from the beginning and create an integreation plan early so everyone can shape his code towards the interfaces and spot better solutions
