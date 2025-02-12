# CMPUT 412: Assignment 2


## Publisher and subscriber:
- The **Publisher (`my_publisher_node.py`)** sends a message every second to the `/chatter` topic.
- The **Subscriber (`my_subscriber_node.py`)** listens to this topic and logs the received message.

### How to run:
- First build the code `dts devel build -H csc22907.local -f`

- - In one terminal run  `dts devel run -H csc22907.local -L my-subscriber`
- Then run the publisher in one terminal using `dts devel run -H csc22907 -L my-publisher -n publisher`
      - It should say something like `Publishing message: 'Hello from csc22907!`  
      - The subscriber should say something like `I heard 'Hello from ROBOT_NAME!'`
  This shows that the publisher and subscriber are woking good.

### Camera processed image:
- Run the command `dts devel run -H csc22907.local -L camera-reader`
- On the other terminal, run `dts start_gui_tools`
- Then use `rqt_image_view ` to view the processed image 

## Straight and rotate:
Purpose: go straight for 1.25m, reverse for 1.25m, rotate 90 degrees then 90 degrees back to original place
The bag file for this run is moveS.D

### How to run:
-  Run the command: `dts devel run -H csc22907.local -L straight-line-task`
-  The robot will go straight for 1.25m, reverse for 1.25m, and rotate counterclock wise and clockwise and then come to stop

## D-tracing
Purpose: around in D-shape

### How to run:
-  Run the command: `dts devel run -H csc22907.local -L D-trajectory`
-  The robot will trace d shape expecifically :
    - Start from the left of the D
    - Move straight for 1.2m
    - turn sharp 90 degree
    - move straight for 0.92m 
    - curve right
    - move straight for 0.92m
    - turn 90 to the right
    The bag file of this run in moveD.bag