# EG2310 - Fundamentals of Systems Design

### Overview

This is a ROS2 project on building engineering systems which solve tasks in real-world scenarios. The mission is for the Turtlebot to deliver a can from a fixed dispenser to a specific table autonomously in a simulated restaurant environment. To accomplish this robotic system, my team and I devised the following subsystems: Dispenser Storage, Dispatching, Turtlebot Load-sensing Holder and Turtlebot Navigation. More information in the [Final Assessment Rubrics](/documents/EG2310%20Final%20Assessment%20rubrics.pdf)

### Conceptual Design

- Dispenser Storage Subsystem <br>
  The dispenser is built using dimensioned acrylic boards held together by metal brackets. It stores a single can drink in its storage location, supported by 2 servo motors. It is responsible for dispensing the can into the Turtlebot's Holder Subsystem upon its arrival.
- Dispatching Subsystem <br>
  We built a mobile application using the [MIT App Inventor](https://appinventor.mit.edu/) as a tool to send information regarding the table delivery number to the Turtlebot via MQTT.
- Turtlebot Load-sensing Holder Subsystem <br>
  A can holder built on the Turtlebot detects when the can has been received from the dispenser. The button within the can holder is connected to the RPi's GPIO pins. When the button is pushed from the weight of the can in the can holder, it triggers the navigation of the Turtlebot.
- Turtlebot Navigation Subsystem <br>
  Waypoints are set onto the map by manually driving the Turtlebot around the map and saving coordinates at the appropriate locations. Depending on the table number being pressed in the dispatching phase, the Turtlebot follows that set of predetermined waypoints to reach the target location. Following the same algorithm, the Turtlebot navigates back to the dispenser after its delivery.

### Other information

More information can be found in the "documents" tab. For full details on our project and setup instructions, view our [Final Report](/documents/EG2310_G10_FinalReport.pdf).
