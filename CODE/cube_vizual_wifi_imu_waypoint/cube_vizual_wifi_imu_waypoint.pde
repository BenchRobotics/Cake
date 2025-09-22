//Draw the path as the robot moves. Need serial connectioin. Pay attenting to +,- sign of the input angle.
import hypermedia.net.*;  // Import the UDP library
//import processing.serial.*;
import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

//Serial myPort;  // The serial port object
float x = 0;    // X position of the robot
float y = 0;    // Y position of the robot
float prevX;    // Previous X position for drawing lines
float prevY;    // Previous Y position for drawing lines
float theta = 0; // Robot's orientation in radians
float wheelBase = 0.056; // Distance between wheels in meters (11 cm)
float wheelRadius = 0.033; // Wheel radius in meters (8 cm diameter)
float scalingFactor; // Dynamic scaling factor for better visualization

///float rotation_scaling_factor = 6.28 /10;  // Adjusting 9 radians to 6.28 radians

// Variables to store previous angles
float prevAngleL = 0;
float prevAngleR = 0;

// List to keep track of the path
//ArrayList<PVector> path = new ArrayList<PVector>();

CopyOnWriteArrayList<PVector> path = new CopyOnWriteArrayList<PVector>();

ArrayList<PVector> waypoints = new ArrayList<PVector>();
// UDP object
UDP udp;  // Create a new UDP object

//UDP udpSend;  

String robotIP = "192.168.137.19";  // Replace with your robot's IP
int robotPort = 12345;           // Replace with your robot's listening port

// Constants
float angleTolerance = radians(10);  // Tolerance range for the robot's orientation in radians
float waypointRadius = 20;           // Radius within which the waypoint is considered reached (scaled to the screen)
int moveSpeed = 100;                // Speed to move towards the waypoint (0-100)
float waypointBoundary = 10;
int currentWaypointIndex = 0;  // Index of the waypoint being followed



PVector currentWaypoint = null;  // Current waypoint being followed

void setup() {
  size(800, 800);  // Set the window size
  background(255);

  // Initialize starting position to the center of the screen
  x = width / 2;
  y = height / 2;
  prevX = x;
  prevY = y;

  scalingFactor = 100;  // Adjust scaling as needed
  
  udp = new UDP(this, 12345);  // Create a new UDP object with the specified port
  udp.listen(true);  // Listen for incoming messages
  
  //udpSend = new UDP(this);  // UDP for sending commands
  //udpSend.send("Connected", robotPort, robotIP);  // Optional: Send a connection message
}

void draw() {
  // Clear the screen
  background(255);

  // Draw the robot and path
  stroke(255, 0, 0);
  noFill();
  beginShape();
  for (PVector p : path) {
    vertex(p.x, p.y);
  }
  endShape();
  drawRobot();

  // Draw all waypoints
  for (int i = 0; i < waypoints.size(); i++) {
    PVector waypoint = waypoints.get(i);

    // Highlight the current waypoint
    if (i == currentWaypointIndex) {
      fill(0, 255, 0);  // Green for the current waypoint
    } else {
      fill(0, 0, 255);  // Blue for other waypoints
    }

    // Draw the waypoint and boundary
    ellipse(waypoint.x, waypoint.y, 10, 10);  // Waypoint
    noFill();
    stroke(0, 255, 0);
    ellipse(waypoint.x, waypoint.y, waypointBoundary * 2, waypointBoundary * 2);  // Boundary
  }

  // Draw a line from the robot to the current waypoint
  if (!waypoints.isEmpty() && currentWaypointIndex < waypoints.size()) {
    PVector currentWaypoint = waypoints.get(currentWaypointIndex);
    stroke(0, 255, 0);
    line(x, y, currentWaypoint.x, currentWaypoint.y);
  }

  // Follow the current waypoint
  follow_waypoint();
}

void receive(byte[] data) {
  
 String inString = new String(data).trim();  // Convert the byte array to a string and trim it
  println("Received: " + inString);  // Print the received string for debugging
  
  
  if (inString != null) {
    inString = trim(inString);  // Remove any whitespace characters
    println("Received: " + inString);  // Print the received string for debugging

    String[] angles = split(inString, ',');  // Split the data by comma

    // Check if we received exactly two values
    if (angles.length == 2) {
      try {
        // Parse the angles and handle potential errors
        float angleL = float(angles[0])*-1;  // Left wheel angle in radians
        float angleR = float(angles[1]);  // Right wheel angle in radians
        //float heading = float(angles[2]);
        
        // Convert heading from -180 to 180 range to 0 to 360 range 
      //  if (heading < 0) 
    //  { 
    //    heading += 360; 
   //   }

        // Check for NaN values
        if (!Float.isNaN(angleL) && !Float.isNaN(angleR)) {
          // Calculate the difference from the previous angles (these are absolute values)
          float deltaAngleL = angleL - prevAngleL;
          float deltaAngleR = angleR - prevAngleR;

          // Update previous angles for the next iteration
          prevAngleL = angleL;
          prevAngleR = angleR;

          // Compute the distance each wheel has traveled
          float dL =   deltaAngleL * wheelRadius;  // Distance covered by left wheel (negated)
          float dR = deltaAngleR * wheelRadius;  // Distance covered by right wheel

          // Compute the robot's linear and angular displacement
          float dTheta = (dR - dL) / wheelBase;  // Change in orientation (radians)
          
          //theta = radians(heading);
          // Update only if there is a significant change
          if (abs(dTheta) > 0.0001 || abs(dL + dR) > 0.0001) {
            float dX = ((dL + dR) / 2) * cos(theta + dTheta / 2);  // Change in X
            float dY = ((dL + dR) / 2) * sin(theta + dTheta / 2);  // Change in Y

            // Update robot's position and orientation
            x += dX * scalingFactor;  // Apply scaling for visualization
            y += dY * scalingFactor;  // Apply scaling for visualization
            theta += dTheta/2;
            

            // Add the new position to the path list
            path.add(new PVector(x, y));
            if (path.size() > 1000) {  // Limit the path length to avoid memory issues
              path.remove(0);
            }
          }
          
        } else {
          println("Invalid data: NaN values received.");
        }
      } catch (NumberFormatException e) {
        println("Error parsing angles: " + e.getMessage());
      }
    } else {
      println("Invalid input: Expected two values separated by a comma.");
    }

    // Print the robot's position and orientation for debugging
    println("X: " + x + " Y: " + y + " Theta: " + theta);
  }
}

void drawRobot() {
  // Draw the robot as a rectangle
  pushMatrix();  // Save the current transformation matrix
  translate(x, y);  // Move to the robot's current position
  rotate((theta));  // Rotate the rectangle to match the robot's orientation
  fill(0, 0, 255);  // Fill color for the robot (blue)
  rectMode(CENTER);  // Draw the rectangle from the center
  rect(0, 0, 20, 10);  // Draw the robot as a rectangle (width 20, height 10)
    // Draw the front marker (circle)
  fill(255, 0, 0);  // Red for the front marker
  ellipse(10, 0, 6, 6);
  
  popMatrix();  // Restore the original transformation matrix
}

void drawWaypoints() {
  stroke(0, 255, 0); // Green color for lines
  fill(0, 255, 0);   // Green color for waypoints
  for (PVector waypoint : waypoints) {
    // Draw waypoint as a circle
    ellipse(waypoint.x, waypoint.y, 10, 10);

    // Draw line from waypoint to the robot
    line(waypoint.x, waypoint.y, x, y);

    // Display waypoint coordinates
    fill(0); // Black text
    textAlign(CENTER);
    text("(" + nf(waypoint.x, 0, 1) + ", " + nf(waypoint.y, 0, 1) + ")", waypoint.x, waypoint.y - 10);
  }
}
void mousePressed() {
  // Add a waypoint at the mouse position
  PVector newWaypoint = new PVector(mouseX, mouseY);
  waypoints.add(newWaypoint);
  println("Waypoint added: " + newWaypoint);
}


void keyPressed() {
  if (key == '1') {
    // Example: Move forward at 70% speed, with PID enabled and screen_select = 1
    send_command('f', 100, 0.1, 0.05, 0.01, 1, 1);
  } else if (key == '2') {
    // Example: Stop the robot with PID disabled and screen_select = 0
    send_command('s', 0, 0, 0, 0, 0, 0);
  }
}


void follow_waypoint() {
  // Check if there are waypoints and the current index is valid
  if (waypoints.isEmpty() || currentWaypointIndex >= waypoints.size()) {
    // Stop the robot if there are no waypoints or we've reached the last waypoint
    send_command('s', 0, 0, 0, 0, 0, 0);
    println("All waypoints reached.");
    return;
  }

  // Get the current waypoint
  PVector currentWaypoint = waypoints.get(currentWaypointIndex);

  // Calculate the angle to the waypoint
  float deltaX = currentWaypoint.x - x;
  float deltaY = currentWaypoint.y - y;
  float targetAngle = atan2(deltaY, deltaX);

  // Calculate the angle difference
  float angleDifference = targetAngle - theta;

  // Normalize the angle to the range [-PI, PI]
  angleDifference = (angleDifference + PI) % TWO_PI - PI;

  // Check if the robot is within the waypoint boundary
  float distanceToWaypoint = dist(x, y, currentWaypoint.x, currentWaypoint.y);
  if (distanceToWaypoint < waypointBoundary) {
    // Stop the robot and move to the next waypoint
    send_command('s', 0, 0, 0, 0, 0, 0);
    println("Reached waypoint: " + currentWaypointIndex);
    currentWaypointIndex++;
    return;
  }

  // Check if the robot is facing the waypoint within the tolerance range
  if (abs(angleDifference) > angleTolerance) {
    // Turn towards the waypoint
    if (angleDifference > 0) {
      send_command('l', moveSpeed, 0, 0, 0, 0, 1); // Rotate right
    } else {
      send_command('r', moveSpeed, 0, 0, 0, 0, 1); // Rotate left
    }
  } else {
    // Move towards the waypoint
    send_command('f', moveSpeed, 0, 0, 0, 0, 1);
  }
}

void send_command(char direction, int speed, float P, float I, float D, int screen_select, int enable_pid) {
  // Validate inputs
  if (speed < 0 || speed > 100) {
    println("Invalid speed. Must be between 0 and 100.");
    return;
  }
  if (screen_select != 0 && screen_select != 1) {
    println("Invalid screen_select. Must be 0 or 1.");
    return;
  }
  if (enable_pid != 0 && enable_pid != 1) {
    println("Invalid enable_pid. Must be 0 or 1.");
    return;
  }
  
  // Construct the command string
  String command = direction + "," + speed + "," + P + "," + I + "," + D + "," + screen_select + "," + enable_pid;

  // Send the command over UDP
  udp.send(command, robotIP, robotPort);

  // Debug output
 // println("Sent command: " + command);
}
