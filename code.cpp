// === Ultrasonic Pins ===
#define TRIG_FRONT 2
#define ECHO_FRONT 1
#define TRIG_LEFT 21
#define ECHO_LEFT 20
#define TRIG_RIGHT 42
#define ECHO_RIGHT 41

// === L298N Motor Driver Pins ===
#define IN1 14
#define IN2 17
#define IN3 18
#define IN4 19
#define ENA 13
#define ENB 12

// IR sensor pins
#define IR_L  7  // Left sensor
#define IR_ML 6  // Middle-left
#define IR_MR 5  // Middle-right
#define IR_R  4  // Right sensor

// === PID Constants ===
float Kp = 2.5;    // Reduced for more precise control
float Ki = 0.1;    // Small integral for steady state
float Kd = 0.8;    // Reduced derivative for smoother response

// === PID Variables ===
float error = 0, previous_error = 0, integral = 0;

// === Maze Navigation Variables ===
const float WALL_DISTANCE = 7.0;  // Wall detection distance
const float FRONT_SAFE_DISTANCE = 8.5;  // Safe distance from front wall
const int BASE_SPEED = 60;  // Reduced base speed for better control
const int FAST_SPEED = 70;  // Fast speed for open passages
const float DISTANCE_MARGIN = 2.0;  // Acceptable margin for centering
const int BACKWARD_SPEED = 120;  // Speed for moving backward
const int BACKWARD_TIME = 200;  // Time to move backward (ms)

// Line following PID variables
double setpoint = 0.0;   // Center position
double input = 0.0, output = 0.0;
double lastError = 0.0, integral_line = 0.0;

// Line following parameters
const double Kp_line = 15.0;  // Proportional gain for line following
const double Ki_line = 0.18;  // Small integral gain for stability
const double Kd_line = 1.8;   // Derivative gain for damping
const int BASE_SPEED_LINE = 100;   // Base speed for line following
const int MAX_SPEED_LINE = 120;   // Maximum speed for line following
const int MIN_SPEED_LINE = 32;    // Minimum speed for line following
const int TURN_SPEED_FACTOR = 1.95;  // Enhanced turn speed

// Line tracking variables
int lastLinePosition = 0;
int lineLostCount = 0;
const int MAX_LINE_LOST = 50;  // Increased threshold before 360° turn

// Junction detection variables
bool junctionDetected = false;
unsigned long junctionTime = 0;
const unsigned long JUNCTION_TIMEOUT = 1000; // 1 second timeout

// Right path preference variables
bool rightPathChoice = false;
unsigned long rightPathTime = 0;
const unsigned long RIGHT_PATH_DURATION = 500; // Duration to execute right turn

// Enhanced line condition tracking
int lastLineCondition = 1;
int consecutiveSameCondition = 0;

// 360-degree turn recovery variables
bool is360TurnActive = false;
unsigned long turn360StartTime = 0;
const unsigned long TURN_360_DURATION = 3200; // Time for complete 360° turn
const unsigned long POST_TURN_SEARCH_DURATION = 1000; // Time to search after 360° turn
unsigned long searchStartTime = 0;
bool isPostTurnSearch = false;

// Autonomous mode detection variables
enum RobotMode {
  LINE_FOLLOWING = 0,
  MAZE_NAVIGATION = 1
};

RobotMode currentMode = LINE_FOLLOWING;
RobotMode previousMode = LINE_FOLLOWING;
bool modeChanged = false;

// Mode detection thresholds
const float LINE_FOLLOWING_THRESHOLD = 25.0;  // If walls are >25cm, use line following
const float MAZE_NAVIGATION_THRESHOLD = 15.0;  // If walls are <15cm, use maze navigation
const int MODE_STABILITY_COUNT = 5;  // Number of consecutive readings to confirm mode
int lineFollowingCount = 0;
int mazeNavigationCount = 0;

// Distance history for mode detection
float leftDistHistory[5] = {0};
float rightDistHistory[5] = {0};
float frontDistHistory[5] = {0};
int historyIndex = 0;

void setup() {
  Serial.begin(115200);

  // Ultrasonic sensor pin modes
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Motor control pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize IR sensor pins
  pinMode(IR_L, INPUT);
  pinMode(IR_ML, INPUT);
  pinMode(IR_MR, INPUT);
  pinMode(IR_R, INPUT);

  stopMotors();  // Stop motors at start
  
  Serial.println("Super Autonomous Hybrid Robot Started");
  Serial.println("Auto Mode Detection: Line Following when walls >25cm, Maze Navigation when walls <15cm");
  Serial.println("Wall Distance: " + String(WALL_DISTANCE) + "cm");
  Serial.println("Base Speed: " + String(BASE_SPEED));
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout at 30ms
  float distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > 100) return 50.0;  // Default if invalid
  return distance;
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor direction
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }

  // Right motor direction
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  Serial.println("Moving BACKWARD to create space");
  setMotorSpeed(-BACKWARD_SPEED, -BACKWARD_SPEED);  // Move backward
  delay(BACKWARD_TIME);
}

void turnRight() {
  Serial.println("Turning RIGHT - Left wall too close or more space on right");
  setMotorSpeed(30, -30);  // Turn right
  delay(300);
}

void turnLeft() {
  Serial.println("Turning LEFT - Right wall too close or more space on left");
  setMotorSpeed(-30, 30);  // Turn left
  delay(300);
}

RobotMode detectOptimalMode(float leftDist, float rightDist, float frontDist) {
  // Update distance history
  leftDistHistory[historyIndex] = leftDist;
  rightDistHistory[historyIndex] = rightDist;
  frontDistHistory[historyIndex] = frontDist;
  historyIndex = (historyIndex + 1) % 5;

  // Calculate average distances for stability
  float avgLeftDist = 0, avgRightDist = 0, avgFrontDist = 0;
  for (int i = 0; i < 5; i++) {
    avgLeftDist += leftDistHistory[i];
    avgRightDist += rightDistHistory[i];
    avgFrontDist += frontDistHistory[i];
  }
  avgLeftDist /= 5;
  avgRightDist /= 5;
  avgFrontDist /= 5;

  // Check if we're in a confined space (maze navigation)
  bool isConfined = (avgLeftDist < MAZE_NAVIGATION_THRESHOLD || 
                     avgRightDist < MAZE_NAVIGATION_THRESHOLD || 
                     avgFrontDist < FRONT_SAFE_DISTANCE);

  // Check if we're in open space (line following)
  bool isOpenSpace = (avgLeftDist > LINE_FOLLOWING_THRESHOLD && 
                      avgRightDist > LINE_FOLLOWING_THRESHOLD && 
                      avgFrontDist > FRONT_SAFE_DISTANCE);

  // Mode detection logic
  if (isConfined) {
    mazeNavigationCount++;
    lineFollowingCount = 0;
    if (mazeNavigationCount >= MODE_STABILITY_COUNT) {
      return MAZE_NAVIGATION;
    }
  } else if (isOpenSpace) {
    lineFollowingCount++;
    mazeNavigationCount = 0;
    if (lineFollowingCount >= MODE_STABILITY_COUNT) {
      return LINE_FOLLOWING;
    }
  } else {
    // In between - maintain current mode
    mazeNavigationCount = 0;
    lineFollowingCount = 0;
  }

  return currentMode; // Keep current mode if not enough stable readings
}

void loop() {
  // === Read all sensor data ===
  float leftDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  float rightDist = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
  float frontDist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);

  // === Autonomous Mode Detection ===
  RobotMode detectedMode = detectOptimalMode(leftDist, rightDist, frontDist);
  
  // Check if mode should change
  if (detectedMode != currentMode) {
    previousMode = currentMode;
    currentMode = detectedMode;
    modeChanged = true;
    
    Serial.print("Mode Change: ");
    if (previousMode == LINE_FOLLOWING) Serial.print("LINE_FOLLOWING");
    else Serial.print("MAZE_NAVIGATION");
    Serial.print(" -> ");
    if (currentMode == LINE_FOLLOWING) {
      Serial.println("LINE_FOLLOWING");
      Serial.println("Reason: Open space detected (walls >25cm)");
    } else {
      Serial.println("MAZE_NAVIGATION");
      Serial.println("Reason: Confined space detected (walls <15cm)");
    }
    
    // Reset all states when mode changes
    stopMotors();
    delay(500);
    lineLostCount = 0;
    is360TurnActive = false;
    isPostTurnSearch = false;
    rightPathChoice = false;
    junctionDetected = false;
    integral = 0;
    integral_line = 0;
    lastError = 0;
    previous_error = 0;
  }

  // === Execute Current Mode ===
  if (currentMode == LINE_FOLLOWING) {
    handleLineFollowing();
  } else {
    handleMazeNavigation(leftDist, rightDist, frontDist);
  }

  // Print mode and sensor info
  Serial.print("Mode: ");
  if (currentMode == LINE_FOLLOWING) Serial.print("LINE_FOLLOWING");
  else Serial.print("MAZE_NAVIGATION");
  Serial.print(" | L:"); Serial.print(leftDist, 1);
  Serial.print(" R:"); Serial.print(rightDist, 1);
  Serial.print(" F:"); Serial.print(frontDist, 1);
  Serial.println(" cm");
}

void handleLineFollowing() {
  // Read sensor states (0 = black line, 1 = white)
  int left = digitalRead(IR_L);
  int midLeft = digitalRead(IR_ML);
  int midRight = digitalRead(IR_MR);
  int right = digitalRead(IR_R);

  // Check if we're in the middle of a 360° turn
  if (is360TurnActive) {
    handle360Turn(left, midLeft, midRight, right);
    return; // Skip normal line following during 360° turn
  }

  // Check if we're in post-turn search mode
  if (isPostTurnSearch) {
    handlePostTurnSearch(left, midLeft, midRight, right);
    return; // Skip normal line following during search
  }

  // Check if we're executing a right path choice
  if (rightPathChoice) {
    handleRightPathChoice();
    return; // Skip normal line following during right path execution
  }

  // Calculate line position
  input = getLinePosition(left, midLeft, midRight, right);

  // Detect line condition with enhanced logic
  int lineCondition = detectLineCondition(left, midLeft, midRight, right);

  // Track consecutive conditions for stability
  if (lineCondition == lastLineCondition) {
    consecutiveSameCondition++;
  } else {
    consecutiveSameCondition = 0;
    lastLineCondition = lineCondition;
  }

  // Log sensor data
  logSensorData(left, midLeft, midRight, right, input, lineCondition);

  // Handle line following based on condition
  switch (lineCondition) {
    case 0: // No line
      handleLineLost();
      break;
    case 1: // Straight line
      followStraightLine();
      break;
    case 2: // Sharp turn
      followSharpTurn(input);
      break;
    case 3: // Acute turn
      followAcuteTurn(input);
      break;
    case 4: // Wide line
      followWideLine();
      break;
    case 5: // 45-degree angle
      followFortyFiveDegree(input);
      break;
    case 6: // Path choice (straight + right available)
      initiateRightPathChoice();
      break;
    case 7: // 4-way intersection
      handleIntersection();
      break;
  }

  delay(20); // Optimized delay for responsiveness
}

void handleMazeNavigation(float leftDist, float rightDist, float frontDist) {
  // === Front Obstacle Detection ===
  if (frontDist < FRONT_SAFE_DISTANCE) {
    Serial.println("Front obstacle detected, finding new path...");
    moveBackward();  // Move backward to create space
    // Re-read distances after moving backward
    leftDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
    rightDist = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
    // Choose direction based on which side has more space
    if (leftDist > rightDist) {
      turnLeft();  // More space on left, turn left
    } else {
      turnRight();  // More space on right, turn right
    }
    return;
  }

  // === Check for Open Passage (Fast Mode) ===
  if (leftDist > WALL_DISTANCE && rightDist > WALL_DISTANCE && frontDist > FRONT_SAFE_DISTANCE) {
    Serial.println("Open passage detected! Moving fast forward...");
    setMotorSpeed(FAST_SPEED, FAST_SPEED);  // Move forward at fast speed
    integral = 0;  // Reset integral in open space
    return;
  }

  // === Check if centered between walls ===
  if (abs(leftDist - rightDist) <= DISTANCE_MARGIN && leftDist <= WALL_DISTANCE + DISTANCE_MARGIN && rightDist <= WALL_DISTANCE + DISTANCE_MARGIN) {
    Serial.println("Centered! Moving forward...");
    setMotorSpeed(BASE_SPEED, BASE_SPEED);  // Move straight
    integral = 0;  // Reset integral when centered
    return;
  }

  // === Left Wall Too Close ===
  if (leftDist < WALL_DISTANCE) {
    turnRight();
    return;
  }

  // === Right Wall Too Close ===
  if (rightDist < WALL_DISTANCE) {
    turnLeft();
    return;
  }

  // === Wall Following using PID ===
  error = leftDist - rightDist;  // Error is difference between left and right distances
  integral += error;
  integral = constrain(integral, -50, 50);  // Anti-windup
  float derivative = error - previous_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Limit correction to prevent sudden movements
  correction = constrain(correction, -40, 40);
  previous_error = error;

  // Calculate motor speeds
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  // Ensure minimum speed for movement
  leftSpeed = constrain(leftSpeed, 10, 255);
  rightSpeed = constrain(rightSpeed, 10, 255);

  setMotorSpeed(leftSpeed, rightSpeed);

  // Print PID values for debugging
  Serial.print("Error: "); Serial.print(error, 2);
  Serial.print(" | Correction: "); Serial.print(correction, 2);
  Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
  Serial.print(" | Right Speed: "); Serial.println(rightSpeed);

  delay(50);  // Smoother loop timing
}

void handle360Turn(int left, int midLeft, int midRight, int right) {
  // Check if any sensor detects a line during the turn
  if (left == 0 || midLeft == 0 || midRight == 0 || right == 0) {
    // Line found during 360° turn - stop turning and resume normal operation
    Serial.println("Line found during 360° turn - resuming normal operation");
    is360TurnActive = false;
    lineLostCount = 0;
    return;
  }

  // Check if 360° turn is complete
  if (millis() - turn360StartTime >= TURN_360_DURATION) {
    // 360° turn completed - start post-turn search
    Serial.println("360° turn completed - starting search mode");
    is360TurnActive = false;
    isPostTurnSearch = true;
    searchStartTime = millis();
    return;
  }

  // Continue 360° turn (clockwise)
  int turnSpeed = BASE_SPEED_LINE * 0.8; // Moderate speed for controlled turn
  setMotorSpeed(turnSpeed, -turnSpeed);
  
  // Progress indicator
  unsigned long elapsed = millis() - turn360StartTime;
  int progress = (elapsed * 100) / TURN_360_DURATION;
  if (progress % 25 == 0 && (elapsed % 500) < 50) { // Print every 25% progress
    Serial.print("360° Turn Progress: "); Serial.print(progress); Serial.println("%");
  }
}

void handlePostTurnSearch(int left, int midLeft, int midRight, int right) {
  // Check if any sensor detects a line during search
  if (left == 0 || midLeft == 0 || midRight == 0 || right == 0) {
    // Line found during search - resume normal operation
    Serial.println("Line found during post-turn search - resuming normal operation");
    isPostTurnSearch = false;
    lineLostCount = 0;
    return;
  }

  // Check if search time is expired
  if (millis() - searchStartTime >= POST_TURN_SEARCH_DURATION) {
    // Search failed - stop the robot
    Serial.println("Post-turn search failed - stopping robot");
    isPostTurnSearch = false;
    stopMotors();
    return;
  }

  // Continue searching with oscillating movement
  unsigned long searchElapsed = millis() - searchStartTime;
  int searchSpeed = BASE_SPEED_LINE * 0.4;
  
  // Oscillate left and right to search for line
  if ((searchElapsed / 200) % 2 == 0) {
    // Turn left
    setMotorSpeed(-searchSpeed, searchSpeed);
    Serial.println("Searching left...");
  } else {
    // Turn right  
    setMotorSpeed(searchSpeed, -searchSpeed);
    Serial.println("Searching right...");
  }
}

void handleRightPathChoice() {
  // Check if right path turn is complete
  if (millis() - rightPathTime >= RIGHT_PATH_DURATION) {
    // Right turn completed - resume normal operation
    Serial.println("Right path choice completed - resuming normal line following");
    rightPathChoice = false;
    return;
  }
  
  // Execute right turn
  int turnSpeed = BASE_SPEED_LINE * 1.3;
  setMotorSpeed(turnSpeed, -turnSpeed);
  Serial.println("Executing right path choice turn...");
}

double getLinePosition(int left, int midLeft, int midRight, int right) {
  int totalSensors = left + midLeft + midRight + right;
  if (totalSensors == 0) return lastLinePosition; // Hold last position if line lost

  double position = (-3 * left) + (-1 * midLeft) + (1 * midRight) + (3 * right);
  position /= totalSensors; // Normalize to -3 to +3
  lastLinePosition = position;
  return position;
}

int detectLineCondition(int left, int midLeft, int midRight, int right) {
  int totalSensors = left + midLeft + midRight + right;

  if (totalSensors == 0) {
    lineLostCount++;
    return 0;
  }
  lineLostCount = 0;

  // Enhanced junction detection
  if (left == 0 && right == 0 && midLeft == 0 && midRight == 0) {
    // All sensors detect line - 4-way intersection
    if (!junctionDetected) {
      junctionDetected = true;
      junctionTime = millis();
    }
    return 7; // 4-way intersection
  }

  // Path choice detection - when robot can go straight OR turn right
  // This detects when there are multiple paths available, preferring right turn
  if ((midLeft == 1 && midRight == 0) && // Center sensors see line (straight path available)
      (right == 1 || (right == 1 && midRight == 1))) { // Right sensor also sees line (right path available)
    
    // Additional validation for stable detection
    if (consecutiveSameCondition >= 2) {
      Serial.println("Path choice detected: Straight + Right available - Choosing RIGHT");
      return 6; // Path choice - will trigger right turn
    }
  }

  // Enhanced 45-degree angle detection
  if ((left == 0 && midLeft == 0 && midRight == 1 && right == 1) ||
      (left == 1 && midLeft == 1 && midRight == 0 && right == 0) ||
      (left == 0 && midLeft == 1 && midRight == 0 && right == 1) ||
      (left == 1 && midLeft == 0 && midRight == 1 && right == 0)) {
    return 5; // 45-degree angle
  }

  if (totalSensors == 4) return 4; // Wide line
  if ((left == 0 && right == 0) || (midLeft == 0 && midRight == 0)) return 2; // Sharp turn
  if (totalSensors == 1) return 3; // Acute turn
  return 1; // Straight line
}

void initiateRightPathChoice() {
  // Reset any recovery states
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  // Start right path choice
  rightPathChoice = true;
  rightPathTime = millis();
  
  Serial.println("Initiating right path choice - turning right at junction");
}

void logSensorData(int left, int midLeft, int midRight, int right, double pos, int cond) {
  Serial.print("L:"); Serial.print(left);
  Serial.print(" ML:"); Serial.print(midLeft);
  Serial.print(" MR:"); Serial.print(midRight);
  Serial.print(" R:"); Serial.print(right);
  Serial.print(" Pos:"); Serial.print(pos, 2);
  Serial.print(" Cond:"); Serial.print(cond);
  
  // Add condition names for better debugging
  switch(cond) {
    case 0: Serial.print("(NO_LINE)"); break;
    case 1: Serial.print("(STRAIGHT)"); break;
    case 2: Serial.print("(SHARP_TURN)"); break;
    case 3: Serial.print("(ACUTE_TURN)"); break;
    case 4: Serial.print("(WIDE_LINE)"); break;
    case 5: Serial.print("(45_DEGREE)"); break;
    case 6: Serial.print("(PATH_CHOICE)"); break;
    case 7: Serial.print("(INTERSECTION)"); break;
  }
  
  // Add lost line count for debugging
  if (cond == 0) {
    Serial.print(" LostCount:"); Serial.print(lineLostCount);
  }
  
  Serial.println();
}

void handleLineLost() {
  if (lineLostCount > MAX_LINE_LOST && !is360TurnActive && !isPostTurnSearch) {
    // Initiate 360° turn recovery
    Serial.println("Line lost for extended period - starting 360° recovery turn");
    is360TurnActive = true;
    turn360StartTime = millis();
    lineLostCount = 0; // Reset counter
  } else if (!is360TurnActive && !isPostTurnSearch) {
    // Normal recovery behavior
    int recoverySpeed = BASE_SPEED_LINE * 0.5;
    setMotorSpeed(lastLinePosition < 0 ? -recoverySpeed : recoverySpeed, 
                   lastLinePosition < 0 ? recoverySpeed : -recoverySpeed);
    Serial.print("Recovering... Count: "); Serial.println(lineLostCount);
  }
}

void followStraightLine() {
  // Reset any recovery states when line is found
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  double error = setpoint - input;
  integral_line += error;
  double derivative = error - lastError;
  output = Kp_line * error + Ki_line * integral_line + Kd_line * derivative;
  lastError = error;

  output = constrain(output, -MAX_SPEED_LINE, MAX_SPEED_LINE);
  int leftSpeed = BASE_SPEED_LINE + output;
  int rightSpeed = BASE_SPEED_LINE - output;

  setMotorSpeed(constrain(leftSpeed, MIN_SPEED_LINE, MAX_SPEED_LINE),
                 constrain(rightSpeed, MIN_SPEED_LINE, MAX_SPEED_LINE));
  Serial.print("PID: Err="); Serial.print(error, 2); Serial.print(" Out="); Serial.println(output, 2);
}

void followSharpTurn(double position) {
  // Reset recovery states when following line
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  int turnSpeed = BASE_SPEED_LINE * TURN_SPEED_FACTOR;
  if (position < -1.5) {
    setMotorSpeed(-turnSpeed, turnSpeed);
    Serial.println("Sharp Left Turn");
  } else if (position > 1.5) {
    setMotorSpeed(turnSpeed, -turnSpeed);
    Serial.println("Sharp Right Turn");
  } else {
    followStraightLine();
  }
}

void followAcuteTurn(double position) {
  // Reset recovery states when following line
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  int turnSpeed = BASE_SPEED_LINE * 1.3;
  setMotorSpeed(position < 0 ? -turnSpeed : turnSpeed, position < 0 ? turnSpeed : -turnSpeed);
  Serial.println(position < 0 ? "Acute Left Turn" : "Acute Right Turn");
}

void followFortyFiveDegree(double position) {
  // Reset recovery states when following line
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  // Enhanced 45-degree handling with adaptive speed
  int turnSpeed = BASE_SPEED_LINE * 1.4; // Slightly faster than acute turns
  
  // Use position to determine turn direction and intensity
  if (position < -0.8) {
    // Left 45-degree turn
    setMotorSpeed(-turnSpeed, turnSpeed);
    Serial.println("45-Degree Left Turn");
  } else if (position > 0.8) {
    // Right 45-degree turn
    setMotorSpeed(turnSpeed, -turnSpeed);
    Serial.println("45-Degree Right Turn");
  } else {
    // Gentle 45-degree curve - use PID
    double error = setpoint - position;
    integral_line += error;
    double derivative = error - lastError;
    output = Kp_line * error + Ki_line * integral_line + Kd_line * derivative;
    lastError = error;
    
    output = constrain(output, -MAX_SPEED_LINE * 0.8, MAX_SPEED_LINE * 0.8);
    int leftSpeed = BASE_SPEED_LINE * 0.9 + output;
    int rightSpeed = BASE_SPEED_LINE * 0.9 - output;
    
    setMotorSpeed(constrain(leftSpeed, MIN_SPEED_LINE, MAX_SPEED_LINE),
                   constrain(rightSpeed, MIN_SPEED_LINE, MAX_SPEED_LINE));
    Serial.println("45-Degree Gentle Curve");
  }
}

void handleIntersection() {
  // Reset recovery states when at intersection
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  // Check for junction timeout
  if (millis() - junctionTime > JUNCTION_TIMEOUT) {
    junctionDetected = false;
    return;
  }
  
  // 4-way intersection handling - prefer right turn
  int turnSpeed = BASE_SPEED_LINE * 1.8;
  
  // Always turn right at 4-way intersections
  setMotorSpeed(turnSpeed, -turnSpeed);
  Serial.println("4-Way Intersection: Turning Right (preferred)");
  
  // Reset junction detection after a short delay
  delay(400);
  junctionDetected = false;
}

void followWideLine() {
  // Reset recovery states when following line
  lineLostCount = 0;
  is360TurnActive = false;
  isPostTurnSearch = false;
  
  int wideSpeed = BASE_SPEED_LINE * 0.7;
  setMotorSpeed(wideSpeed, wideSpeed);
  Serial.println("Wide Line - Slow Forward");
}
