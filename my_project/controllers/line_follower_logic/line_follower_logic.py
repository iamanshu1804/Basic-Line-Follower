from controller import Robot
from collections import deque

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- HARDWARE SETUP ---
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

ir_left = robot.getDevice('ir_left')
ir_center = robot.getDevice('ir_center')
ir_right = robot.getDevice('ir_right')

ir_left.enable(timestep)
ir_center.enable(timestep)
ir_right.enable(timestep)

# --- VARIABLES ---
# Standard cruising speed. Adjust if it's too fast or too slow!
BASE_SPEED = 6.0 
THRESHOLD = 600

# --- HISTORY ARRAY SETUP ---
# Calculate how many loop iterations happen in 2000 ms (2 seconds)
history_length = int(500 / timestep) 

# Create the queue. It will hold tuples of (left, center, right)
sensor_history = deque(maxlen=history_length)

# --- MAIN CONTROL LOOP ---
while robot.step(timestep) != -1:
    
    # 1. Read Sensors
    val_l = ir_left.getValue()
    val_c = ir_center.getValue()
    val_r = ir_right.getValue()
    
    # 2. Convert to Boolean (True if it sees the black line)
    line_l = val_l > THRESHOLD
    line_c = val_c > THRESHOLD
    line_r = val_r > THRESHOLD
    
    # 3. Default state is to drive straight
    # (Notice I kept your fix: making the left motor negative so it drives forward!)
    left_speed = -BASE_SPEED 
    right_speed = -BASE_SPEED
    
    # Save the current readings into the history array
    # Because of maxlen, the oldest reading is automatically deleted when a new one is added!
    sensor_history.append((val_l, val_c, val_r))
    
    # The oldest data is at index 0. The newest data is at index -1.
    # oldest_readings
    if len(sensor_history) == history_length:
        oldest_readings = list(sensor_history[0])
        
    
    # 4. State Machine Logic
    if line_c and not line_l and not line_r:
        # Perfect center: drive straight (keep default speeds)
        pass 
        
    elif line_l and not line_r:
        # Drifting right (Left sensor hit the line). Turn LEFT.
        # To turn left, we slow down the left motor (or reverse it) and speed up the right.
        left_speed = -BASE_SPEED * 0.2  
        right_speed = -BASE_SPEED
        
    elif line_r and not line_l:
        # Drifting left (Right sensor hit the line). Turn RIGHT.
        # To turn right, we speed up the left motor and slow down the right.
        left_speed = -BASE_SPEED 
        right_speed = -BASE_SPEED * 0.2 
        
    elif not line_l and not line_c and not line_r:
        # Lost the line entirely! Usually, we just keep doing whatever we were doing, 
        # or we spin in place to find it. Let's just slow down for safety.
            if sensor_history[0][0]>THRESHOLD: 
                left_speed = BASE_SPEED 
                right_speed = -BASE_SPEED 
            elif sensor_history[0][2]>THRESHOLD:
                left_speed = -BASE_SPEED 
                right_speed = BASE_SPEED 
        
    # 5. Apply the speeds to the motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # print(f"left value: {val_l} | center value: {val_c} | right value: {val_r}")
    print(f"Left: {val_l:.2f} | Center: {val_c:.2f} | Right: {val_r:.2f}")