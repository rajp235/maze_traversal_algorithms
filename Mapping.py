# imports
from controller import Robot
import numpy as np
import pickle

#######################################################
# Creates Robot
#######################################################
robot = Robot()


#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()


print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

######################################################################



size = 16
size_of_cell = 0.18*39.3701 #inches
starting_seconds = robot.getTime()


def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*np.pi
    return rad_out*180/np.pi


def pos_to_cell(x_pos, y_pos):
    mid_size = size / 2
    if y_pos > 0:
        y_coeff = np.floor(y_pos / size_of_cell)
        y_coeff = y_coeff - (mid_size-1)
        y_coeff = -1*y_coeff
    else:
        y_pos = -1*y_pos
        y_coeff = np.floor(y_pos / size_of_cell)
        y_coeff = mid_size + y_coeff
    
    if x_pos > 0:
        x_coeff = np.floor(x_pos / size_of_cell)
        x_coeff = (mid_size+1) + x_coeff
    else:
        x_pos = -1*x_pos
        x_coeff = np.floor(x_pos / size_of_cell)
        x_coeff = x_coeff - (mid_size)
        x_coeff = -1*x_coeff

    # CHANGE IF NEEDED
    max_x_coeff = 16
    min_x_coeff = 0
    max_y_coeff = 15
    min_y_coeff = 0
    if y_coeff < min_y_coeff:
        y_coeff = min_y_coeff
    if y_coeff > max_y_coeff:
        y_coeff = max_y_coeff
    if x_coeff < min_x_coeff:
        x_coeff = min_x_coeff
    if x_coeff > max_x_coeff:
        x_coeff = max_x_coeff
    
    output_return = y_coeff*size + x_coeff
    return int(output_return)


def print_diagram(diagram):
    output = ""
    for i in range(size):
        for j in range(size):
            output = output + diagram[size*i + j]
        output = output + '\n'
    print(output)


def cell_to_points(cell):
    mid_size = size / 2

    remainder = cell % (size)
    if remainder == 0:
        x_val = ((mid_size - 1)*size_of_cell + 0.5*size_of_cell)
    elif remainder <= mid_size:
        coeff = mid_size - remainder
        x_val = -1*(coeff*size_of_cell + 0.5*size_of_cell)
    else:
        coeff = remainder - (mid_size+1)
        x_val = (coeff*size_of_cell + 0.5*size_of_cell)

    divided = cell / size
    integer = np.ceil(divided)
    if integer <= mid_size:
        coeff = mid_size - integer
        y_val = (coeff*size_of_cell + 0.5*size_of_cell)
    else:
        coeff = integer - (mid_size+1)
        y_val = -1*(coeff*size_of_cell + 0.5*size_of_cell)
    
    return (x_val, y_val)


def move_options(cell):
    top = cell-size
    left = cell-1
    right = cell+1
    bottom = cell+size
    options = {"top": top, "left": left, "right": right, "bottom": bottom}

    # trim top edge
    top_cells = list(range(1, size+1))
    if cell in top_cells:
        options["top"] = "WALL"

    # trim left edge
    end_cell = size*size - size + 1
    left_cells = list(range(1, end_cell+1))
    to_remove = []
    for item in left_cells:
        if item % size != 1:
            to_remove.append(item)
    for item in to_remove:
        left_cells.remove(item)
    if cell in left_cells:
        options["left"] = "WALL"

    # trim right edge
    end_cell = size*size
    right_cells = list(range(size, end_cell+1))
    to_remove = []
    for item in right_cells:
        if item % size != 0:
            to_remove.append(item)
    for item in to_remove:
        right_cells.remove(item)
    if cell in right_cells:
        options["right"] = "WALL"

    # trim bottom edge
    last_cell = size*size
    begin_cell = last_cell - size + 1
    bottom_cells = list(range(begin_cell, last_cell + 1))
    if cell in bottom_cells:
        options["bottom"] = "WALL"

    return options


# size of map
size = 16
size_of_cell = 0.18*39.3701 #inches
total_cells = size*size
all_cells = []
for i in range(total_cells):
    all_cells.append(i+1)

# adjacent list graph representation
adjacent_list = {}
for i in range(size*size):
    adjacent_list[i+1] = move_options(i+1)

# constants
time_before = robot.getTime()
left_before = 0
right_before = 0
diameter_wheel = 1.6
radius_wheel = diameter_wheel / 2
d_between = 2.28
d_mid = d_between / 2
pi = 3.14



# ----- USER INPUTS -----

# starting position config 1
cell_to_start = 241
starting_position = cell_to_points(cell_to_start)
# config 2
# starting_position = (-15, -15)

# ----- USER INPUTS -----




x_pos = starting_position[0]
y_pos = starting_position[1]
starting_cell = pos_to_cell(x_pos, y_pos)
current_cell = starting_cell
diagram = ['.' for i in range(total_cells)]
diagram[starting_cell - 1] = 'X'

# print if entered new cell
print("--- DIAGRAM ---")
print_diagram(diagram)
IMU_pos = imu.getRollPitchYaw()[2]
IMU_pos_deg = IMU_pos*180/np.pi
print("Robot pose:\t", "x_pos:", x_pos, " y_pos", y_pos, " n:", starting_cell, " theta:", IMU_pos_deg)

# reoreint to facing upwards
while robot.step(timestep) != -1:
    leftMotor.setVelocity(2)
    rightMotor.setVelocity(-2)
    IMU_pos = imu.getRollPitchYaw()[2]
    IMU_pos_deg = IMU_pos*180/np.pi
    if IMU_pos_deg >= 89.5 and IMU_pos_deg <= 90.5:
        leftMotor.setVelocity(0.1)
        rightMotor.setVelocity(-0.1)
        break
    if IMU_pos_deg >= 89.99 and IMU_pos_deg <= 90.01:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break

facing = "up"
stack = []
stack.append([current_cell])
visited_cells = []
visited_cells.append(starting_cell)
first_time = True
taken_route = []
new_time = True
currently = False
degree = 0

# state
state = "determine"
move_back = False
path = []
switch = False


def DFS(start, end):
    stack_dfs = []
    stack_dfs.append([start])
    discovered = []
    discovered.append(start)
    taken_route_dfs = []
    while len(stack_dfs) != 0:
        taken_route_dfs = stack_dfs.pop()
        current_loc = taken_route_dfs[-1]
        if current_loc == end:
            break
        for child in adjacent_list[current_loc].values():
            if child != "WALL" and child not in discovered and child in visited_cells:
                new_route_dfs = taken_route_dfs.copy()
                new_route_dfs.append(child)
                discovered.append(child)
                stack_dfs.append(new_route_dfs)
    return taken_route_dfs


def wavefront_path(start, end):
    queue = []
    best_path = []
    wave_front_values = {}
    for i in range(1, size*size+1):
        if i == end:
            wave_front_values[i] = 2
        else:
            wave_front_values[i] = 0
    queue.append(end)
    while len(queue) != 0:
        this_cell = queue.pop(0)
        for neighbor in adjacent_list[this_cell].values():
            if neighbor != "WALL" and (neighbor in visited_cells or neighbor == end):
                if wave_front_values[neighbor] == 0:
                    wave_front_values[neighbor] = wave_front_values[this_cell] + 1
                    queue.append(neighbor)
    best_path.append(start)
    this_cell = start
    while end not in best_path:
        possible_list = []
        for neighbor in adjacent_list[this_cell].values():
            if neighbor != "WALL" and (neighbor in visited_cells or neighbor == end):
                possible_list.append(neighbor)
        best_so_far = wave_front_values[possible_list[0]]
        next_cell = possible_list[0]
        for possibility in possible_list:
            possibility_value = wave_front_values[possibility]
            if possibility_value < best_so_far and possibility_value != 0:
                best_so_far = possibility_value
                next_cell = possibility
        this_cell = next_cell
        best_path.append(this_cell)

    return best_path


# Main loop
while robot.step(timestep) != -1:
    # distance sensors
    full_range_image = lidar.getRangeImage()
    front_dist = frontDistanceSensor.getValue()*39.3701
    right_dist = rightDistanceSensor.getValue()*39.3701
    rear_dist = rearDistanceSensor.getValue()*39.3701
    left_dist = leftDistanceSensor.getValue()*39.3701
    front_dist = full_range_image[0]*39.3701
    right_dist = full_range_image[90]*39.3701
    rear_dist = full_range_image[180]*39.3701
    left_dist = full_range_image[270]*39.3701

    # determine
    if state == "determine":
        if len(path) == 0:
            move_back = False
        if len(stack) != 0 and len(visited_cells) != (size*size):
            if not move_back:
                taken_route = stack.pop()
                go_to_cell = taken_route[len(taken_route) - 1]
                saved_go_to_cell = go_to_cell
                # if needs to backtrack
                if go_to_cell not in adjacent_list[current_cell].values() and not first_time:
                    stack.append(taken_route)
                    shortest_path = wavefront_path(current_cell, taken_route[-2])
                    shortest_path_line = taken_route
                    for path_line in stack:
                        potential_path = wavefront_path(current_cell, path_line[-2])
                        if len(potential_path) < len(shortest_path):
                            shortest_path = potential_path
                            shortest_path_line = path_line
                    stack.remove(shortest_path_line)
                    stack.append(shortest_path_line)
                    path = shortest_path
                    path.pop(0)
                    move_back = True

                    # path = DFS(current_cell, taken_route[-2])
                    # path = wavefront_path(current_cell, taken_route[-2])
                    # path.pop(0)
                    # move_back = True
                    # stack.append(taken_route)
            if move_back and len(path) != 0:
                go_to_cell = path.pop(0)
            target_cell = go_to_cell
            state = "reoreint"
        else:
            end_seconds = robot.getTime()
            no_visited = [item2 for item2 in all_cells if item2 not in visited_cells]
            for item3 in no_visited:
                adjacent_list[item3]["bottom"] = "WALL"
                adjacent_list[item3]["left"] = "WALL"
                adjacent_list[item3]["right"] = "WALL"
                adjacent_list[item3]["top"] = "WALL"
            print("Finished")
            print("Start time:", starting_seconds, "End time:", end_seconds)
            print("Delta time taken:", (end_seconds - starting_seconds))
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    
    if state == "update_stack":
        if not move_back:
            item_left = adjacent_list[current_cell]["left"]
            item_right = adjacent_list[current_cell]["right"]
            item_top = adjacent_list[current_cell]["top"]
            item_bottom = adjacent_list[current_cell]["bottom"]
            if item_left != "WALL" and item_left not in visited_cells:
                new_route = taken_route.copy()
                new_route.append(item_left)
                stack.append(new_route)
            if item_right != "WALL" and item_right not in visited_cells:
                new_route = taken_route.copy()
                new_route.append(item_right)
                stack.append(new_route)
            if item_top != "WALL" and item_top not in visited_cells:
                new_route = taken_route.copy()
                new_route.append(item_top)
                stack.append(new_route)
            if item_bottom != "WALL" and item_bottom not in visited_cells:
                new_route = taken_route.copy()
                new_route.append(item_bottom)
                stack.append(new_route)
        
            # delete routes in which unvisited cell's characteristics can be inferred
            adjacent_cells = move_options(current_cell)
            can_update = []
            for list_item in adjacent_cells.values():
                if list_item != "WALL" and list_item not in visited_cells:
                    add_list = True
                    surrounding_cells = move_options(list_item)
                    for list_item_2 in surrounding_cells.values():
                        if list_item_2 != "WALL":
                            if list_item_2 not in visited_cells:
                                add_list = False
                    if add_list:
                        can_update.append(list_item)
            for cell_item in can_update:
                surrounding_cells = move_options(cell_item)
                if surrounding_cells["top"] != "WALL":
                    top_char = adjacent_list[surrounding_cells["top"]]["bottom"]
                    if top_char == "WALL":
                        adjacent_list[cell_item]["top"] = "WALL"
                if surrounding_cells["left"] != "WALL":
                    left_char = adjacent_list[surrounding_cells["left"]]["right"]
                    if left_char == "WALL":
                        adjacent_list[cell_item]["left"] = "WALL"
                if surrounding_cells["right"] != "WALL":
                    right_char = adjacent_list[surrounding_cells["right"]]["left"]
                    if right_char == "WALL":
                        adjacent_list[cell_item]["right"] = "WALL"
                if surrounding_cells["bottom"] != "WALL":
                    bottom_char = adjacent_list[surrounding_cells["bottom"]]["top"]
                    if bottom_char == "WALL":
                        adjacent_list[cell_item]["bottom"] = "WALL"

                # mark as visited
                if cell_item not in visited_cells:
                    visited_cells.append(cell_item)
                    diagram[cell_item - 1] = 'X'
                    print("--- DIAGRAM ---")
                    print_diagram(diagram)
                    IMU_pos = imu.getRollPitchYaw()[2]
                    IMU_pos_deg = IMU_pos*180/np.pi
                    print("x_pos:", x_pos, " y_pos:", y_pos, " n:", current_cell, " theta:", IMU_pos_deg, "--- Inferred cell:", cell_item)

                # remove from stack
                for stack_item in stack:
                    last_item = stack_item[-1]
                    if last_item == cell_item:
                        stack.remove(stack_item)

        state = "determine"

    # update adjacent list for walls
    if state == "update_for_walls":
        wall_vector = []
        if left_dist < size_of_cell:
            wall_vector.append("WALL")
        else:
            wall_vector.append("OPEN")
        if front_dist < size_of_cell:
            wall_vector.append("WALL")
        else:
            wall_vector.append("OPEN")
        if right_dist < size_of_cell:
            wall_vector.append("WALL")
        else:
            wall_vector.append("OPEN")
        if rear_dist < size_of_cell:
            wall_vector.append("WALL")
        else:
            wall_vector.append("OPEN")
        
        if facing == "up":
            left_result = wall_vector[0]
            top_result = wall_vector[1]
            right_result = wall_vector[2]
            bottom_result = wall_vector[3]
        elif facing == "left":
            left_result = wall_vector[1]
            top_result = wall_vector[2]
            right_result = wall_vector[3]
            bottom_result = wall_vector[0]
        elif facing == "down":
            left_result = wall_vector[2]
            top_result = wall_vector[3]
            right_result = wall_vector[0]
            bottom_result = wall_vector[1]
        elif facing == "right":
            left_result = wall_vector[3]
            top_result = wall_vector[0]
            right_result = wall_vector[1]
            bottom_result = wall_vector[2]

        if left_result == "WALL":
            adjacent_list[current_cell]["left"] = "WALL"
        if top_result == "WALL":
            adjacent_list[current_cell]["top"] = "WALL"
        if right_result == "WALL":
            adjacent_list[current_cell]["right"] = "WALL"
        if bottom_result == "WALL":
            adjacent_list[current_cell]["bottom"] = "WALL"

        state = "update_stack"

    if state == "reoreint":
        if first_time:
            state = "move"
            continue
        elif True:
            # code taken from motionLibrary provided by Research Assistant and altered
            axel_length = 2.28
            wheel_radius = 1.6/2

            if degree == 0:
                # reoreint if angle not within range
                x_from = cell_to_points(current_cell)[0]
                y_from = cell_to_points(current_cell)[1]
                x_target = cell_to_points(target_cell)[0]
                y_target = cell_to_points(target_cell)[1]
                x_diff = x_target - x_from
                y_diff = y_target - y_from

                if x_diff == 0:
                    if y_diff > 0:
                        needed_angle = 90
                        need_facing = "up"
                    elif y_diff < 0:
                        needed_angle = -90
                        need_facing = "down"
                elif y_diff == 0:
                    if x_diff > 0:
                        needed_angle = 0
                        need_facing = "right"
                    elif x_diff < 0:
                        needed_angle = 180
                        need_facing = "left"

                if facing == need_facing:
                    degree = 0
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    state = "move"
                    continue
                else:
                    if facing == "up":
                        if need_facing == "left":
                            degree = -90
                        elif need_facing == "right":
                            degree = 90
                        else:
                            degree = 180
                    elif facing == "down":
                        if need_facing == "right":
                            degree = -90
                        elif need_facing == "left":
                            degree = 90
                        else:
                            degree = 180
                    elif facing == "right":
                        if need_facing == "up":
                            degree = -90
                        elif need_facing == "down":
                            degree = 90
                        else:
                            degree = 180
                    elif facing == "left":
                        if need_facing == "down":
                            degree = -90
                        elif need_facing == "up":
                            degree = 90
                        else:
                            degree = 180

                if degree < 0:
                    sign = -1
                else:
                    sign = 1
                X_rad = degree*np.pi/180
                phi = sign*2

                # CHANGE IF NEEDED
                phi = sign*2

                # Calculates time need for rotation
                omega = 2*abs(phi)*wheel_radius / axel_length
                T = abs(X_rad / omega)
                # end_heading = (predicted_pose[3] - degree)%360

                t_start = robot.getTime()
                leftMotor.setVelocity(phi)
                rightMotor.setVelocity(-phi)

                starting_theta = round(imu_cleaner(imu.getRollPitchYaw()[2]))
                end_heading = round((starting_theta - degree)%360,2)

                # CHANGE IF NEEDED
                marg_error = 0.01
                facing = need_facing


            # loop
            leftMotor.setVelocity(phi)
            rightMotor.setVelocity(-phi)
            current_heading = imu_cleaner(imu.getRollPitchYaw()[2])
            east_flag = True if end_heading <= 4 or end_heading >= 356 else False
            if (robot.getTime() - t_start) >= T:
                if east_flag:
                    current_heading = current_heading - 360 if current_heading > 355 else current_heading
                    end_heading = end_heading - 360 if end_heading > 355 else end_heading
                if current_heading > (end_heading+marg_error):
                    leftMotor.setVelocity(marg_error)
                    rightMotor.setVelocity(-1*marg_error)
                elif current_heading < (end_heading-marg_error):
                    leftMotor.setVelocity(-1*marg_error)
                    rightMotor.setVelocity(marg_error)
                else:
                    degree = 0
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    state = "move"
        else:
            # reoreint if angle not within range
            x_from = cell_to_points(current_cell)[0]
            y_from = cell_to_points(current_cell)[1]
            x_target = cell_to_points(target_cell)[0]
            y_target = cell_to_points(target_cell)[1]
            x_diff = x_target - x_from
            y_diff = y_target - y_from

            if x_diff == 0:
                if y_diff > 0:
                    needed_angle = 90
                    need_facing = "up"
                elif y_diff < 0:
                    needed_angle = -90
                    need_facing = "down"
            elif y_diff == 0:
                if x_diff > 0:
                    needed_angle = 0
                    need_facing = "right"
                elif x_diff < 0:
                    needed_angle = 180
                    need_facing = "left"

            if facing == need_facing:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                state = "move"
                continue
            else:
                if facing == "up":
                    if need_facing == "left":
                        need_turn = "left"
                    else:
                        need_turn = "right"
                elif facing == "down":
                    if need_facing == "right":
                        need_turn = "left"
                    else:
                        need_turn = "right"
                elif facing == "right":
                    if need_facing == "up":
                        need_turn = "left"
                    else:
                        need_turn = "right"
                else:
                    if need_facing == "down":
                        need_turn = "left"
                    else:
                        need_turn = "right"
                    
            current_angle = imu.getRollPitchYaw()[2]*180/np.pi
            if current_angle < 0:
                current_angle = 180 + current_angle + 180
            if needed_angle < 0:
                needed_angle = 180 + needed_angle + 180
            bound1 = needed_angle - 0.5
            bound2 = needed_angle + 0.5
            cond1 = current_angle < bound1
            cond2 = current_angle > bound2

            # while robot.step(timestep) != -1:
            #     leftMotor.setVelocity(2)
            #     rightMotor.setVelocity(-2)
            #     IMU_pos = imu.getRollPitchYaw()[2]
            #     IMU_pos_deg = IMU_pos*180/np.pi
            #     if IMU_pos_deg >= 89.5 and IMU_pos_deg <= 90.5:
            #         leftMotor.setVelocity(0.1)
            #         rightMotor.setVelocity(-0.1)
            #         break
            #     if IMU_pos_deg >= 89.99 and IMU_pos_deg <= 90.01:
            #         leftMotor.setVelocity(0)
            #         rightMotor.setVelocity(0)
            #         break

            if(current_angle < bound1 or current_angle > bound2):
                if need_turn == "left":
                    leftMotor.setVelocity(-1)
                    rightMotor.setVelocity(1)
                else:
                    leftMotor.setVelocity(1)
                    rightMotor.setVelocity(-1)
                current_angle = imu.getRollPitchYaw()[2]*180/np.pi
                if current_angle < 0:
                    current_angle = 180 + current_angle + 180
            else:
                facing = need_facing
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                state = "move"

    if state == "move":
        # move 10 inches, print if entered new cell, update current cell,
        # set to determine
        if first_time:
            state = "update_for_walls"
            first_time = False
            continue
        elif False:
            if new_time:
                current_distance = 0
                new_time = False
                current_time = robot.getTime()
                target_time = current_time + 10/(5*radius_wheel)
            target_distance = size_of_cell
            leftMotor.setVelocity(5)
            rightMotor.setVelocity(5)

            time_after = robot.getTime()
            left_after = leftposition_sensor.getValue()
            right_after = rightposition_sensor.getValue()
            time_diff = time_after - time_before
            left_diff = left_after - left_before
            right_diff = right_after - right_before
            if time_diff == 0 or (left_diff == 0 and right_diff == 0):
                pass
            else:
                left_speed = (left_diff / time_diff) * radius_wheel
                right_speed = (right_diff / time_diff) * radius_wheel
                robot_speed = (left_speed + right_speed) / 2
                robot_speed = abs(robot_speed)
                x_distance_moved = (robot_speed * np.cos(imu.getRollPitchYaw()[2])) * time_diff
                y_distance_moved = (robot_speed * np.sin(imu.getRollPitchYaw()[2])) * time_diff
                x_pos = x_pos + x_distance_moved
                y_pos = y_pos + y_distance_moved
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi
                current_distance += robot_speed * time_diff

            # update before variables
            left_before = left_after
            right_before = right_after
            time_before = time_after
            current_time += time_diff

            current_cell = pos_to_cell(x_pos, y_pos)
            if current_cell not in visited_cells:
                visited_cells.append(current_cell)
                diagram[current_cell - 1] = 'X'
                print("--- DIAGRAM ---")
                print_diagram(diagram)
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi
                print("x_pos:", x_pos, " y_pos:", y_pos, " n:", current_cell, " theta:", IMU_pos_deg)
            
            # if current_distance > target_distance:
            #     leftMotor.setVelocity(0)
            #     rightMotor.setVelocity(0)
            #     new_time = True
            #     state = "update_for_walls"
            if current_time > target_time:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                new_time = True
                state = "update_for_walls"
        elif True:
            if not currently:
                start_position = leftposition_sensor.getValue()
                currently = True
                time_before = robot.getTime()
                left_before = leftposition_sensor.getValue()
                right_before = rightposition_sensor.getValue()

            # obtained from Research Assistant's motionLibrary function driveD()
            wheel_radius = 1.6/2
            D = size_of_cell
            V = 5.024
            phi = V / wheel_radius
            # T = D / V
            leftMotor.setVelocity(phi)
            rightMotor.setVelocity(phi)

            # CHANGE IF NEEDED
            if left_dist < 1:
                rightMotor.setVelocity(phi-0.1)
            if right_dist < 1:
                leftMotor.setVelocity(phi-0.1)

            # update position
            time_after = robot.getTime()
            left_after = leftposition_sensor.getValue()
            right_after = rightposition_sensor.getValue()
            time_diff = time_after - time_before
            left_diff = left_after - left_before
            right_diff = right_after - right_before
            if time_diff == 0 or (left_diff == 0 and right_diff == 0):
                pass
            else:
                left_speed = (left_diff / time_diff) * radius_wheel
                right_speed = (right_diff / time_diff) * radius_wheel
                robot_speed = (left_speed + right_speed) / 2
                robot_speed = abs(robot_speed)
                x_distance_moved = (robot_speed * np.cos(imu.getRollPitchYaw()[2])) * time_diff
                y_distance_moved = (robot_speed * np.sin(imu.getRollPitchYaw()[2])) * time_diff
                x_pos = x_pos + x_distance_moved
                y_pos = y_pos + y_distance_moved
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi
            left_before = left_after
            right_before = right_after
            time_before = time_after

            current_cell = pos_to_cell(x_pos, y_pos)
            if current_cell not in visited_cells:
                visited_cells.append(current_cell)
                diagram[current_cell - 1] = 'X'
                print("--- DIAGRAM ---")
                print_diagram(diagram)
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi
                print("x_pos:", x_pos, " y_pos:", y_pos, " n:", current_cell, " theta:", IMU_pos_deg)
                for path_line in stack:
                    last_in_line = path_line[-1]
                    if last_in_line in visited_cells:
                        stack.remove(path_line)

            # obtained from Research Assistant's motionLibrary function driveD()
            # Checks if wheel distance is larger than D
            if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                currently = False
                state = "update_for_walls"
        else:
            # move forward
            if facing == "up":
                dist_cam = y_target - y_pos
            if facing == "down":
                dist_cam = y_pos - y_target
            if facing == "left":
                dist_cam = x_pos - x_target
            if facing == "right":
                dist_cam = x_target - x_pos
            error = dist_cam
            if error < -5:
                error = -5
            if error > 5:
                error = 5
            leftMotor.setVelocity(error)
            rightMotor.setVelocity(error)
            
            # get time and encoder information to calculate distance
            time_after = robot.getTime()
            left_after = leftposition_sensor.getValue()
            right_after = rightposition_sensor.getValue()

            time_diff = time_after - time_before
            left_diff = left_after - left_before
            right_diff = right_after - right_before
            if time_diff == 0 or (left_diff == 0 and right_diff == 0):
                pass
            else:
                left_speed = (left_diff / time_diff) * radius_wheel
                right_speed = (right_diff / time_diff) * radius_wheel
                robot_speed = (left_speed + right_speed) / 2
                robot_speed = abs(robot_speed)
                x_distance_moved = (robot_speed * np.cos(imu.getRollPitchYaw()[2])) * time_diff
                y_distance_moved = (robot_speed * np.sin(imu.getRollPitchYaw()[2])) * time_diff
                x_pos = x_pos + x_distance_moved
                y_pos = y_pos + y_distance_moved
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi

            # update before variables
            left_before = left_after
            right_before = right_after
            time_before = time_after

            current_cell = pos_to_cell(x_pos, y_pos)
            if current_cell not in visited_cells:
                visited_cells.append(current_cell)
                diagram[current_cell - 1] = 'X'
                print("--- DIAGRAM ---")
                print_diagram(diagram)
                IMU_pos = imu.getRollPitchYaw()[2]
                IMU_pos_deg = IMU_pos*180/np.pi
                print("x_pos:", x_pos, " y_pos:", y_pos, " n:", current_cell, " theta:", IMU_pos_deg)
            
            if error < 0.5:
                state = "update_for_walls"


# saved object json or pickle
import json
with open("../map_structure.json", "w+") as outfile:
    json.dump(adjacent_list, outfile, indent=2)