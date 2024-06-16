# maze_traversal_algorithms

This project involves Webots by Cyberbotics and the use of a robotic partner that can map and traverse a maze in the shortest amount of time to get to the center goal. Concepts utilized in this project involve the wavefront expansion algorithm in order to determine the possible routes to the center goal.

Along the way, clever heuristics are employed in order to map the fastest as well as traverse the maze the fastest. For mapping, the robot can detect walls when it is within the center of a given tile. That way, it can determine whether each tile has walls on each of its four sides. In that sense, it can "determine" the mapping of some future tiles if its neighboring four adjacent tiles on its sides have already been mapped.

The output of the Mapping.py file is map_structure.json, which the next file Run.py utilizes in order to read in the overall structure of the entire map.
