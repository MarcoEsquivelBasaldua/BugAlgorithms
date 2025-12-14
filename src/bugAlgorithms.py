import numpy as np
import sys
sys.path.insert(1, './src')
from screenActors import distance as dist

def bug1(screen, obstacleColor, robot, goal):
    goalReached = robot.is_goal_reached(goal.get_position())

    if not goalReached:
        # Check collision
        collision, collisionAngles = robot.check_collision(screen, obstacleColor)

        if collision:
            mustFollowObstacle = True
            # Check if obstacle has been encircled
            if len(robot.hitPoints) > robot.minHitPoints:
                if dist(robot.hitPoints[-1][0], robot.hitPoints[0][0]) < 3.0:
                    robot.obsEncircled = True
            
            if robot.obsEncircled:
                if dist(robot.pos, goal.pos) <= (robot.minDist2Goal + 2):
                    mustFollowObstacle = False

            if mustFollowObstacle:
                robot.follow_obstacle_boundary(screen, goal.get_position(), collisionAngles, obstacleColor)
            else:
                robot.move_toward_goal(screen, goal.get_position())
        else:
            robot.move_toward_goal(screen, goal.get_position())

    return goalReached

def bug2(start, goal, obstacles):
    pass

def tangentBug(start, goal, obstacles):
    pass
    