import numpy as np
import sys
sys.path.insert(1, './src')
from screenActors import distance as dist

def bug1(screen, obstacleColor, robot, goal):
    """
    Implements the Bug 1 algorithm for robot navigation.
    Args:
        screen: The pygame surface where the robot will be drawn.
        obstacleColor: The color used to represent obstacles on the screen.
        robot: The robot object that will navigate the environment.
        goal: The goal object representing the target position.
    Returns:
        goalReached: A boolean indicating if the robot has reached the goal.
        goalCanBeReached: A boolean indicating if the goal can be reached.
    """
    goalReached      = robot.is_goal_reached(goal.get_position())
    goalCanBeReached = robot.goalCanBeReached

    if not goalReached and goalCanBeReached:
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
                goalCanBeReached       = not(robot.move_toward_goal(screen, goal.get_position(), obstacleColor))
                robot.goalCanBeReached = goalCanBeReached
        else:
            _ = robot.move_toward_goal(screen, goal.get_position(), obstacleColor)

    return goalReached, goalCanBeReached

def bug2(screen, obstacleColor, robot, goal):
    pass

def tangentBug(screen, obstacleColor, robot, goal):
    pass
    