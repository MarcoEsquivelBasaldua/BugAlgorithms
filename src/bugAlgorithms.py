import numpy as np
import sys
sys.path.insert(1, './src')
from screenActors import distance as dist
from screenActors import wrap_angle

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
    robot.bug1Active = True
    robot.bug2Active = False

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
    """
    Implements the Bug 2 algorithm for robot navigation.
    Args:
        screen: The pygame surface where the robot will be drawn.
        obstacleColor: The color used to represent obstacles on the screen.
        robot: The robot object that will navigate the environment.
        goal: The goal object representing the target position.
    Returns:
        goalReached: A boolean indicating if the robot has reached the goal.
        goalCanBeReached: A boolean indicating if the goal can be reached.
    """
    robot.bug1Active = False
    robot.bug2Active = True

    headingTh = 0.08

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
                goalCanBeReached       = False
                robot.goalCanBeReached = goalCanBeReached
            else:
                # Get current mLine
                dist2GoalXY = goal.pos - robot.pos
                mLine     = np.atan2(dist2GoalXY[1], dist2GoalXY[0])
                mLine     = wrap_angle(mLine)

                if np.abs(wrap_angle(mLine - robot.mLineHeading)) <= headingTh:
                    if len(robot.hitPoints) > 3:
                        if dist(robot.pos, robot.hitPoints[0][0]) > 3.0:
                            if dist(robot.pos, goal.pos) < robot.dist2goalAtHitPoint:
                                mustFollowObstacle = False

            if mustFollowObstacle:
                robot.follow_obstacle_boundary(screen, goal.get_position(), collisionAngles, obstacleColor)
            else:
                _ = robot.move_toward_goal(screen, goal.get_position(), obstacleColor)
        else:
            _ = robot.move_toward_goal(screen, goal.get_position(), obstacleColor)

    return goalReached, goalCanBeReached




def tangentBug(screen, obstacleColor, robot, goal):
    pass
    