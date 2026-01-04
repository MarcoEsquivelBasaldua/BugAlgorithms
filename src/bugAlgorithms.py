import numpy as np
import sys
sys.path.insert(1, './src')
from screenActors import distance as dist
from screenActors import wrap_angle
from screenActors import angle_diff

def bug1(robot, goal):
    """
    Implements the Bug 1 algorithm for robot navigation.
    Args:
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
        collision, collisionAngles = robot.check_collision()

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
                robot.follow_obstacle_boundary(goal.get_position(), collisionAngles)
            else:
                goalCanBeReached       = not(robot.move_toward_goal(goal.get_position()))
                robot.goalCanBeReached = goalCanBeReached
        else:
            _ = robot.move_toward_goal(goal.get_position())

    return goalReached, goalCanBeReached


def bug2(robot, goal):
    """
    Implements the Bug 2 algorithm for robot navigation.
    Args:
        robot: The robot object that will navigate the environment.
        goal: The goal object representing the target position.
    Returns:
        goalReached: A boolean indicating if the robot has reached the goal.
        goalCanBeReached: A boolean indicating if the goal can be reached.
    """
    headingTh = 0.087  # ~5 degrees

    goalReached      = robot.is_goal_reached(goal.get_position())
    goalCanBeReached = robot.goalCanBeReached

    if not goalReached and goalCanBeReached:
        # Check collision
        collision, collisionAngles = robot.check_collision()

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
                robot.follow_obstacle_boundary(goal.get_position(), collisionAngles)
            else:
                _ = robot.move_toward_goal(goal.get_position())
        else:
            _ = robot.move_toward_goal(goal.get_position())

    return goalReached, goalCanBeReached



def tangentBug(robot, goal):
    """
    Implements the Tangent Bug algorithm for robot navigation.
    Args:
        robot: The robot object that will navigate the environment.
        goal: The goal object representing the target position.
    Returns:
        goalReached: A boolean indicating if the robot has reached the goal.
        goalCanBeReached: A boolean indicating if the goal can be reached.
    """
    goalReached      = robot.is_goal_reached(goal.get_position())
    goalCanBeReached = robot.goalCanBeReached

    if not goalReached and goalCanBeReached:
        # Find and show discontinuity points
        discPoints = robot.get_discontinuities()

        # Check collision
        collision, collisionAngles = robot.check_collision()

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
                # Check if goal is seen
                if not robot.is_obstacle_in_path_to_goal(goal.get_position()):
                    # Check heading to goal
                    dist2GoalXY   = goal.pos - robot.pos
                    headingToGoal = np.atan2(dist2GoalXY[1], dist2GoalXY[0])
                    headingToGoal = wrap_angle(headingToGoal)
                    newPos        = robot.pos + robot.stepSize * np.array([np.cos(headingToGoal), np.sin(headingToGoal)])
                    newPos        = np.round(newPos).astype(int)

                    # Check if moving to new position collides with obstacle
                    collisionAtNewPos, _ = robot.check_collision(True, newPos)
                    if not collisionAtNewPos:
                        mustFollowObstacle = False

            if mustFollowObstacle:
                robot.follow_obstacle_boundary(goal.get_position(), collisionAngles)
            else:
                _ = robot.move_toward_goal(goal.get_position())
        elif len(discPoints) > 0:
            if robot.is_obstacle_in_path_to_goal(goal.get_position()):
                # Chose a discontinuity point to follow
                followPoint = discPoints[0][0]
                minDist     = dist(robot.pos, followPoint) + dist(followPoint, goal.get_position())

                for discPoint in discPoints[1:]:
                    point      = discPoint[0]
                    pointAngle = discPoint[1]
                    currDist   = dist(robot.pos, point) + dist(point, goal.get_position())

                    if currDist < minDist and angle_diff(pointAngle, robot.heading) < np.deg2rad(15.0):
                        minDist     = currDist
                        followPoint = point

                _ = robot.move_toward_goal(followPoint)
            else:
                _ = robot.move_toward_goal(goal.get_position())
        else:
            _ = robot.move_toward_goal(goal.get_position())
            
    return goalReached, goalCanBeReached
    