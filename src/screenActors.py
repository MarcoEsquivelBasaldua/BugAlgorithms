import pygame
import copy
import numpy as np

twoPi = 2.0 * np.pi

class Obstacle:
    def __init__(self):
        """
        Initializes an obstacle with an empty list of vertices.
        Arguments:
            None
        Returns:
            None
        """
        self.vertices   = []

    def add_vertice(self, vertice):
        """
        Adds a vertice to the obstacle's vertices list.
        Arguments:
            vertice: A tuple representing the (x, y) coordinates of the vertice to be added.
        Returns:
            None
        """
        self.vertices.append(vertice)

    def reset(self):
        """
        Resets the obstacle by clearing its vertices list.
        Arguments:
            None
        Returns:
            None
        """
        self.vertices = []


class Robot:
    def __init__(self, color):
        """
        Initializes a robot with a position, existence flag, position history, radius, color, and range sensor.
        Arguments:
            color: A tuple representing the RGB color of the robot.
            rangeSensor: An integer representing the range of the robot's sensor (default is 0).
        Returns:
            None"""
        self.pos              = None
        self.exist            = False
        self.goalReached      = False
        self.goalCanBeReached = True
        self.heading          = 0.0
        self.stepSize         = 3
        self.__nearGoalTh     = 5
        self.__moving         = False
        self.__posHistory     = []
        self.__radius         = 10
        self.__color          = color
        self.__moving2Goal    = False
        self.__followingObs   = False
        self.__normalSign     = 1.0

        # Collision check flag
        samples            = 12
        angleRes           = twoPi / samples
        checkAngles        = np.array(list(range(samples))).astype(np.float64)
        self.__checkAngles = angleRes * checkAngles

        # Bug 1 and Bug 2 specific variables
        self.hitPoints    = []
        self.minHitPoints = 38
        self.obsEncircled = False

        # Bug 1 specific variables
        self.bug1Active   = False
        self.minDist2Goal = np.inf

        # Bug 2 specific variables
        self.bug2Active          = False
        self.hitpoint            = None
        self.mLineHeading        = 0.0
        self.hitObstacle         = False
        self.prevDist2Goal       = np.inf
        self.dist2goalAtHitPoint = np.inf

        # Tangent Bug specific variables
        self.TBugActive = False
        self.discontinuityPoints = []
        samples            = 120
        angleRes           = twoPi / samples
        checkAngles        = np.array(list(range(samples))).astype(np.float64)
        self.__theta       = angleRes * checkAngles
        self.rangeSensor   = 0
        

    def move_toward_goal(self, screen, goalPos, obstacleColor):
        """
        Moves the robot toward the specified goal position in a straight line.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
            goalPos: A tuple representing the (x, y) coordinates of the goal position.
        Returns:
            A flag telling if the robot is in collision or not
        """
        if isinstance(goalPos, tuple):
            goalPos = np.array(goalPos).astype(int)

        # Reset hit points list
        self.hitPoints    = []
        self.obsEncircled = False
        
        if self.bug1Active:
            self.minDist2Goal = np.inf
        elif self.bug2Active:
            self.hitObstacle         = False
            self.dist2goalAtHitPoint = np.inf

        # Get new robot position
        dist2GoalXY = goalPos - self.pos
        heading     = np.atan2(dist2GoalXY[1], dist2GoalXY[0])
        heading     = wrap_angle(heading)
        
        newPos    = self.__move_oneStep(heading)
        self.pos += newPos

        # Check collision on new proposed position
        collision, _ = self.check_collision(screen, obstacleColor, True, self.pos)

        self.__posHistory.append(np.array(self.pos, dtype=np.int64))
        self.draw(screen)

        if self.bug2Active:
            self.mLineHeading = heading

        # Update robot motion state
        self.__moving2Goal  = True
        self.__followingObs = False

        return collision

    def follow_obstacle_boundary(self, screen, goalPos, collisionAngles, obstacleColor):
        """
        Moves the robot along the boundary of an obstacle based on collision angles.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
            goalPos: A tuple representing the (x, y) coordinates of the goal position.
            collisionAngles: A list of angles where the robot is in contact with the obstacle.
            obstacleColor: A tuple representing the RGB color of the obstacles.
        Returns:
            None
        """
        # Save hit point
        dist2Goal = distance(self.pos, goalPos)
        self.hitPoints.append((self.pos, dist2Goal))

        if self.bug1Active:
            self.minDist2Goal = min(self.minDist2Goal, dist2Goal)
        elif self.bug2Active:
            if not self.hitObstacle:
                self.hitObstacle         = True
                self.hitpoint            = self.pos
                self.dist2goalAtHitPoint = distance(self.pos, goalPos)

        collisionAnglesLen = len(collisionAngles)
        if collisionAnglesLen > 0:

            if collisionAnglesLen == 2:
                normal2Obs = mean_angle(collisionAngles)
                anglesDiff = angle_diff(collisionAngles[0], collisionAngles[1])
            else:
                normal2Obs = collisionAngles[0]
                anglesDiff = 0.0

            normal2Obs = wrap_angle(normal2Obs)

            if self.__moving2Goal:
                heading1 = normal2Obs + (0.5 * np.pi)
                heading = wrap_angle(heading1)

                heading2 = normal2Obs - (0.5 * np.pi)
                heading = wrap_angle(heading2)

                if angle_diff(self.heading, heading1) > angle_diff(self.heading, heading2):
                    self.__normalSign = -1.0
                else:
                    self.__normalSign = 1.0

            heading = normal2Obs + (self.__normalSign * (0.5 * np.pi))
            #heading = normal2Obs + (0.5 * np.pi)
            heading = wrap_angle(heading)
        else:
            print('This should not happen')
            heading         = self.heading
            normal2Obs      = heading - (0.5 * np.pi)
            correctDistance = 0.0

        # Propose new position
        newPos = self.__move_oneStep(heading)
        pos    = self.pos + newPos

        # Check if new position is still in contact to obstacle
        collision, _ = self.check_collision(screen, obstacleColor, True, pos)

        if collision: # Push robot away from obstacle
            # Get angles difference, map to a distance and pull robot away from obstacle
            correctDistance = linear_regression(0.0, np.pi, 0.0, 5, anglesDiff)
            normalFromObs   = normal2Obs + np.pi
            normalFromObs   = wrap_angle(normalFromObs)
            deltaXY         = correctDistance * np.array((np.cos(normalFromObs), np.sin(normalFromObs)))
            deltaXY         = np.round(deltaXY).astype(int)
            pos            += deltaXY

        # Check if new position is still in contact to obstacle
        collision, _ = self.check_collision(screen, obstacleColor, True, pos)

        if not collision: # Pull robot toward the obstacle
            steps2Check = 20
            for step in range(1,steps2Check+1):
                newPos    = step * np.array((np.cos(normal2Obs), np.sin(normal2Obs))).astype(float)
                newPos    = np.round(newPos).astype(int)
                pos2Check = pos + newPos

                coll, _ = self.check_collision(screen, obstacleColor, True, pos2Check)

                if coll:
                    pos = pos2Check
                    break

        # Update robotPos
        self.pos = pos

        # Save robot position into history
        self.__posHistory.append(np.array(self.pos, dtype=np.int64))
        self.draw(screen)

        # Update robot motion state
        self.__moving2Goal  = False
        self.__followingObs = True


    def place_robot(self, screen, button, toolbarWidth, wasMousePresed):
        """
        Places the robot on the screen when the specified button is pressed and the mouse is clicked.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
            button: The button that triggers the placement of the robot.
            toolbarWidth: The width of the toolbar to ensure the robot is placed outside of it.
            wasMousePresed: A boolean indicating if the mouse was pressed.
        Returns:
            None
        """
        if button.wasPressed and pygame.mouse.get_pos()[0] > toolbarWidth:
            currentPos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, self.__color, currentPos, self.__radius)

            if wasMousePresed:
                self.pos   = np.array(currentPos, dtype=np.int64)
                self.exist = True
                button.reset()
        if not self.__moving:
            self.draw(screen)

    def draw_history(self, screen):
        """
        Draws the robot's position history on the screen with a fading effect.
        Arguments:
            screen: The pygame surface where the history will be drawn.
        Returns:
            None
        """
        nOfSteps = len(self.__posHistory)

        for i, pos in enumerate(self.__posHistory):
            alphaColor = (i / nOfSteps) * 255
            alphaColor = 255 - int(alphaColor)
            newColor   = (alphaColor, alphaColor, 255)

            pygame.draw.circle(screen, newColor, pos, self.__radius // 3)

    def is_goal_reached(self, goalPos):
        """
        Checks if the robot has reached the specified goal position.
        Arguments:
            goalPos: A tuple representing the (x, y) coordinates of the goal position.
        Returns:
            A boolean indicating whether the goal has been reached.
        """
        dist2Goal = distance(self.pos, goalPos)

        if dist2Goal > self.__nearGoalTh:
            self.goalReached = False
            self.__moving    = True
        else:
            self.goalReached = True
            self.__moving    = False

        return self.goalReached

    def reset(self):
        """
        Resets the robot's position and existence flag.
        Arguments:
            None
        Returns:
            None
        """
        self.pos              = None
        self.exist            = False
        self.goalReached      = False
        self.goalCanBeReached = True
        self.heading          = 0.0
        self.__moving         = False
        self.__posHistory     = []
        self.__moving2Goal    = False
        self.__followingObs   = False
        
        # Bug 1 and Bug 2 specific variables
        self.hitPoints        = []
        self.obsEncircled     = False

        # Bug 1 specific variables
        self.bug1Active   = False
        self.minDist2Goal = np.inf

        # Bug 2 specific variables
        self.bug2Active          = False
        self.mLineHeading        = 0.0
        self.hitObstacle         = False
        self.dist2goalAtHitPoint = np.inf
        self.prevDist2Goal       = np.inf

        # Tangent Bug specific variables
        self.TBugActive = False
        self.descontinuityPoints = []

    def update_range_sensor(self, rangeSensor):
        """
        Updates the range sensor distance of the robot.
        Arguments:
            rangeSensor: An integer representing the new range sensor distance.
        Returns:
            None
        """
        self.rangeSensor  = max(self.stepSize, rangeSensor)
        self.rangeSensor += (self.__radius)

        

    def check_collision(self, screen, obstacleColor, localUse = False, pos = None):
        """
        Checks for collisions around the robot using its range sensor.
        Arguments:
            screen: The pygame surface where the robot is drawn.
            obstacleColor: A tuple representing the RGB color of the obstacles.
        Returns:
            A flag telling if the robot is in collision or not
            A list containing the first and last contact angles where a collision is detected.
        """
        rangeSensor = self.stepSize + self.__radius

        collision           = False
        firstAndLastContact = []

        if localUse:
            usePos = pos
        else:
            usePos = self.pos

        for angle in self.__checkAngles:
            checkPos  = np.array((np.cos(angle), np.sin(angle)))
            checkPos *= rangeSensor
            checkPos  =  np.round(checkPos).astype(np.int64)
            checkPos += usePos

            if screen.get_at(checkPos) == obstacleColor:
                collision = True
                if len(firstAndLastContact) < 2:
                    firstAndLastContact.append(angle)
                else:
                    firstAndLastContact[1] = angle

        return collision, firstAndLastContact
    

    def get_discontinuities(self, screen, obstacleColor):
        """
        Gets the discontinuity points detected by the robot's range sensor.
        Arguments:
            screen: The pygame surface where the robot is drawn.
            obstacleColor: A tuple representing the RGB color of the obstacles.
        Returns:
            A list of discontinuity points detected by the robot's range sensor.
        """
        discPoints = []

        # Define steps along each theta angle
        samples    = 25
        stepRes    = self.rangeSensor / samples
        checkSteps = np.array(list(range(samples))).astype(np.float64)
        steps      = stepRes * checkSteps

        # Check if first theta is facing obstacle
        theta      = self.__theta[0]
        prevTheta  = theta
        prevPos    = None
        prevRadius = None

        wasObstacle = False
        for step in steps:
            checkPos   = np.array((np.cos(theta), np.sin(theta)))
            checkPos  *= step
            checkPos   =  np.round(checkPos).astype(np.int64)
            checkPos  += self.pos
            prevRadius = distance(self.pos, checkPos)

            if checkPos[0] < 200 or checkPos[1] < 0:
                prevPos = checkPos + np.array((1, 1)).astype(int)
                break
            if checkPos[0] >= screen.get_width() or checkPos[1] >= screen.get_height():
                prevPos = checkPos - np.array((2, 2)).astype(int)
                break

            if screen.get_at(checkPos) == obstacleColor:
                wasObstacle = True
                prevPos     = checkPos
                break

            prevPos = checkPos

        nOfSteps = len(steps)
        for theta in self.__theta[1:]:
            for i, step in enumerate(steps):
                checkPos  = np.array((np.cos(theta), np.sin(theta)))
                checkPos *= step
                checkPos  =  np.round(checkPos).astype(np.int64)
                checkPos += self.pos
                radius    = distance(self.pos, checkPos)

                if checkPos[0] < 200 or checkPos[1] < 0:
                    prevPos = checkPos + np.array((1, 1)).astype(int)
                    break
                if checkPos[0] >= screen.get_width() or checkPos[1] >= screen.get_height():
                    prevPos = checkPos - np.array((2, 2)).astype(int)
                    break

                if screen.get_at(checkPos) == obstacleColor:
                    if not wasObstacle:
                        discPoints.append([checkPos, theta])
                    elif np.abs(prevRadius - radius) > 23:
                        discPoints.append([prevPos, prevTheta])
                        discPoints.append([checkPos, theta])

                    wasObstacle = True
                    prevPos   = checkPos
                    prevTheta = theta
                    prevRadius = radius
                    break
                if i == (nOfSteps-1):
                    if (screen.get_at(checkPos) != obstacleColor) and (wasObstacle):
                        discPoints.append([prevPos, prevTheta])
                        wasObstacle = False
                    prevPos   = checkPos
                    prevTheta = theta
            prevRadius = radius

        self.discontinuityPoints = discPoints
        return discPoints
    
    def draw_discontinuity_points(self, screen, color):
        """
        Draws the discontinuity points on the screen.
        Arguments:
            screen: The pygame surface where the discontinuity points will be drawn.
            color: A tuple representing the RGB color of the discontinuity points.
        Returns:
            None
        """
        for dPoint in self.discontinuityPoints:
            pygame.draw.circle(screen, color, dPoint[0], 5)


    def is_obstacle_in_path_to_goal(self, screen, goalPos, obstacleColor):
        """
        Checks if an obstacle is in the path from the robot to the goal.
        Arguments:
            screen: The pygame surface where obstacles are checked.
            goalPos: A numpy array representing the position of the goal.
            obstacleColor: A tuple representing the RGB color of obstacles.
        Returns:
            A boolean indicating whether an obstacle is in the path to the goal.
        """
        # Define steps along each theta angle
        isObstacle = False
        samples    = 25
        stepRes    = self.rangeSensor / samples
        checkSteps = np.array(list(range(samples))).astype(np.float64)
        steps      = stepRes * checkSteps

        dist2GoalXY = goalPos - self.pos
        heading     = np.atan2(dist2GoalXY[1], dist2GoalXY[0])
        heading     = wrap_angle(heading)

        for step in steps:
            checkPos   = np.array((np.cos(heading), np.sin(heading)))
            checkPos  *= step
            checkPos   =  np.round(checkPos).astype(np.int64)
            checkPos  += self.pos

            if screen.get_at(checkPos) == obstacleColor:
                isObstacle = True
                break

        return isObstacle


    def draw(self, screen):
        """
        Draws the robot on the screen if it exists.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
        Returns:
            None
        """
        if self.exist:
            pygame.draw.circle(screen, self.__color, self.pos, self.__radius)

            if self.TBugActive:
                pygame.draw.circle(screen, self.__color, self.pos, self.rangeSensor, width=1)


    def __move_oneStep(self, heading):
        """
        Moves the robot one step in the specified heading direction.
        Arguments:
            heading: The heading angle in radians.
        Returns:
            A numpy array representing the change in position (x, y)."""
        heading      = wrap_angle(heading)
        self.heading = heading
        newPos       = self.stepSize * np.array((np.cos(heading), np.sin(heading))).astype(float)
        newPos       = np.round(newPos).astype(int)

        return newPos



class Goal:
    def __init__(self, color):
        """
        Initializes a goal with a position, existence flag, color, and radius.
        Arguments:
            color: A tuple representing the RGB color of the goal.
        Returns:
            None
        """
        self.pos      = None
        self.exist    = False
        self.__color  = color
        self.__radius = 20

    def place_goal(self, screen, button, toolbarWidth, wasMousePresed):
        """
        Places the goal on the screen when the specified button is pressed and the mouse is clicked.
        Arguments:
            screen: The pygame surface where the goal will be drawn.
            button: The button that triggers the placement of the goal.
            toolbarWidth: The width of the toolbar to ensure the goal is placed outside of it.
            wasMousePresed: A boolean indicating if the mouse was pressed.
        Returns:
            None
        """
        if button.wasPressed and pygame.mouse.get_pos()[0] > toolbarWidth:
            currentPos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, self.__color, currentPos, self.__radius)

            if wasMousePresed:
                self.pos   = np.array(currentPos, dtype=np.int64)
                self.exist = True
                button.reset()

        self.__draw(screen)

    def reset(self):
        """
        Resets the goal's position and existence flag.
        Arguments:
            None
        Returns:
            None
        """
        self.pos   = None
        self.exist = False

    def get_position(self):
        """
        Returns the position of the goal.
        Arguments:
            None
        Returns:
            A tuple representing the (x, y) coordinates of the goal's position.
        """
        return self.pos

    def __draw(self, screen):
        """
        Draws the goal on the screen if it exists.
        Arguments:
            screen: The pygame surface where the goal will be drawn.
        Returns:
            None
        """
        if self.exist:
            pygame.draw.circle(screen, self.__color, self.pos, self.__radius)


# Functions

def distance(x1, x2):
    """
    Calculates the Euclidean distance between two points.
    Arguments:
        x1: A tuple or array representing the (x, y) coordinates of the first point.
        x2: A tuple or array representing the (x, y) coordinates of the second point.
    Returns:
        The Euclidean distance between the two points.
    """
    if isinstance(x1, tuple):
        x1 = np.array(x1).astype(int)
    if isinstance(x2, tuple):
        x2 = np.array(x2).astype(int)

    dist = np.linalg.norm(x1 - x2)
    
    return dist

def draw_obstacle(screen, obstacle, color, width):
    """
    Draws an obstacle on the screen.
    Arguments:
        screen: The pygame surface where the obstacle will be drawn.
        obstacle: An Obstacle object containing the vertices of the obstacle.
        color: A tuple representing the RGB color of the obstacle.
        width: An integer representing the width of the obstacle lines.
    Returns:
        None
    """
    nOfVertices = len(obstacle.vertices)
    initPos = obstacle.vertices[0]
    lastVertice = initPos

    for i in range(nOfVertices):
        pygame.draw.circle(screen, color, obstacle.vertices[i], int(0.4 * width))
        if i < (nOfVertices - 1):
            lastVertice = obstacle.vertices[i+1]
            
        pygame.draw.line(screen, color, obstacle.vertices[i], lastVertice, width)


def draw_new_obstacle(screen, obstacleList, newObstacle, button, color, lineWidth, toolbarWidth, wasMousePresed):
    """
    Draws a new obstacle on the screen while the user is defining its vertices.
    Arguments:
        screen: The pygame surface where the obstacle will be drawn.
        obstacleList: A list to store completed obstacles.
        newObstacle: An Obstacle object representing the new obstacle being defined.
        button: The button that triggers the drawing of the new obstacle.
        color: A tuple representing the RGB color of the obstacle.
        lineWidth: An integer representing the width of the obstacle lines.
        toolbarWidth: The width of the toolbar to ensure the obstacle is drawn outside of it.
        wasMousePresed: A boolean indicating if the mouse was pressed.
    Returns:
        None
    """
    if button.wasPressed and pygame.mouse.get_pos()[0] > toolbarWidth:
        closePolygonDistance = 10.0

        currentMousePos = pygame.mouse.get_pos()
        newVertice = currentMousePos

        if len(newObstacle.vertices) > 0:
            dist2FirstVertice = distance(currentMousePos, newObstacle.vertices[0])

            if dist2FirstVertice < closePolygonDistance:
                newVertice = newObstacle.vertices[0]
            else:
                newVertice = currentMousePos

            pygame.draw.line(screen, color, newObstacle.vertices[-1], newVertice, lineWidth)
        pygame.draw.circle(screen, color, newVertice, int(0.4 * lineWidth))

        if wasMousePresed:
            newObstacle.add_vertice(newVertice)
            
            if newVertice == newObstacle.vertices[0] and len(newObstacle.vertices) > 1:
                obstacleList.append(copy.deepcopy(newObstacle))
                newObstacle.reset()
                button.reset()

        if len(newObstacle.vertices) > 1:
            draw_obstacle(screen, newObstacle, color, lineWidth)

def wrap_angle(angleRadians):
    """
    Wraps an angle in radians to the range [0, 2π].
    Arguments:
        angleRadians: The angle in radians to be wrapped.
    Returns:
        The wrapped angle in radians.
    """
    return angleRadians % twoPi

def mean_angle(angles):
    """
    Calculates the mean of a list of angles in radians.
    Arguments:
        angles: A list of angles in radians.
    Returns:
        The mean angle in radians.
    """
    angle1 = angles[0]
    angle2 = angles[1]

    # Convert angles to 2D vectors
    x1, y1 = np.cos(angle1), np.sin(angle1)
    x2, y2 = np.cos(angle2), np.sin(angle2)

    # Sum the vectors
    xSum = x1 + x2
    ySum = y1 + y2

    # Calculate the angle of the resulting vector
    meanAngle = np.atan2(ySum, xSum)
    meanAngle = wrap_angle(meanAngle)

    return meanAngle

def angle_diff(angle1, angle2):
    """
    Calculates the smallest difference between two angles.
    Arguments:
        angle1: The first angle in radians.
        angle2: The second angle in radians.
    Returns:
        The smallest difference between the two angles in radians.
    """
    angle1 = wrap_angle(angle1)
    angle2 = wrap_angle(angle2)
    diff   = np.abs(angle1 - angle2)
    return min(diff, twoPi - diff)
    

def linear_regression(xMin, xMax, yMin, yMax, value):
    """
    Performs linear regression to map a value from one range to another.
    Arguments:
        xMin: The minimum value of the input range.
        xMax: The maximum value of the input range.
        yMin: The minimum value of the output range.
        yMax: The maximum value of the output range.
        value: The input value to be mapped.
    Returns:
        The mapped value in the output range.
    """
    if value >= xMax:
        return yMax
    elif value <= xMin:
        return yMin
    
    deltaX = (xMax - xMin)
    deltaY = (yMax - yMin)
    m      = deltaY / deltaX

    return (m * (value - xMin) + yMin)

