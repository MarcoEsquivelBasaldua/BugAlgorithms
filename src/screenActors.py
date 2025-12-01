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

    def addVertice(self, vertice):
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
    def __init__(self, color, rangeSensor = 0):
        """
        Initializes a robot with a position, existence flag, position history, radius, color, and range sensor.
        Arguments:
            color: A tuple representing the RGB color of the robot.
            rangeSensor: An integer representing the range of the robot's sensor (default is 0).
        Returns:
            None"""
        self.pos           = None
        self.exist         = False
        self.goalReached   = False
        self.heading       = 0.0
        self.__step        = 3
        self.__nearGoalTh  = 5
        self.__moving      = False
        self.__posHistory  = []
        self.__radius      = 10
        self.__color       = color
        self.__rangeSensor = max(self.__step, rangeSensor)
        self.__rangeSensor += (self.__radius)

        # Collision check flag
        samples            = 48
        angleRes           = twoPi / samples
        checkAngles        = np.array(list(range(samples))).astype(np.float64)
        self.__checkAngles = angleRes * checkAngles

    def moveTowardGoal(self, screen, goalPos):
        """
        Moves the robot toward the specified goal position in a straight line.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
            goalPos: A tuple representing the (x, y) coordinates of the goal position.
        Returns:
            None
        """
        self.__moving = True
        dist2Goal     = distance(self.pos, goalPos)

        if dist2Goal > self.__nearGoalTh:
            dist2GoalXY = goalPos - self.pos
            heading     = np.atan2(dist2GoalXY[1], dist2GoalXY[0])
            
            newPos = self.__moveOneStep(heading)

            self.pos += newPos

            self.__posHistory.append(np.array(self.pos, dtype=np.int64))
            self.__draw(screen)
        else:
            self.goalReached = True
            self.__moving    = False

    def followObstacleBoundary(self, screen, goalPos, collisionAngles):
        self.__moving = True
        dist2Goal     = distance(self.pos, goalPos)

        if dist2Goal > self.__nearGoalTh:
            collisionAnglesLen = len(collisionAngles)

            if collisionAnglesLen > 0:

                if collisionAnglesLen == 2:
                    normal2Obs = mean_angle(collisionAngles)
                    anglesDiff = angleDiff(collisionAngles[0], collisionAngles[1])
                else:
                    normal2Obs = collisionAngles[0]
                    anglesDiff = 0.0

                heading = normal2Obs + (0.5 * np.pi)
                heading = wrapAngle(heading)
            else:
                print('This should not happen')
                normal2Obs      = 0.0
                self.heading    = 0.0
                correctDistance = 0.0

            # Propose new position
            newPos = self.__moveOneStep(heading)
            pos    = self.pos + newPos

            # Check if new position is still in contact to obstacle
            collision, _ = self.checkCollision(screen, (0, 0, 0), True, pos)

            if collision: # Push robot away from obstacle
                # Get angles difference, map to a distance and pull robot away from obstacle
                correctDistance = linearRegression(0.0, np.pi, 0.0, 2, anglesDiff)
                normalFromObs   = normal2Obs + np.pi
                normalFromObs   = wrapAngle(normalFromObs)
                deltaXY         = correctDistance * np.array((np.cos(normalFromObs), np.sin(normalFromObs)))
                deltaXY         = np.round(deltaXY).astype(int)
                pos            += deltaXY

            else: # Pull robot toward the obstacle
                steps2Check = 20
                for step in range(1,steps2Check+1):
                    newPos    = step * np.array((np.cos(normal2Obs), np.sin(normal2Obs))).astype(float)
                    newPos    = np.round(newPos).astype(int)
                    pos2Check = pos + newPos

                    coll, _ = self.checkCollision(screen, (0,0,0), True, pos2Check)

                    if coll:
                        pos = pos2Check
                        break

            # Update robotPos
            self.pos = pos

            self.__posHistory.append(np.array(self.pos, dtype=np.int64))
            self.__draw(screen)

        else:
            self.goalReached = True
            self.__moving    = False


    def placeRobot(self, screen, button, toolbarWidth, wasMousePresed):
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
            self.__draw(screen)

    def drawHistory(self, screen):
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

    def reset(self):
        """
        Resets the robot's position and existence flag.
        Arguments:
            None
        Returns:
            None
        """
        self.pos          = None
        self.exist        = False
        self.goalReached  = False
        self.heading      = 0.0
        self.__moving     = False 
        self.__posHistory = []

    def checkCollision(self, screen, obstacleColor, localUse = False, pos = None):
        """
        Checks for collisions around the robot using its range sensor.
        Arguments:
            screen: The pygame surface where the robot is drawn.
            obstacleColor: A tuple representing the RGB color of the obstacles.
        Returns:
            A flag telling if the robot is in collision or not
            A list containing the first and last contact angles where a collision is detected.
        """
        collision           = False
        firstAndLastContact = []

        if localUse:
            usePos = pos
        else:
            usePos = self.pos

        for angle in self.__checkAngles:
            checkPos  = np.array((np.cos(angle), np.sin(angle)))
            checkPos *= self.__rangeSensor
            checkPos  =  np.round(checkPos).astype(np.int64)
            checkPos += usePos

            if screen.get_at(checkPos) == obstacleColor:
                collision = True
                if len(firstAndLastContact) < 2:
                    firstAndLastContact.append(angle)
                else:
                    firstAndLastContact[1] = angle

        return collision, firstAndLastContact


    def __draw(self, screen):
        """
        Draws the robot on the screen if it exists.
        Arguments:
            screen: The pygame surface where the robot will be drawn.
        Returns:
            None
        """
        if self.exist:
            pygame.draw.circle(screen, self.__color, self.pos, self.__radius)

    def __moveOneStep(self, heading):
        """
        Moves the robot one step in the specified heading direction.
        Arguments:
            heading: The heading angle in radians.
        Returns:
            A numpy array representing the change in position (x, y)."""
        heading      = wrapAngle(heading)
        self.heading = heading
        newPos       = self.__step * np.array((np.cos(heading), np.sin(heading))).astype(float)
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

    def placeGoal(self, screen, button, toolbarWidth, wasMousePresed):
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
        x1: A tuple representing the (x, y) coordinates of the first point.
        x2: A tuple representing the (x, y) coordinates of the second point.
    Returns:
        The Euclidean distance between the two points.
    """
    delta1 = x1[0] - x2[0]
    delta2 = x1[1] - x2[1]
    delta1 = delta1 ** 2
    delta2 = delta2 ** 2
    sum_ = delta1 + delta2

    return np.sqrt(sum_)

def drawObstacle(screen, obstacle, color, width):
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


def drawNewObstacle(screen, obstacleList, newObstacle, button, color, lineWidth, toolbarWidth, wasMousePresed):
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
            newObstacle.addVertice(newVertice)
            
            if newVertice == newObstacle.vertices[0] and len(newObstacle.vertices) > 1:
                obstacleList.append(copy.deepcopy(newObstacle))
                newObstacle.reset()
                button.reset()

        if len(newObstacle.vertices) > 1:
            drawObstacle(screen, newObstacle, color, lineWidth)

def wrapAngle(angleRadians):
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
    meanAngle = wrapAngle(meanAngle)

    return meanAngle

def angleDiff(angle1, angle2):
    """
    Calculates the smallest difference between two angles.
    Arguments:
        angle1: The first angle in radians.
        angle2: The second angle in radians.
    Returns:
        The smallest difference between the two angles in radians.
    """
    angle1 = wrapAngle(angle1)
    angle2 = wrapAngle(angle2)
    diff   = np.abs(angle1 - angle2)
    #diff = diff % twoPi
    return min(diff, twoPi - diff)
    

def linearRegression(xMin, xMax, yMin, yMax, value):
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

