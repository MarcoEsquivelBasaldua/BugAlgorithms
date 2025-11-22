import pygame
import copy
import numpy as np

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
        self.__step        = 10
        self.__nearGoalTh  = 10
        self.__moving      = False
        self.__posHistory  = []
        self.__radius      = 10
        self.__color       = color
        self.__rangeSensor = rangeSensor

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
            dist2GoalXY  = goalPos - self.pos
            steps2goal = int(dist2Goal / self.__step)

            steps = (dist2GoalXY / steps2goal).astype(int)

            self.pos += steps

            self.__posHistory.append(np.array(self.pos, dtype=np.int64))
            self.__draw(screen)
        else:
            self.goalReached = True
            self.__moving    = False

    def followObstacleBoundary(self):
        pass

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

            pygame.draw.circle(screen, newColor, pos, self.__radius // 2)

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
        self.__moving     = False 
        self.__posHistory = []

    def __draw(self, screen):
        if self.exist:
            pygame.draw.circle(screen, self.__color, self.pos, self.__radius)

    def __add2History(self):
        pass

    def __obstacleEncountered(self):
        pass


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
        pygame.draw.circle(screen, color, obstacle.vertices[i], width // 2)
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
        pygame.draw.circle(screen, color, newVertice, lineWidth // 2)

        if wasMousePresed:
            newObstacle.addVertice(newVertice)
            
            if newVertice == newObstacle.vertices[0] and len(newObstacle.vertices) > 1:
                obstacleList.append(copy.deepcopy(newObstacle))
                newObstacle.reset()
                button.reset()

        if len(newObstacle.vertices) > 1:
            drawObstacle(screen, newObstacle, color, lineWidth)
