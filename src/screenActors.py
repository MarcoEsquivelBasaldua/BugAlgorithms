import pygame
import copy
import numpy as np

class Obstacle:
    def __init__(self):
        self.vertices   = []

    def addVertice(self, vertice):
        self.vertices.append(vertice)

    def reset(self):
        self.vertices = []


class Robot:
    def __init__(self, color, rangeSensor = 0):
        self.pos           = None
        self.exist         = False
        self.__posHistory  = []
        self.__radius      = 10
        self.__color       = color
        self.__rangeSensor = rangeSensor

        self.__posHistory.append(self.pos)

    def __add2History(self):
        pass

    def __obstacleEncountered(self):
        pass

    def moveTowardGoal(self):
        pass

    def followObstacleBoundary(self):
        pass

    def __draw(self, screen):
        if self.exist:
            pygame.draw.circle(screen, self.__color, self.pos, self.__radius)

    def placeRobot(self, screen, button, color, toolbarWidth, wasMousePresed):
        if button.wasPressed and pygame.mouse.get_pos()[0] > toolbarWidth:
            currentPos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, color, currentPos, self.__radius)

            if wasMousePresed:
                self.pos   = currentPos
                self.exist = True
                button.reset()

        self.__draw(screen)

    def drawHistory(self, screen):
        nOfSteps = len(self.__posHistory)

        for i, pos in enumerate(self.__posHistory, start=1):
            alphaColor = (i / nOfSteps) * 255
            alphaColor = int(alphaColor)
            newColor   = (255, alphaColor, alphaColor)

            pygame.draw.circle(screen, newColor, pos, self.__radius // 2)

    def reset(self):
        pass



class Goal:
    def __init__(self, color):
        self.pos = None

    #def placeGoal()

# Functions

def distance(x1, x2):
    delta1 = x1[0] - x2[0]
    delta2 = x1[1] - x2[1]
    delta1 = delta1 ** 2
    delta2 = delta2 ** 2
    sum_ = delta1 + delta2

    return np.sqrt(sum_)

def drawObstacle(screen, obstacle, color, width):
    nOfVertices = len(obstacle.vertices)
    initPos = obstacle.vertices[0]
    lastVertice = initPos

    for i in range(nOfVertices):
        pygame.draw.circle(screen, color, obstacle.vertices[i], width // 2)
        if i < (nOfVertices - 1):
            lastVertice = obstacle.vertices[i+1]
            
        pygame.draw.line(screen, color, obstacle.vertices[i], lastVertice, width)


def drawNewObstacle(screen, obstacleList, newObstacle, button, color, lineWidth, toolbarWidth, wasMousePresed):
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