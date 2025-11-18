import pygame

class Obstacle:
    def __init__(self):
        self.vertices   = []

    def addVertice(self, vertice):
        self.vertices.append(vertice)

    def reset(self):
        self.vertices = []


class Robot:
    def __init__(self, pos, color, rangeSensor = 0):
        self.pos           = pos
        self.exist         = False
        self.__posHistory  = []
        self.__radius      = 10
        self.__color       = color
        self.__rangeSensor = rangeSensor

        self.__posHistory.append(self.pos)

    def __step(self):
        pass

    def __obstacleEncountered(self):
        pass

    def moveTowardGoal(self):
        pass

    def followObstacleBoundary(self):
        pass

    def draw(self, screen, pos = None):
        if self.exist:
            if pos == None:
                pos = self.pos

            #self.__posHistory.append(self.pos)
            pygame.draw.circle(screen, self.__color, pos, self.__radius)

    def drawHistory(self, screen):
        nOfSteps = len(self.__posHistory)

        for i, pos in enumerate(self.__posHistory, start=1):
            alphaColor = (i / nOfSteps) * 255
            alphaColor = int(alphaColor)
            newColor   = (255, alphaColor, alphaColor)

            pygame.draw.circle(screen, newColor, pos, self.__radius // 2)

    def reset(self):
        pass





#class Goal: