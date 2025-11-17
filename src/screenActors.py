import pygame

class Obstacle:
    def __init__(self):
        self.vertices = []
        self.isNearInit = False

    def addVertice(self, vertice):
        self.vertices.append(vertice)

    def reset(self):
        self.vertices = []


#class Robot:



#class Goal: