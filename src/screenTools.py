import pygame

class Button:
    def __init__(self, x, y, width, height, text, color, pressedColor, wasPressed=False):
        """
        Initializes a button with position, size, text, colors, and pressed state.
        Arguments:
            x: The x-coordinate of the button's top-left corner.
            y: The y-coordinate of the button's top-left corner.
            width: The width of the button.
            height: The height of the button.
            text: The text displayed on the button.
            color: The normal color of the button.
            pressedColor: The color of the button when pressed.
            wasPressed: A boolean indicating if the button was pressed.
        Returns:
            None
        """
        self.wasPressed     = wasPressed
        self.__rect         = pygame.Rect(x, y, width, height)
        self.__text         = text
        self.__color        = color
        self.__pressedColor = pressedColor
        self.__currentColor = color
        self.__font         = pygame.font.SysFont("Arial", 20)

    def draw(self, surface):
        """
        Draws the button on the given surface.
        Arguments:
            surface: The pygame surface where the button will be drawn.
        Returns:
            None
        """
        pygame.draw.rect(surface, self.__currentColor, self.__rect)

        text_surface = self.__font.render(self.__text, True, (0, 0, 0)) # Black text
        text_rect    = text_surface.get_rect(center=self.__rect.center)

        surface.blit(text_surface, text_rect)

    def handle_event(self, event):
        """
        Handles mouse events to update the button's state.
        Arguments:
            event: The pygame event to handle.
        Returns:
            None
        """
        if self.__rect.collidepoint(event.pos):
            self.wasPressed     = True
            self.__currentColor = self.__pressedColor

    def reset(self):
        """
        Resets the button's pressed state and color.
        Arguments:
            None
        Returns:
            None
        """
        self.wasPressed     = False
        self.__currentColor = self.__color

    def was_button_pressed(self):
        """
        Returns whether the button was pressed.
        Arguments:
            None
        Returns:
            bool: True if the button was pressed, False otherwise.
        """
        return self.wasPressed



class Visualizer:
    def __init__(self, pos, fontSize, width, color):
        """
        Initializes a visualizer with position, font size, width, color, and fixed message.
        Arguments:
            pos: A tuple (x, y) representing the position of the visualizer.
            fontSize: The font size for the text.
            width: The width of the visualizer.
            color: The color of the visualizer background.
        Returns:
            None
        """
        self.pos    = pos
        self.color  = color
        self.height = fontSize + 10
        self.width  = width
        self.__rect = pygame.Rect(pos[0], pos[1], width, self.height)
        self.__font = pygame.font.SysFont("Arial", fontSize)

    def draw(self, screen, messageVar):
        """
        Draws the visualizer on the given screen with a variable message.
        Arguments:
            screen: The pygame surface where the visualizer will be drawn.
            message: A variable message string to display alongside the fixed message.
        Returns:
            None
        """
        text_surface = self.__font.render(messageVar, True, (0, 0, 0)) # Black text
        text_rect    = text_surface.get_rect(center=self.__rect.center)

        screen.blit(text_surface, text_rect)


#class SlideBar:
    