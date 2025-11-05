import pygame
import sys
sys.path.insert(1, './src')
import screenTools


if __name__ == "__main__":
    pygame.init()

    # Environment sizes
    ENV_WIDTH     = 1200
    ENV_LENGTH    = 800
    TOOLBAR_WIDTH = 200

    # Colors
    SCREEN_COLOR         = (255, 255, 255)  # White
    TOOLBAR_COLOR        = (160, 160, 160)  # Gray
    BUTTON_COLOR         = (192, 192, 192)
    BUTTON_PRESSED_COLOR = (128, 128, 128)
    ROBOT_COLOR          = (0, 0, 255)      # Blue
    GOAL_COLOR           = (255, 0, 0)      # Red
    OBSTACLE_COLOR       = (0, 0, 0)        # Black
    PATH_COLOR           = (153, 255, 255)  # Light blue
    MESSAGE_COLOR        = (0, 0, 0)        # Black
    WARNING_COLOR        = (255, 0, 0)      # Red

    screen = pygame.display.set_mode((ENV_WIDTH, ENV_LENGTH))
    pygame.display.set_caption("Bug Algorithms")

    # Buttons
    
    DRAW_OBSTACLE_BUTTON = screenTools.Button(10, 10, 100, 200, 'Draw obstacle', BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                DRAW_OBSTACLE_BUTTON.handle_event(event)

        screen.fill(SCREEN_COLOR)  # Fill the screen with white
        toolsBar = pygame.Rect(0, 0, TOOLBAR_WIDTH, ENV_LENGTH)
        pygame.draw.rect(screen, TOOLBAR_COLOR, toolsBar, width=0)

        DRAW_OBSTACLE_BUTTON.draw(screen)

        pygame.display.flip()    # Update the display

    pygame.quit()