import pygame

# Colors
SCREEN_COLOR   = (255, 255, 255)  # White
TOOLBAR_COLOR  = (160, 160, 160)  # Gray
ROBOT_COLOR    = (0, 0, 255)      # Blue
GOAL_COLOR     = (255, 0, 0)      # Red
OBSTACLE_COLOR = (0, 0, 0)        # Black
PATH_COLOR     = (153, 255, 255)  # Light blue
MESSAGE_COLOR  = (0, 0, 0)        # Black
WARNING_COLOR  = (255, 0, 0)      # Red

if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    pygame.display.set_caption("Bug Algorithms")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(SCREEN_COLOR)  # Fill the screen with white
        toolsBar = pygame.Rect(0, 0, 200, 600)
        pygame.draw.rect(screen, TOOLBAR_COLOR, toolsBar, width=0)

        pygame.display.flip()    # Update the display

    pygame.quit()