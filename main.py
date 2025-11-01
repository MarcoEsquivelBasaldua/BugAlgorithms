import pygame

if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    pygame.display.set_caption("Bug Algorithms")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))  # Fill the screen with white
        toolsArea = pygame.Rect(0, 0, 200, 600)
        pygame.draw.rect(screen, (255, 0, 0), toolsArea, width=0)

        pygame.display.flip()    # Update the display

    pygame.quit()