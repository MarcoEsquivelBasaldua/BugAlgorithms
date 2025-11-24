import pygame
import sys
sys.path.insert(1, './src')
import screenTools
import screenActors

if __name__ == "__main__":
    pygame.init()

    # Environment sizes
    ENV_HEIGHT    = 900
    ENV_WIDTH     = 1400
    TOOLBAR_WIDTH = 200

    # Colors
    SCREEN_COLOR         = (255, 255, 255)  # White
    TOOLBAR_COLOR        = (160, 160, 160)  # Gray
    BUTTON_COLOR         = (192, 192, 192)  # Light gray
    BUTTON_PRESSED_COLOR = (128, 128, 128)  # Dark gray
    ROBOT_COLOR          = (0, 0, 255)      # Blue
    GOAL_COLOR           = (255, 0, 0)      # Red
    OBSTACLE_COLOR       = (0, 0, 0)        # Black
    PATH_COLOR           = (153, 255, 255)  # Light blue
    MESSAGE_COLOR        = (0, 0, 0)        # Black
    WARNING_COLOR        = (255, 0, 0)      # Red

    screen = pygame.display.set_mode((ENV_WIDTH, ENV_HEIGHT))
    pygame.display.set_caption("Bug Algorithms")

    # Buttons
    BUTTON_HEIGHT_BIG    = 80
    BUTTON_WIDTH_BIG     = 140
    BUTTON_HEIGHT_SMALL  = 60
    BUTTON_WIDTH_SMALL   = 76

    DRAW_OBSTACLE_BUTTON = screenTools.Button(30 , 40 , BUTTON_WIDTH_BIG  , BUTTON_HEIGHT_BIG  , 'Draw Obstacle', BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    PLACE_GOAL_BUTTON    = screenTools.Button(30 , 160, BUTTON_WIDTH_BIG  , BUTTON_HEIGHT_BIG  , 'Place Goal'   , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    PLACE_ROBOT_BUTTON   = screenTools.Button(30 , 280, BUTTON_WIDTH_BIG  , BUTTON_HEIGHT_BIG  , 'Place Robot'  , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    BUG1_BUTTON          = screenTools.Button(16 , 400, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'BUG 1'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    BUG2_BUTTON          = screenTools.Button(108, 400, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'BUG 2'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    TANGENT_BUG_BUTTON   = screenTools.Button(62 , 480, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'T BUG'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    RESET_PLACES_BUTTON  = screenTools.Button(16 , 620, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'RESET'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    RESET_ALL_BUTTON     = screenTools.Button(108, 620, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'EMPTY'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)
    GO_BUTTON            = screenTools.Button(30 , 700, BUTTON_WIDTH_BIG  , BUTTON_HEIGHT_BIG  , 'GO!!!'        , BUTTON_COLOR, BUTTON_PRESSED_COLOR)

    # Obstacles list
    OBSTACLE_WIDTH = 20
    newObs = screenActors.Obstacle()
    wasMousePresed = False
    obstacles = []

    # Point Robot
    robot = screenActors.Robot(ROBOT_COLOR)

    # Goal
    goal = screenActors.Goal(GOAL_COLOR)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                wasMousePresed = True

                DRAW_OBSTACLE_BUTTON.handle_event(event)
                PLACE_GOAL_BUTTON.handle_event(event)
                PLACE_ROBOT_BUTTON.handle_event(event)
                BUG1_BUTTON.handle_event(event)
                BUG2_BUTTON.handle_event(event)
                TANGENT_BUG_BUTTON.handle_event(event)
                GO_BUTTON.handle_event(event)
                RESET_PLACES_BUTTON.handle_event(event)
                RESET_ALL_BUTTON.handle_event(event)

        screen.fill(SCREEN_COLOR)  # Fill the screen with white
        toolsBar = pygame.Rect(0, 0, TOOLBAR_WIDTH, ENV_HEIGHT)
        pygame.draw.rect(screen, TOOLBAR_COLOR, toolsBar, width=0)

        # Place buttons in tools bar
        DRAW_OBSTACLE_BUTTON.draw(screen)
        PLACE_GOAL_BUTTON.draw(screen)
        PLACE_ROBOT_BUTTON.draw(screen)
        BUG1_BUTTON.draw(screen)
        BUG2_BUTTON.draw(screen)
        TANGENT_BUG_BUTTON.draw(screen)
        GO_BUTTON.draw(screen)
        RESET_PLACES_BUTTON.draw(screen)
        RESET_ALL_BUTTON.draw(screen)

        # Draw obstacles
        for obstacle in obstacles:
            screenActors.drawObstacle(screen, obstacle, OBSTACLE_COLOR, OBSTACLE_WIDTH)

        # Draw new obstacle
        screenActors.drawNewObstacle(screen, obstacles, newObs,
                                     DRAW_OBSTACLE_BUTTON, OBSTACLE_COLOR,
                                     OBSTACLE_WIDTH, TOOLBAR_WIDTH, wasMousePresed)

        # Place Goal
        goal.placeGoal(screen, PLACE_GOAL_BUTTON, TOOLBAR_WIDTH, wasMousePresed)

        # Place Robot
        robot.placeRobot(screen, PLACE_ROBOT_BUTTON, TOOLBAR_WIDTH, wasMousePresed)

        # Reset Options
        if RESET_PLACES_BUTTON.wasPressed or RESET_ALL_BUTTON.wasPressed:
            robot.reset()
            goal.reset()
            GO_BUTTON.reset()
            RESET_PLACES_BUTTON.reset()

            if RESET_ALL_BUTTON.wasPressed:
                obstacles = []
                BUG1_BUTTON.reset()
                BUG2_BUTTON.reset()
                TANGENT_BUG_BUTTON.reset()
                RESET_ALL_BUTTON.reset()

        # Move to goal
        if GO_BUTTON.wasPressed == True and robot.goalReached == False:
            # Check collision
            angles = robot.checkCollision(screen, OBSTACLE_COLOR)

            if not robot.collision:
                robot.moveTowardGoal(screen, goal.pos, OBSTACLE_COLOR)

            robot.drawHistory(screen)

            pygame.time.delay(100)
        
        if robot.goalReached:
            robot.drawHistory(screen)

        wasMousePresed = False
        pygame.display.flip()    # Update the display
        


    pygame.quit()
