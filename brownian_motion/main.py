import pygame
from borwnian_module import Brownian_Bot

# pygame setup
pygame.init()
screen_width = 800
screen_height = 400
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()
running = True

# create a Brownian Bot
bot = Brownian_Bot(screen_width, screen_height, 10, "blue")

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("white")

    pygame.draw.circle(screen, bot.color, bot.render(), bot.radius)
    pygame.draw.line(screen, bot.color, bot.render(), bot.position + bot.velocity * 20)

    # RENDER YOUR GAME HERE

    # flip() the display to put your work on screen
    pygame.display.flip()

    bot.update()
    clock.tick(60)  # limits FPS to 60

pygame.quit()
