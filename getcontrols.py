import pygame

def get_controls():
    keys = pygame.key.get_pressed()
    
    commands = [0, 0, 0] # forward/backward, rotate left/right, up/down

    if keys[pygame.K_LEFT]:
        commands[1] = -1
    if keys[pygame.K_RIGHT]:
        commands[1] = 1
    if keys[pygame.K_UP]:
        commands[0] = 1
    if keys[pygame.K_DOWN]:
        commands[0] = -1

    return commands