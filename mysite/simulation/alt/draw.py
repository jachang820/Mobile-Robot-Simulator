import pygame, sys, os
from pygame.locals import *


def scale(px, scale):
	return round(px*scale)


def blit_alpha(target, source, location, opacity):
        x = location[0]
        y = location[1]
        temp = pygame.Surface((source.get_width(), source.get_height())).convert()
        temp.blit(target, (-x, -y))
        temp.blit(source, (0, 0))
        temp.set_alpha(opacity)        
        target.blit(temp, location)


pygame.init()
s = 0.75
g = 0.5
screen = pygame.display.set_mode((scale(1000, s),scale(800, s)))

robot = pygame.image.load("robot.png").convert()
sim = pygame.image.load("robot.png").convert()


while True:

	rect = pygame.transform.rotozoom(robot, 45, g)
	rect = rect.convert_alpha()
	rect.set_colorkey((0,0,0))
	#rect.fill((255,255,255,0))

	#rect = pygame.transform.scale(robot, (scale(90,g),scale(108,g)))
	screen.blit(rect, (100,100))
	screen.blit(sim, (110, 110))
	pygame.display.flip()

	events = pygame.event.get()
	for event in events:
		if event.type == pygame.QUIT:
			sys.exit()
		elif event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE or event.key == pygame.K_RETURN:
				sys.exit()
