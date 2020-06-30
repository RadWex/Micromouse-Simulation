import sys
import numpy
import pygame
from pygame.locals import *
from os import listdir
from os.path import isfile, join


def load_graphics(path):
    graphics = []
    # list_of_files=natsort.natsorted(listdir(path),reverse=False)
    #list_of_files = sorted(listdir(path),key=lambda x: int(os.path.splitext(x)[0]))
    list_of_files = [f for f in listdir(path) if isfile(join(path, f))]

    for i in list_of_files:
        # print(i)
        graphics.append(pygame.image.load(path+"/"+i))

    return graphics


def draw_window():
    pygame.init()
    DISPLAYSURF = pygame.display.set_mode((500, 500))
    pygame.display.set_caption('Hello World!')
    maze_map = numpy.loadtxt('file.txt', delimiter=",", dtype=int)
    rows = maze_map.shape[0]
    cols = maze_map.shape[1]
    graphics = load_graphics("map_segments")
    while True:  # main game loop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        i = 0
        j = 0
        for x in range(0, rows):
            j = 0
            for y in range(0, cols):
                if maze_map[y][x] != 0:
                    DISPLAYSURF.blit(graphics[maze_map[y][x]], (i, j))
                j += 40
            i += 40
        pygame.display.update()


if __name__ == "__main__":
    draw_window()
