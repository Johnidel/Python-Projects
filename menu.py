import pygame
import time
import sys

##Things that need fixing:
## - When Instructions or Credits is clicked, if the cursor is moved even
##   the slightest, the instruction/credit box disappears.
## - Cursor icon should change when hovering over a clickable item. Right now,
##   it's the same old arrow cursor throughout.
## - BETTER ICON DESIGNS. I just took the icons from the internet; I think we
##   should create our own, or at least choose better ones from the internet.
## - To put music or not to put music? That is the question.

##Possible Fixes:
## - Make a boolean that is toggled on when you click one of the icons
##   that will remain on until the mouse is moved outside of the icon rectangle
## - I kinda fixed your cursor thing but you need to upload a bustom cursor image
##   or use a crappy default one. There is a flickering problem which I can't figure out.
## - I think we should make our own images as well, so that the art 
## - style is the same throughout
## - We should put music, but I don't like this one.

class Menu:
    def __init__(self, handler):
        self.size = (1024,768)
        self.title = pygame.image.load("res/211title.png").convert_alpha()
        self.bgImg = pygame.image.load("res/menubg.png").convert_alpha()
        playIcon = pygame.image.load("res/playicon.png").convert_alpha()
        instructionsIcon = pygame.image.load("res/instructionsicon.png").convert_alpha()
        creditsIcon = pygame.image.load("res/creditsicon.png").convert_alpha()
        self.icon_list = [playIcon, instructionsIcon, creditsIcon]
        pygame.font.init()

        self.handler = handler



        icon_rect_list = []
        x = 0
        for icon in self.icon_list:
            icon_rect = icon.get_rect()
            icon_rect.x,icon_rect.y = (420,450+x)
            icon_rect_list.append(icon_rect)

        self.icon_rect_list = []

        x = 0
        for icon in self.icon_list:
            icon_rect = icon.get_rect()
            icon_rect.x,icon_rect.y = (420,450+x)
            self.icon_rect_list.append(icon_rect)
            x+=100
        
        self.font = pygame.font.SysFont('comicsansms', 16)

        #This is if we want to add music. I suggest we use a more fitting song/sound
        #though (same thing applies for all the images I used)
        pygame.mixer.music.load("res/Nas.wav")
        pygame.mixer.music.play(-1)

    #Used for instructions/credit screen
    def onMouseClick(self, screen, JPEG):
        if pygame.mouse.get_pressed()[0]:
            imgToDraw = pygame.image.load(JPEG)
            screen.blit(imgToDraw,(58,450))

    def draw(self, screen):
        screen.fill((0, 0, 0))
        screen.blit(self.bgImg,(0,0))
        screen.blit(self.title,(350,100))

        for i in range(len(self.icon_list)):
            screen.blit(self.icon_list[i], self.icon_rect_list[i])
        
        for i, icon in enumerate(self.icon_rect_list):
            if icon.collidepoint(pygame.mouse.get_pos()) and i == 0:
                playIcon = pygame.image.load("res/play_highlighted.png")
                screen.blit(playIcon, (295,330))
                playGameTxt = self.font.render("Play Game", 1, (255,0,0))
                screen.blit(playGameTxt, (320, 450))
                pygame.mouse.set_cursor(*pygame.cursors.diamond)
                if pygame.mouse.get_pressed()[0]:
                    pygame.mixer.music.fadeout(1500)
                    self.handler.curState = 3
                break

            if icon.collidepoint(pygame.mouse.get_pos()) and i == 1:
                instructionsIcon = pygame.image.load("res/instructions_highlighted.png")
                screen.blit(instructionsIcon, (445,330))
                instructionsTxt = self.font.render("Instructions", 1, (255,0,0))
                screen.blit(instructionsTxt, (460, 450))
                pygame.mouse.set_cursor(*pygame.cursors.diamond)
                self.onMouseClick(screen, "res/instructions_box.jpg")
                if pygame.mouse.get_pressed()[0]:
                    self.handler.curState = 3
                break

            if icon.collidepoint(pygame.mouse.get_pos()) and i == 2:
                creditsIcon = pygame.image.load("res/credits_highlighted.png")
                screen.blit(creditsIcon,(595,330))
                creditsTxt = self.font.render("Credits", 1, (255,0,0))
                screen.blit(creditsTxt, (625, 450))
                pygame.mouse.set_cursor(*pygame.cursors.diamond)
                self.onMouseClick(screen, "res/credits_box.jpg")
                if pygame.mouse.get_pressed()[0]:
                    self.handler.curState
                break

    def update(self):
        pass
            
