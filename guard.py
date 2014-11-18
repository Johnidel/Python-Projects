import pygame
import sys
import time
import urllib
import math
import os
import urllib.request
import io
import astar

def rot_center(image, rect, angle):
	"""rotate an image while keeping its center"""
	rot_image = pygame.transform.rotate(image, angle)
	rot_rect = rot_image.get_rect(center=rect.center)
	return rot_image,rot_rect

class Guard:
	def __init__(self, imageFile, pos_, path_):
		#initialize vars
		self.path = path_
		self.speed = 4
		self.pos = [0,0]
		self.pos[0], self.pos[1] = pos_[0], pos_[1]
		self.img = pygame.image.load(imageFile)
		self.flash = pygame.image.load("flashlight.png")
		self.guardRect = self.img.get_rect()
		self.guardRect.center = self.pos
		self.startPoint = self.guardRect.center[0], self.guardRect.center[1]
		#double size of flashlight image
		self.flash = pygame.transform.scale2x(self.flash)

	def draw(self, screen):
		#rotate the image of the guard
		rot_img, self.guardRect = rot_center(self.img, self.guardRect, self.theta)

		#move pos based on speed
		self.pos[0], self.pos[1] = self.pos[0] - (self.magMove[0] * self.speed), self.pos[1] - (self.magMove[1] * self.speed)

		#move the guard based on position
		self.guardRect.center = self.pos[0], self.pos[1]

		#draw guard on screen
		screen.blit(rot_img, self.guardRect)

		#rotate flashlight
		rot_img, newRect = rot_center(self.flash, self.flash.get_rect(), self.theta)

		#position flashlight at center of guard
		newRect.center = self.guardRect.center

		#move the flashlight to tip of guard based on direction(this was hard)
		newRect.center = (newRect.center[0] + 1 - (math.sin(math.radians(self.theta)) * ( 44 + self.flash.get_height() / 2)), newRect.center[1] + 1 - (math.cos(math.radians(self.theta)) * (44 + self.flash.get_height() / 2)))

		#draw flashlight
		screen.blit(rot_img, newRect)


	def update(self):
		self.magMove = [0, 0]
		thet = 0

		#get direction of movement based on path
		dirMove = (self.startPoint[0] - self.path[0][0], self.startPoint[1] - self.path[0][1])
		
		#get unit vector for movement
		mag = ((dirMove[0] ** 2) + (dirMove[1] ** 2)) ** (1/2)

		#set the class var to the movement determine angle
		if mag != 0: 
			self.magMove[0], self.magMove[1] = dirMove[0] / mag, dirMove[1] / mag
			thet = math.asin(dirMove[0]/ mag)
		#adjust angle based of the direction of movement
		if dirMove[1] > 0 and dirMove[0] < 0:
			thet = math.pi - thet
		if dirMove[1] > 0 and dirMove[0] >= 0:
			thet = math.pi - thet

		#convert to degrees and assign to class var
		self.theta = (-thet * 180 / math.pi) + 180

		if math.fabs(self.path[0][0] - self.guardRect.center[0]) <= 5 and math.fabs(self.path[0][1] - self.guardRect.center[1]) <= 5:
			self.startPoint = self.path[0]
			self.guardRect.center = (self.path[0][0], self.path[0][1])
			tmp = self.path.pop(0)
			self.path.append(tmp)