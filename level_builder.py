import pygame

class LevelBuilder():
        
	def __init__(self, walls, guards):
		self.walls = walls
		self.guards = guards

	def draw(self, screen):
		gray = (104,104,104)
		for wall in self.walls:
			pygame.draw.rect(screen, gray, wall)
		for guard in self.guards:
			guard.update()
			guard.draw(screen)
			
	def getRectGrid(self):
		"""Returns a 64x48 matrix of 0s and 1s, where 1s denote the presence of a wall"""
	
		##Works under assumption that width of walls is 32 pixels (as in, takes up
		##two spots in the 64x48 grid). I can probably change it to work for all widths,
		##but I see no need.
		
		GridList = [[0 for x in range(64)] for y in range(48)]
		for wall in self.walls:
			left_coordinate, right_coordinate = int(wall.left/16), int(wall.right/16)
			top_coordinate, bottom_coordinate = int(wall.top/16), int(wall.bottom/16)
			
			for i in range(left_coordinate, right_coordinate):
				GridList[top_coordinate][i] = 1
				GridList[top_coordinate+1][i] = 1
				
			for i in range(top_coordinate, bottom_coordinate):
				GridList[i][left_coordinate] = 1
				GridList[i][left_coordinate+1] = 1
				
		return GridList
