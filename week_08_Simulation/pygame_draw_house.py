# Import a library of functions called 'pygame'
import pygame
 
# Initialize the game engine
pygame.init()
 
# Define the colors we will use in RGB format
BLACK= ( 0,  0,  0)
WHITE= (255,255,255)
BLUE = ( 0,  0,255)
GREEN= ( 0,255,  0)
RED  = (255,  0,  0)
 
# Set the height and width of the screen
size  = [400,300]
screen= pygame.display.set_mode(size)
  
pygame.display.set_caption("Game Title")
  
#Loop until the user clicks the close button.
done= False
clock= pygame.time.Clock()
  
while not done:
  
    # This limits the while loop to a max of 10 times per second.
    # Leave this out and we will use all CPU we can.
    clock.tick(10)
     
    # Main Event Loop
    for event in pygame.event.get():# User did something
        if event.type == pygame.QUIT:# If user clicked close
            done=True # Flag that we are done so we exit this loop
  
    # All drawing code happens after the for loop and but
    # inside the main while done==False loop.
      
    # Clear the screen and set the screen background
    screen.fill(WHITE)
 
    '''
    Your Work.....
    '''
    pygame.draw.polygon(screen, GREEN, [[30,150], [125,100], [220,150]],5)
    pygame.draw.polygon(screen, GREEN, [[30,150], [125,100], [220,150]],0)
    pygame.draw.lines(screen, RED,False, [[50,150], [50,250], [200,250], [200,150]],5)
    pygame.draw.rect(screen, BLACK, [75,175,75,50],5)
    pygame.draw.rect(screen, BLUE, [75,175,75,50],0)
    pygame.draw.line(screen, BLACK, [112,175], [112,225],5)
    pygame.draw.line(screen, BLACK, [75,200], [150,200],5)
 
    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
  
# Be IDLE friendly
pygame.quit()

