#!/usr/bin/env python3


import rospy
import pygame
import math
from lego_car_msgs.msg import EnginePower


class UserInterface():
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800,600))
        pygame.display.set_caption("Lego Car Controller")
        
        
        #Buttons and Texts for the UI
        #User Is able to change between Car and Tankmode 
        self.mode_bt = pygame.Rect((300,50,200,50))
        self.font = pygame.font.Font(None, 32)
        self.bt_text = self.font.render("Change MODE", True, (0,128,0))
        self.speed_text = self.font.render("Speed: 0 [%]", True,(255,0,0))
        self.angle_text = self.font.render("Angle: 0 [RAD]", True, (128,128,0))


        #Data for the publisher, and the publisher itself
        self.data = EnginePower()
        self.pub = rospy.Publisher('enginePowerControl', EnginePower)
        rospy.init_node('ManualControl', anonymous=True)
        self.rate = rospy.Rate(10)
        self.up = False
        self.down = False
        self.left = False
        self.right = False
    
    def event_loop(self):
        

        #searching and interpreting of Key-Events
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_KP7:
                        self.data.left_engine = 100
                    if event.key == pygame.K_KP4:
                        self.data.left_engine = -100
                    if event.key == pygame.K_KP8:
                        self.data.right_engine = 100
                    if event.key == pygame.K_KP5:
                        self.data.right_engine = -100
                    if event.key == pygame.K_UP:
                        self.up = True
                    if event.key == pygame.K_DOWN:
                        self.down = True
                    if event.key == pygame.K_LEFT:
                        self.left = True
                    if event.key == pygame.K_RIGHT:
                        self.right = True
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_KP7:
                        self.data.left_engine = 0
                    if event.key == pygame.K_KP4:
                        self.data.left_engine = 0
                    if event.key == pygame.K_KP8:
                        self.data.right_engine = 0
                    if event.key == pygame.K_KP5:
                        self.data.right_engine = 0
                    if event.key == pygame.K_UP:
                        self.up = False
                    if event.key == pygame.K_DOWN:
                        self.down = False
                    if event.key == pygame.K_LEFT:
                        self.left = False
                    if event.key == pygame.K_RIGHT:
                        self.right = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if self.mode_bt.collidepoint(event.pos):
                        if self.data.mode == EnginePower.CARMODE:
                            self.data.mode = EnginePower.TANKMODE
                        else:
                            self.data.mode = EnginePower.CARMODE

            #Check if a arrow key is pressed and changing the data according to the pressed keys
            if self.up == True:
                self.data.speed += 5
            if self.down == True:
                self.data.speed -= 5
            if self.left == True:
                self.data.steering_angle += (math.pi / 2) / 20
            if self.right == True:
                self.data.steering_angle -= (math.pi / 2) / 20

            # Correction of too big or too small values
            if self.data.speed > 100:
                self.data.speed = 100
            if self.data.speed < -100:
                self.data.speed = -100
            if self.data.steering_angle > math.pi / 2:
                self.data.steering_angle = math.pi / 2
            if self.data.steering_angle < - math.pi / 2:
                self.data.steering_angle = - math.pi / 2

            #output and drawing of the pygame UI
            self.screen.fill((255,255,255))
            if self.data.mode == EnginePower.CARMODE:
                self.speed_text = self.font.render("Speed: " + str(self.data.speed) + " [%]",True, (255,0,0))
                self.angle_text = self.font.render(("Angle: " + str(self.data.steering_angle) + " [RAD]"),True,(128,128,0))
                self.screen.blit(self.speed_text,(50,200))
                self.screen.blit(self.angle_text, (50,250))

            pygame.draw.rect(self.screen,(255,0,0),self.mode_bt)
            self.screen.blit(self.bt_text,(300,50))

            pygame.display.update()

            #Publish and Wait
            self.pub.publish(self.data)
            self.rate.sleep()
            

if __name__ == '__main__':
    window = UserInterface()
    try:
        window.event_loop()
    except rospy.ROSInterruptException:
        pass
    
