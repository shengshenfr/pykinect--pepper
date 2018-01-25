from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

import socket
import select
import threading as thd
import time
from naoqi import ALProxy

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None
        self.active_bodies_indices = []
        self._positions = None
        self._orientations = None

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()


            self.get_movements()
            

                #res.append([self.convert_positions(), self.convert_orientation(), hands])
                #return res
                #print(self.active_bodies_indices)
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


    def get_movements(self):    
        upRHand = []
        upLHand = []
        upTwoHand = []
        downTwoHand = []
        hand_states = [0,0]

        if self._bodies is not None:
            for k in range(0,3):
                print(k)
                for i in range(0, self._kinect.max_body_count):

                    #print(i)

                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    else:
                        if i not in self.active_bodies_indices:
                            self.active_bodies_indices.append(i)
                    
                    joints = body.joints
                    hands = [body.hand_right_state, body.hand_left_state]
                    #self._positions = body.joints
                    #self._orientations = body.joint_orientations

                    #print(hands) 
                    if hands[0] !=0 and hands[1] !=0 :
                        upTwoHand.append(1)
                    elif hands[0] ==0 and hands[1] !=0:
                        upLHand.append(1)
                        print("raise left ", upLHand)
                    elif hands[0] !=0 and hands[1] ==0:
                        upRHand.append(1)
                        print("raise right ",upRHand)
                    else :
                        downTwoHand.append(1)


                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])
            
            hand_states = self.calculerPossibility(downTwoHand,upLHand,upRHand,upTwoHand)
            upRHand = []
            upLHand = []
            upTwoHand = []
            downTwoHand = []
            print("hand_states  ",  hand_states) 
            self.sendMessage(hand_states)
            hand_states = [0,0]
                                                      
        # --- draw skeletons to _frame_surface
        ''' 
        if self._bodies is not None:

               
            for index in self.active_bodies_indices:
                if not self._bodies.bodies[index].is_tracked:
                    self.active_bodies_indices.remove(index)                   
            k = 0
            for i in range(0, self._kinect.max_body_count):
                k = k + 1

                #print(i)

                body = self._bodies.bodies[i]
                if not body.is_tracked: 
                    continue 
                else:
                    if i not in self.active_bodies_indices:
                        self.active_bodies_indices.append(i)
                
                joints = body.joints
                hands = [body.hand_right_state, body.hand_left_state]
                #self._positions = body.joints
                #self._orientations = body.joint_orientations

                
                if hands[0] !=0 and hands[1] !=0 :
                    downTwoHand.append(1)
                elif hands[0] !=0 and hands[1] ==0:
                    upLHand.append(1)
                elif hands[0] ==0 and hands[1] !=0:
                    upRHand.append(1)
                else :
                    upTwoHand.append(1)
                    
                #res.append([self.convert_positions(), self.convert_orientation(), hands])
                #print(joints)
                #print("@@@")
                #print(self._orientations)
                #print("####") 
            
                # convert joint coordinates to color space 
                joint_points = self._kinect.body_joints_to_color_space(joints)
                self.draw_body(joints, joint_points, SKELETON_COLORS[i])

                                
                if hands[0] !=0 and hands[1] ==0:
                    managerProxy.runBehavior('kinectmove/lever_main_droite')
                print(hands)
            
            if k == 20:
                hand_states = self.calculerPossibility(downTwoHand,upLHand,upRHand,upTwoHand)
                self.sendMessage(hand_states)
                k= 0
                break 
            '''
    
    def calculerPossibility(self,downTwoHand,upLHand,upRHand,upTwoHand):
        length_total = []



        length_downTwoHand = len(downTwoHand)
        length_total.append(length_downTwoHand)


        length_upLHand = len(upLHand)
        length_total.append(length_upLHand)

        length_upRHand = len(upRHand)
        length_total.append(length_upRHand)


        length_upTwoHand = len(upTwoHand)
        length_total.append(length_upTwoHand)

        index_max = length_total.index(max(length_total))
        if (index_max == 0):
            return [0,0]
        elif (index_max == 1):
            return [0,1]
        elif (index_max == 2):
            return [1,0]
        else :
            return [1,1]
        
    def sendMessage(self,hand_states):
        if hand_states[0] != 0 and hand_states[1] == 0:
            managerProxy.runBehavior('kinectmove/lever_main_gauche')

        if hand_states[0] == 0 and hand_states[1] != 0:
            managerProxy.runBehavior('kinectmove/lever_main_droite')

        if hand_states[0] != 0 and hand_states[1] != 0:
            managerProxy.runBehavior('kinectmove/lever_deux_mains')


__main__ = "Kinect v2 Body Game"
robotIP = "10.65.34.164"
managerProxy = ALProxy("ALBehaviorManager", robotIP, 9559)
game = BodyGameRuntime();
game.run();

