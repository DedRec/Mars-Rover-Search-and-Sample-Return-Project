import random
import numpy as np

'''places = [ {"center" : (48,94) , "radius" :40 , "partially_visited" : False ,"totally_visited" : False}
    , {"center" : (118,40) , "radius" : 40, "partially_visited" : False ,"totally_visited" : False},
    {"center" : (155,100) , "radius" :20 , "partially_visited" : False ,"totally_visited" : False},
           {"center": (115, 153), "radius":41 , "partially_visited": False, "totally_visited": False}
    ]'''
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
places = [ {"equ": (-83,0)  ,"partially_visited" : False ,"totally_visited" : False},
           {"equ": (0,-65)  ,"partially_visited" : False ,"totally_visited" : False},
           {"equ": (0,115)  ,"partially_visited" : False ,"totally_visited" : False} # dont forget to multiply by negative
           ]




def decision_step(Rover):   # checks if the position is nearly is the same as prev position

    #print("ROVER MODE " ,Rover.mode)
    def same_pos():
        if (abs(Rover.pos[0] - Rover.pos_prev[0]) < 0.01) and (abs(Rover.pos[1] - Rover.pos_prev[1]) < 0.01) and Rover.mode == "forward":
            return True
        else :
            return False
    def check_partially_visiting():
        #mark visited
        x,y = Rover.pos
        equ= [0,0,0]
        equ[0] = places[0]["equ"][0] + x
        equ[1] = places[0]["equ"][1] + y
        equ[2] = places[0]["equ"][0] - y
        for i in range(3):
            if(equ[i]<=0):
                places[i]["partially_visited"] = True
                if i not in Rover.visited :
                    Rover.visited.append(i)
    def check_totally_visited() :
        x, y = Rover.pos
        equ = [0, 0, 0]
        equ[0] = places[0]["equ"][0] + x
        equ[1] = places[0]["equ"][1] + y
        equ[2] = places[0]["equ"][0] - y
        for i in range(3):
            if (equ[i] > 0) and places[i]["partially_visited"]:
                places[i]["totally_visited"] = True
                if i not in Rover.t_visited:
                    Rover.t_visited.append(i)
        pass
    def check_if_inside_visited() :
        #rotate 180 and go forward for 0.5 seconds
        x,y = Rover.pos
        equ = [0, 0, 0]
        equ[0] = places[0]["equ"][0] + x
        equ[1] = places[0]["equ"][1] + y
        equ[2] = places[0]["equ"][0] - y
        for i in range(3):
            if(equ[0]<=0) and places[i]["totally_visited"] :
                return True

 

    #print(Rover.steer_count , "steer count")
    #print(Rover.pos_count , "POS count")
    #print(Rover.mode , "mode")
    ###STUCK ########################################

    check_partially_visiting()
    check_totally_visited()

    if Rover.mode == "forward" and check_if_inside_visited():
        Rover.mode = "jump_scare"


    if same_pos():
        Rover.pos_count +=1
    else :
        Rover.pos_count =0

    Rover.pos_prev = Rover.pos
    if(Rover.pos_count >= Rover.max_pos_count) :
        Rover.toggle = not Rover.toggle
        r = random.randint(0, 10) # if stuck try different moves
        if Rover.toggle :
            Rover.mode = "pickedUp"
        else :
            Rover.mode = "rotate" ##should be the stop mode where it rotate
            #seif change

        Rover.pos_count = 0

    ###########POS END#######################################
    #####STERING FOR TOO LONG #############
    upper = Rover.steer_prev + 2
    lower = Rover.steer_prev - 2
    if (Rover.steer >= lower) and (Rover.steer <= upper) and (Rover.steer > 10 or Rover.steer < -10) and not Rover.gold_flag and Rover.mode == "forward":
        Rover.steer_count += 1
    else:
        Rover.steer_count = 0

    if (Rover.steer_count >= Rover.max_steer_count) : ####better 250 needs to be tried ;;;;;;;;;;
        Rover.steer_count = 0
        Rover.brake = 15
        Rover.steer = -15
        Rover.mode = "pickedUp"
    Rover.steer_prev = Rover.steer

    Rover.gold_flag  = False

    #####STERING END ###############################

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!


    # Example:
    # Check if we have vision data to make decisions with

    if Rover.nav_angles is not None:
        # Check for Rover.mode status

        if Rover.mode == 'pickedUp': # i just picked up a rock and probably facing a wall # 1e4 is estimated to be 1 second
            #print("i just picked up")
            Rover.steer = 0             #bid5ol fel kol el amaken el day2a + bigarab amaken gdeda
            Rover.throttle = - 0.1
            Rover.brake = 0
            #print("going backward pep pep...")
            if Rover.picking_up == 0 :
                Rover.backward_timer += 1
            #    print("increasing")
            #print("Timer values",Rover.backward_timer)

            if Rover.backward_timer > int(80) :
                Rover.mode = 'forward'
                Rover.brake = 15
                Rover.backward_timer = 0

        elif Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'


        elif Rover.mode == "rotate":
            r = random.randint(0, 1)  # if stuck try different moves
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = (-1*r) * 15  # Could be more clever here about which way to turn -1/1
            Rover.rotate_timer +=1
            if(Rover.rotate_timer >= 120)  :
                Rover.rortate_timer = 0

                Rover.mode = "forward"  ##should be the stop mode where it rotate
                

        elif Rover.mode == "jump_scare": #just saw the devil
            if Rover.vel == 0:
                Rover.mode = "scared"
                Rover.first_yaw = Rover.yaw
                return Rover
            else :
                Rover.brake = 15
                Rover.throttle = 0
                # Release the brake to allow turning


        elif Rover.mode == "scared" : # rotate to run
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = 15  # Could be more clever here about which way to turn -1/1
            if (abs(Rover.yaw-Rover.first_yaw) >= 180): #rotate 180 degres
                Rover.mode = "run"
                return Rover

        elif Rover.mode == "run": #keep going until out
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -8, 8)
            Rover.brake = 0
            Rover.throttle = 0.3
            if not check_if_inside_visited() :
                Rover.mode = "forward"
            return Rover






        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    
    if Rover.near_sample:
        Rover.gold_flag = True
        Rover.throttle = 0
        Rover.brake = 20
        Rover.steer = 0


    elif len(Rover.rocks_angles) > 0:
        Rover.gold_flag = True
        Rover.throttle = 0.07
        Rover.steer = np.clip(np.mean(Rover.rocks_angles * 180 / np.pi), -15, 15)


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = "pickedUp"

    return Rover

