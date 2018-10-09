import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
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
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if len(Rover.nav_angles) >= Rover.rand_angle_thresh:
                    if np.random.random() <= 0.4:
                        # print("get random direction because angle: ", len(Rover.nav_angles))
                        if Rover.steer >= 0:
                            Rover.steer = np.clip(np.random.normal(Rover.steer - 5, 5), -15, 15)
                        else:
                            Rover.steer = np.clip(np.random.normal(Rover.steer + 5, 5), -15, 15)
                

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                if Rover.sample_angle is None or len(Rover.sample_angle) <15:
                    # print("Smaller than stop forward")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

            

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                # print("too fast in a stop mode so stop")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    # print("i am stopped and cannot go so i will look around ")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # print("let's go again because i can go now")
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
            # print("nothing to do so do something")
            Rover.throttle = 0
            Rover.steer = 15
            Rover.brake = 0
            
        # If in a state where want to pickup a rock send pickup command
        if Rover.near_sample:
            if Rover.vel == 0:
                # print("picking rock up")
                if not Rover.picking_up:
                    Rover.send_pickup = True
            else:
                # print("stop to pick rock")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        elif Rover.sample_angle is not None and len(Rover.sample_angle) >= 15:
            Rover.steer = np.clip(np.median(Rover.sample_angle * 180/np.pi), -15, 15)
            if Rover.vel >= 1:
                # print("STOPPING for TO ROCKS")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.mode = 'stop'
            else:
                # print("Going for rocks")
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.mode = 'forward'
            # print("################# rock angles: ", len(Rover.rock_angles))

    
    return Rover