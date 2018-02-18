import numpy as np

ANGLE_THRESH = 0.1

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
            # Check if sample is located
            if Rover.samples_located > Rover.samples_collected and not np.isnan(Rover.angle_to_nearest_rock) and Rover.collect_rocks:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.current_rock_angle = Rover.angle_to_nearest_rock
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

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            #if Rover.samples_located - Rover.samples_collected == 1 :
            if Rover.samples_located > Rover.samples_collected and not np.isnan(Rover.angle_to_nearest_rock) and Rover.collect_rocks:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                # Rover.steer = 0
                Rover.mode = 'rotate_to_rock'
                
                #Rover.distance_to_nearest_rock = mean_dist_rock
                #Rover.angle_to_nearest_rock = mean_angle_rock
            # If we're in stop mode but still moving keep braking
            elif Rover.vel > 0.2:
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

        # if a rock is found, steer the rover towards it
        # elif Rover.mode == 'rotate_to_rock':
        elif Rover.mode == 'rotate_to_rock' and not np.isnan(Rover.angle_to_nearest_rock):
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif abs(Rover.angle_to_nearest_rock) < ANGLE_THRESH:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'move_towards_rock'
                print("Move to move_towards_rock_state")
            elif Rover.angle_to_nearest_rock > ANGLE_THRESH:
                Rover.steer = 5
                Rover.brake = 0
                print("Steer pos")
            elif Rover.angle_to_nearest_rock < -ANGLE_THRESH:
                Rover.steer = -5
                Rover.brake = 0
                print("Steer neg")

        # if aiming towards rock, move forward
        elif Rover.mode == 'move_towards_rock' and not np.isnan(Rover.angle_to_nearest_rock):
            print("In move_towards_rock state")
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            Rover.steer = 0
            if Rover.angle_to_nearest_rock > ANGLE_THRESH:
                Rover.steer = 5
                Rover.brake = 0
                print("Steer pos")
            elif Rover.angle_to_nearest_rock < -ANGLE_THRESH:
                Rover.steer = -5
                Rover.brake = 0
                print("Steer neg")
            print("Moving towards rock")
            
    # Just to make the rover do something 
    # even if no modifications have been made to the code
        else:
            Rover.mode = 'forward'
    
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = 'stop'

    # If near sample, then stop to be able to pick up
    if Rover.near_sample:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

    
    return Rover

