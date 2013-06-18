import time
import math

class TankDrive():
    """
    Implements a basic tank-like drive system with four wheels & a motor per wheel.
    This is available in case you want to use your robot outside, where we recommend
    the use of rubber/pneumatic tires for better traction, but without holonomic drive.
    """
    def __init__(self):
        self.max_x_velocity = 1.5    #Maximum X (forwards) velocity in M/S
        self.min_x_velocity = -1.5   #Minimum X (backwards) velocity in M/S
        self.max_z_rotational = 0.0    #Maximum Z (counter-clockwise) rotation in radians/S
        self.min_z_rotational = 0.0    #Minimum Z (clockwise) rotation in radians/S
        self.max_vehicle_velocity = 1.5 #Maximum vehicle velocity in M/S
    
    def Compute(self, X_velocity, Z_rotational):
        """
        Arguments:
        
        X_velocity (M/S), Z_rotational (Radians/S)

        Returns:

        [LF, RF, LR, RR], all in the range between -1.0 and 1.0 (fraction of maximum motor power)
        
        This function does the critical step of converting a velocity vector into float wheel speed commands
        """
        # While this is hardly the most elegant way to do it, for readability and
        # understandability, the 8 different cases will be treated separately here
        if (X_velocity == 0.0 and Z_rotational == 0.0):
            # This is a trivial case where we're stopped
            return [0.0, 0.0, 0.0, 0.0]
        
        elif (X_velocity != 0.0 and Z_rotational == 0.0):
            # When we're going straight forwards/backwards
            fraction_of_max = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if X_velocity > 0.0:
                return [fraction_of_max, fraction_of_max, fraction_of_max, fraction_of_max]
            elif X_velocity < 0.0:
                return [-fraction_of_max, -fraction_of_max, -fraction_of_max, -fraction_of_max]
                
        elif (X_velocity == 0.0 and Z_rotational != 0.0):
            # When we're only turning in place
            fraction_of_max = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if Z_rotational > 0.0:
                return [-fraction_of_max, fraction_of_max, -fraction_of_max, fraction_of_max]
            elif Z_rotational < 0.0:
                return [fraction_of_max, -fraction_of_max, fraction_of_max, -fraction_of_max]
            
        elif (X_velocity != 0.0 and Z_rotational != 0.0):
            # When we're going forwards/backwards and turning (moving like a car)
            # multiply inside motor commands by (1 - percentage of max rotational)
            fraction_of_max_x = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max_x > 1.0:
                fraction_of_max_x = 1.0
                
            fraction_of_max_z = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max_z > 1.0:
                fraction_of_max_z = 1.0
                
            if X_velocity > 0.0 and Z_rotational > 0.0:
                return [(fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x]
            elif X_velocity < 0.0 and Z_rotational > 0.0:
                return [-(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x]
            elif X_velocity > 0.0 and Z_rotational < 0.0:
                return [fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z))]
            elif X_velocity < 0.0 and Z_rotational < 0.0:
                return [-fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z))]

class MecanumDrive():
    """
    Implements the drive control math for a holonomic (all directions) drive system
    using mecanum wheels. NOT RECOMMENED FOR OUTDOOR USE
    """

    def __init__(self):
        self.max_x_velocity = 1.5    #Maximum X (forwards) velocity in M/S
        self.min_x_velocity = -1.5   #Minimum X (backwards) velocity in M/S
        self.max_y_velocity = 1.5    #Maximum Y (rightwards) velocity in M/S
        self.min_y_velocity = -1.5   #Minimum Y (leftwards) velocity in M/S
        self.max_z_rotational = 2.4    #Maximum Z (counter-clockwise) rotation in radians/S
        self.min_z_rotational = -2.4    #Minimum Z (clockwise) rotation in radians/S
        self.max_vehicle_velocity = 1.5 #Maximum vehicle velocity in M/S

    def Compute(self, X_velocity, Y_velocity, Z_rotational):
        """
        Arguments:
        
        X_velocity (M/S), Y_velocity (M/S), Z_rotational (Radians/S)

        Returns:

        [LF, RF, LR, RR], all in the range between -1.0 and 1.0 (fraction of maximum motor power)
        
        This function does the critical step of converting a velocity vector into float wheel speed commands
        """
        # While this is hardly the most elegant way to do it, for readability and
        # understandability, the 8 different cases will be treated separately here
        if (X_velocity == 0.0 and Y_velocity == 0.0 and Z_rotational == 0.0):
            # This is a trivial case where we're stopped
            return [0.0, 0.0, 0.0, 0.0]
        
        elif (X_velocity != 0.0 and Y_velocity == 0.0 and Z_rotational == 0.0):
            # When we're going straight forwards/backwards
            fraction_of_max = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if X_velocity > 0.0:
                return [fraction_of_max, fraction_of_max, fraction_of_max, fraction_of_max]
            elif X_velocity < 0.0:
                return [-fraction_of_max, -fraction_of_max, -fraction_of_max, -fraction_of_max]
                
        elif (X_velocity == 0.0 and Y_velocity != 0.0 and Z_rotational == 0.0):
            # When we're going straight left/right
            fraction_of_max = abs(Y_velocity / self.max_y_velocity)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if Y_velocity > 0.0:
                return [fraction_of_max, -fraction_of_max, -fraction_of_max, fraction_of_max]
            elif Y_velocity < 0.0:
                return [-fraction_of_max, fraction_of_max, fraction_of_max, -fraction_of_max]
            
        elif (X_velocity == 0.0 and Y_velocity == 0.0 and Z_rotational != 0.0):
            # When we're only turning in place
            fraction_of_max = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if Z_rotational > 0.0:
                return [-fraction_of_max, fraction_of_max, -fraction_of_max, fraction_of_max]
            elif Z_rotational < 0.0:
                return [fraction_of_max, -fraction_of_max, fraction_of_max, -fraction_of_max]
            
        elif (X_velocity != 0.0 and Y_velocity == 0.0 and Z_rotational != 0.0):
            # When we're going forwards/backwards and turning (moving like a car)
            # multiply inside motor commands by (1 - percentage of max rotational)
            fraction_of_max_x = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max_x > 1.0:
                fraction_of_max_x = 1.0
                
            fraction_of_max_z = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max_z > 1.0:
                fraction_of_max_z = 1.0
                
            if X_velocity > 0.0 and Z_rotational > 0.0:
                return [(fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x]
            elif X_velocity < 0.0 and Z_rotational > 0.0:
                return [-(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x]
            elif X_velocity > 0.0 and Z_rotational < 0.0:
                return [fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z))]
            elif X_velocity < 0.0 and Z_rotational < 0.0:
                return [-fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z))]
            
        elif (X_velocity == 0.0 and Y_velocity != 0.0 and Z_rotational != 0.0):
            # When we're going left/right and turning
            # multiply inside motor commands by (1 - percentage of max rotational)
            fraction_of_max_y = abs(Y_velocity / self.max_y_velocity)
            if fraction_of_max_y > 1.0:
                fraction_of_max_y = 1.0
                
            fraction_of_max_z = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max_z > 1.0:
                fraction_of_max_z = 1.0
                
            if Y_velocity > 0.0 and Z_rotational > 0.0:
                return [(fraction_of_max_y * (1 - fraction_of_max_z)), -(fraction_of_max_y * (1 - fraction_of_max_z)), -fraction_of_max_y, fraction_of_max_y]
            elif Y_velocity < 0.0 and Z_rotational > 0.0:
                return [-(fraction_of_max_y * (1 - fraction_of_max_z)), (fraction_of_max_y * (1 - fraction_of_max_z)), fraction_of_max_y, -fraction_of_max_y]
            elif Y_velocity > 0.0 and Z_rotational < 0.0:
                return [fraction_of_max_y, -fraction_of_max_y, -(fraction_of_max_y * (1 - fraction_of_max_z)), (fraction_of_max_y * (1 - fraction_of_max_z))]
            elif Y_velocity < 0.0 and Z_rotational < 0.0:
                return [-fraction_of_max_y, fraction_of_max_y, (fraction_of_max_y * (1 - fraction_of_max_z)), -(fraction_of_max_y * (1 - fraction_of_max_z))]
            
        elif (X_velocity != 0.0 and Y_velocity != 0.0 and Z_rotational == 0.0):
            # When we're strafing diagonally
            fraction_of_max_x = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max_x > 1.0:
                fraction_of_max_x = 1.0

            fraction_of_max_y = abs(Y_velocity / self.max_y_velocity)
            if fraction_of_max_y > 1.0:
                fraction_of_max_y = 1.0

            #Correct to make sure we're not going to try to overdrive the vehicle
            total_fraction = math.sqrt(fraction_of_max_x ** 2 + fraction_of_max_y ** 2)
            normalized_total_fraction = total_fraction
            if total_fraction > 1.0:
                fraction_of_max_x = fraction_of_max_x / total_fraction
                fraction_of_max_y = fraction_of_max_y / total_fraction
                normalized_total_fraction = 1.0

            #Now, figure out how to mix them together
            if X_velocity > 0.0 and Y_velocity > 0.0:
                #Diagonally forward and to the right
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [normalized_total_fraction, sideways_damp, sideways_damp, normalized_total_fraction]

            elif X_velocity > 0.0 and Y_velocity < 0.0:
                #Diagonally forward and to the left
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [sideways_damp, normalized_total_fraction, normalized_total_fraction, sideways_damp]

            elif X_velocity < 0.0 and Y_velocity > 0.0:
                #Diagonally backward and to the right
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [sideways_damp, -normalized_total_fraction, -normalized_total_fraction, sideways_damp]

            elif X_velocity < 0.0 and Y_velocity < 0.0:
                #Diagonally backward and to the left
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [-normalized_total_fraction, sideways_damp, sideways_damp, -normalized_total_fraction]

            
        elif (X_velocity != 0.0 and Y_velocity != 0.0 and Z_rotational != 0.0):
            # When we're simultaneously strafing and turning (WHY?!)
            fraction_of_max_x = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max_x > 1.0:
                fraction_of_max_x = 1.0

            fraction_of_max_y = abs(Y_velocity / self.max_y_velocity)
            if fraction_of_max_y > 1.0:
                fraction_of_max_y = 1.0

            fraction_of_max_z = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max_z > 1.0:
                fraction_of_max_z = 1.0

            #Correct to make sure we're not going to try to overdrive the vehicle
            total_fraction = math.sqrt(fraction_of_max_x ** 2 + fraction_of_max_y ** 2)
            normalized_total_fraction = total_fraction
            if total_fraction > 1.0:
                fraction_of_max_x = fraction_of_max_x / total_fraction
                fraction_of_max_y = fraction_of_max_y / total_fraction
                normalized_total_fraction = 1.0

            #Now, figure out how to mix them together
            if X_velocity > 0.0 and Y_velocity > 0.0 and Z_rotational > 0.0:
                #Diagonally forward and to the right and rotate left
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [(normalized_total_fraction * (1 - fraction_of_max_z)), sideways_damp, (sideways_damp * (1 - fraction_of_max_z)), normalized_total_fraction]

            elif X_velocity > 0.0 and Y_velocity > 0.0 and Z_rotational < 0.0:
                #Diagonally forward and to the right and rotate right
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [normalized_total_fraction, (sideways_damp * (1 - fraction_of_max_z)), sideways_damp, (normalized_total_fraction * (1 - fraction_of_max_z))]

            elif X_velocity > 0.0 and Y_velocity < 0.0 and Z_rotational > 0.0:
                #Diagonally forward and to the left and rotate left
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [(sideways_damp * (1 - fraction_of_max_z)), normalized_total_fraction, (normalized_total_fraction * (1 - fraction_of_max_z)), sideways_damp]

            elif X_velocity > 0.0 and Y_velocity < 0.0 and Z_rotational < 0.0:
                #Diagonally forward and to the left and rotate right
                sideways_damp = fraction_of_max_x - fraction_of_max_y
                return [sideways_damp, (normalized_total_fraction * (1 - fraction_of_max_z)), normalized_total_fraction, (sideways_damp * (1 - fraction_of_max_z))]

            elif X_velocity < 0.0 and Y_velocity > 0.0 and Z_rotational > 0.0:
                #Diagonally backward and to the right and rotate left
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [(sideways_damp * (1 - fraction_of_max_z)), -normalized_total_fraction, (-normalized_total_fraction * (1 - fraction_of_max_z)), sideways_damp]

            elif X_velocity < 0.0 and Y_velocity > 0.0 and Z_rotational < 0.0:
                #Diagonally backward and to the right and rotate right
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [sideways_damp, (-normalized_total_fraction * (1 - fraction_of_max_z)), -normalized_total_fraction, (sideways_damp * (1 - fraction_of_max_z))]

            elif X_velocity < 0.0 and Y_velocity < 0.0 and Z_rotational > 0.0:
                #Diagonally backward and to the left and rotate left
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [(-normalized_total_fraction * (1 - fraction_of_max_z)), sideways_damp, (sideways_damp * (1 - fraction_of_max_z)), -normalized_total_fraction]

            elif X_velocity < 0.0 and Y_velocity < 0.0 and Z_rotational < 0.0:
                #Diagonally backward and to the left and rotate right
                sideways_damp = fraction_of_max_y - fraction_of_max_x
                return [-normalized_total_fraction, (sideways_damp * (1 - fraction_of_max_z)), sideways_damp, (-normalized_total_fraction * (1 - fraction_of_max_z))]
