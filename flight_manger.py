'''
flight_manager.py
version: 1.0.2

Theodore Tasman
2025-01-30
PSU UAS

This module is responsible for managing the flight of the drone.

Flight class - manages the flight plan of the drone.

SITL Start Command: 
python3 ../MAVLink/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --console --map --custom-location 38.31527628,-76.54908330,40,282.5

'''

from flight_utils import Mission_Item, Mission, Coordinate
from mav_controller import Controller
from pymavlink import mavutil
import time

class Flight:
    '''
        Manages the flight plan of the drone.
    '''
    
    PREFLIGHT_CHECK_ERROR = 301
    DETECT_LOAD_ERROR = 302

    def __init__(self):
        '''
            Initialize the flight manager.
        '''
        # Create a controller object
        self.controller = Controller()

        # initialize preflight check
        self.preflight_check_done = False

        # create missions
        self.takeoff_mission = Mission(self.controller)
        self.detect_mission = Mission(self.controller)
        self.land_mission = Mission(self.controller)
        self.airdrop_mission = Mission(self.controller)
        self.geofence = Mission(self.controller, type=1) # type 1 is geofence

        # initialize mission list
        self.mission_list = [self.takeoff_mission]


    def decode_error(self, error_code):
        '''
            Decode an error code.
            error_code: int
            returns: str
        '''

        errors_dict = {
            301: "\nPREFLIGHT CHECK ERROR (301)\n",
            302: "\nDETECT LOAD ERROR (302)\n"
        }

        # check if error code is a controller error
        if error_code >= 100 and error_code < 200:
            return self.controller.decode_error(error_code)
        
        # check if error code is a flight error
        elif error_code >= 200 and error_code < 300:
            return self.decode_flight_error(error_code)

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")


    def takeoff(self, takeoff_mission_file):
        '''
            Takeoff the drone. Preflight check must be done first.
            returns:
                0 if the drone took off successfully
                Mission error code if the drone failed to takeoff (see flight_utils Mission class)
        '''

        # verify preflight check
        if not self.preflight_check_done:
            return self.PREFLIGHT_CHECK_ERROR

        # Load the takeoff mission from the file
        response = self.takeoff_mission.load_mission_from_file(takeoff_mission_file)

        # verify that the mission was loaded successfully
        if response:
            return response

        # send the takeoff mission
        print("Sending takeoff mission")
        response = self.takeoff_mission.send_mission()

        # verify that the mission was sent successfully
        if response:
            return response
        
        # wait a second for mission to be fully received
        time.sleep(1)

        # set the mode to AUTO
        print("Setting mode to AUTO")
        response = self.controller.set_mode('AUTO')

        # verify that the mode was set successfully
        if response:
            return response
        
        # arm the drone
        print("Arming drone")
        response = self.controller.arm()

        # verify that the drone was armed successfully
        if response:
            return response
        
        return 0
        


    def build_airdrop_mission(self, target_coordinate, airdrop_mission_file, target_index):
        '''
            Build the airdrop mission.
            target_coordinate: Coordinate
            airdrop_mission_file: str
            target_index: int
            returns:
                0 if the mission was built successfully
                Mission error code if the mission failed to build (see flight_utils Mission class)
        '''

        # Load the mission from the file up to the target index
        result = self.airdrop_mission.load_mission_from_file(airdrop_mission_file, end=target_index)
        
        # verify that the mission was loaded successfully
        if result:
            return result

        # Create a target mission item
        target_mission_item = Mission_Item(
            frame=3,
            command=16,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=target_coordinate.latitude,
            y=target_coordinate.longitude,
            z=0
        )

        # Add the target mission item to the mission
        result = self.airdrop_mission.add_mission_item(target_mission_item)
        # verify that the mission item was added successfully
        if result:
            return result

        # Load the rest of the mission from the file
        result = self.airdrop_mission.load_mission_from_file(airdrop_mission_file, start=target_index)
        # verify that the mission was loaded successfully
        if result:
            return result
        
        return 0
    

    def append_airdrop_mission(self):
        '''
            Append the airdrop mission to the mission list.
        '''
        self.mission_list.append(self.airdrop_mission)
    

    def append_detect_mission(self, detect_mission_file=None):
        '''
            Append the detect mission to the mission list.
        '''
        if self.detect_mission.get_length() == 0:
            
            # Check if file is provided
            if detect_mission_file:
                self.detect_mission.load_mission_from_file(detect_mission_file)

            # if no file provided, return error
            else:
                return self.DETECT_LOAD_ERROR


        self.mission_list.append(self.detect_mission)
        print("self.mission_list: ", self.mission_list)
    

    def wait_and_send_next_mission(self):
        '''
            Waits for the last waypoint to be reached,
            clears the mission, sends the next mission,
            sets mode to auto.
            target_index: int
            returns:
                0 if the mission was sent successfully
                101 if the response timed out
        '''
        # Get the current mission
        current_mission = self.mission_list.pop(0)

        # if the mission list is empty, set the next mission to the land mission
        if len(self.mission_list) == 0:
            print("No more missions, queueing land mission")
            next_mission = self.land_mission
        
        # otherwise, set the next mission to the next mission in the list
        else:
            print("Queuing next mission in list")
            next_mission = self.mission_list[0]

        # calculate the target index
        target_index = current_mission.get_length() - 1

        # Wait for the target index to be reached
        response = current_mission.wait_for_waypoint_reached(target_index, 30)

        # verify that the response was received
        if response == Controller.TIMEOUT_ERROR:
            return response

        # Clear the mission
        current_mission.clear_mission()

        # Send the next mission
        result = next_mission.send_mission()

        # set the mode to AUTO
        response = self.controller.set_mode('AUTO')

        # verify that the mode was set successfully
        if response:
            return response

        return result
    

    def preflight_check(self, land_mission_file, geofence_file, home_coordinate=(0,0,0)):
        '''
            Perform a preflight check. On success, set preflight_check_done to True.
            land_mission_file: str
            geofence_file: str
            home_coordinate: Coordinate (optional, if not provided, the current position will be used)
            returns:
                0 if the preflight check passed
                nonzero if the preflight check failed
        '''
        
        # Set home location
        response = self.controller.set_home(home_coordinate)

        # verify that the home location was set successfully
        if response:
            return response
        
        # load geofence
        response = self.geofence.load_mission_from_file(geofence_file)

        # verify that the geofence was loaded successfully
        if response:
            return response
        
        # send geofence
        response = self.geofence.send_mission()

        # verify that the geofence was sent successfully
        if response:
            return response
        
        # load land mission
        response = self.land_mission.load_mission_from_file(land_mission_file)

        # verify that the land mission was loaded successfully
        if response:
            return response
        
        # enable geofence
        response = self.controller.enable_geofence()

        # verify that the geofence was enabled successfully
        if response:
            return response
        
        # set preflight check done
        self.preflight_check_done = True

        return 0
        




    

        
            

def main():

    # Create a flight object
    flight = Flight()
    
    # Perform preflight check
    response = flight.preflight_check('landing_mission.txt', 'geofence.txt')
    if response:
        print(flight.decode_error(response))
        return

    # Append detect mission
    response = flight.append_detect_mission('airdrop_detect_mission.txt')
    if response:
        print(flight.decode_error(response))
        return

    # Takeoff the drone
    response = flight.takeoff('takeoff_mission.txt')
    if response:
        print(flight.decode_error(response))
        return

    # wait and run next mission
    response = flight.wait_and_send_next_mission() # detect mission
    if response:
        print(flight.decode_error(response))
        return

    # wait and run next mission
    response = flight.wait_and_send_next_mission() # land mission

    # if the land mission fails
    if response:

        # print the error
        print(flight.decode_error(response))
        
        # wait for drone to recover
        time.sleep(30)

        # retry the land mission
        flight.land_mission.send_mission()
        flight.controller.set_mode('AUTO')



if __name__ == '__main__':
    main()