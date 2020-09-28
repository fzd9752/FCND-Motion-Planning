import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from graph_planning_utils import Sampler, a_star, heuristic, create_grid, create_graph, load_csv

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal_position=np.array([-122.3990892, 37.79342573, -5.0], dtype='Float64')):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.global_goal_position = global_goal_position

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.N_SAMPLE = 50
        self.K = 8
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        lat0, lon0 = load_csv(filename)
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)        
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (int(np.ceil(local_position[0] - north_offset)), int(np.ceil(local_position[1] - east_offset)))        # TODO: convert start position to current position rather than map center
        start = (int(np.ceil(local_position[0])), int(np.ceil(local_position[1])), int(np.ceil(local_position[2])))       # TODO: convert start position to current position rather than map center

        # Set goal as some arbitrary position on the grid
        local_goal_position = global_to_local(self.global_goal_position, self.global_home)
        #grid_goal = (int(np.ceil(local_goal_position[0] - north_offset)), int(np.ceil(local_goal_position[1] - east_offset)))        # TODO: adapt to set goal as latitude / longitude position and convert
        goal = (int(np.ceil(local_goal_position[0])), int(np.ceil(local_goal_position[1])), int(np.ceil(local_goal_position[2])))        # TODO: adapt to set goal as latitude / longitude position and convert
        print('Local Start and Goal: ', start, goal)
        
        #print('Local Start and Goal: ', grid_start, grid_goal)
        
        ## Sampling points from the data
        sampler = Sampler(data, start, goal)
        polygons = sampler._polygons
        nodes = sampler.circular_random(self.N_SAMPLE)
        print("Valid sample nodes: %i" %len(nodes))
        
        ## Create graph based on samples
        g = create_graph(nodes, self.K, polygons)
        print("Number of edges", len(g.edges))
        
        # Run A* to find a path from start to goal
        # move to a different search space such as a graph (not done here)
        path, _ = a_star(g, heuristic, start, goal)
        
        # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        
        waypoints = [[p[0], p[1], p[2], 0] for p in path]
        print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=str, default=' -122.3990892', help="Goal longitude")
    parser.add_argument('--goal_lat', type=str, default='37.79342573', help="Goal latitude")
    parser.add_argument('--goal_alt', type=str, default='-5.0', help="Goal altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    goal_global_position = np.fromstring(f'{args.goal_lon},{args.goal_lat},{args.goal_alt}', dtype='Float64', sep=',')
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
