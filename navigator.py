""" A Module to objectify this problem into objects and associate
associated types with one another.  This module contains a NodeState
class, a MapState class and a NavigationPro class.  NavigationPro
drives the state machine using MapState and NodeStates to perform
A Star search on a mapping file.

MATTHEW CARLIS
"""
from collections import defaultdict
import sys
import copy
import math
import time
import heapq

# Useing these keys to represent (x, y) coordinates in tuple objects.
X_IND, Y_IND = 0, 1

############## HEURISTICS
MANHATTAN = lambda pos, dest, parent: (abs(pos[X_IND] - dest[X_IND]) + abs(pos[Y_IND] - dest[Y_IND]))
EULICID = lambda pos, dest, parent: math.sqrt((pos[X_IND] - dest[X_IND])**2 + (pos[Y_IND] - dest[Y_IND])**2)
def MADE_UP(pos, dest, matt_args):
    """A function to calculate the angle between the destination vector and
    the vector from a parent node to this position.  This angle is used to
    weight the """
    x_val, y_val = 0, 1
    h_eulicid = EULICID(pos, dest, None)
    parent, h_eulicid_max = matt_args
    if parent == pos or pos == dest:
        return h_eulicid
    vect_dest = (dest[x_val] - pos[x_val], dest[y_val] - pos[y_val])
    vect_n = (pos[x_val] - parent[x_val], pos[y_val] - parent[y_val])
    # Take the dot product of the vectro from the pos to the dest and
    # the vector from arent to pos.
    dot_product = vect_dest[x_val] * vect_n[x_val] + vect_dest[y_val] * vect_n[y_val]
    mag_vect_n = EULICID(pos, parent, None)
    mag_vect_dest = EULICID(dest, pos, None)
    try:
        inv_cos_arg = (dot_product*1.0) / (mag_vect_dest * mag_vect_n)
        # Take the angle between the (x,y) vector set.
        vect_theta = math.acos(inv_cos_arg)
    except:
        # Burp.
        return h_eulicid
    # Weight the period between 0,180 degrees to,from the destination
    # 180 degrees is amplitude, take abs.  This makes going away from
    # the destination harder.
    AMPLITUDE = 1
    h_vect = abs(AMPLITUDE*math.sin(vect_theta/2.0))
    if vect_theta < 1.22173048: # 1.2217 radians aprox 70 degrees.
        h_vect = 0
    ret_v = h_eulicid
    vect_weight = h_vect * (abs(h_eulicid - h_eulicid_max) / h_eulicid_max) # Use the vector as we get closer.
    #print parent, pos, dest, vect_theta*57.29578, (abs(h_eulicid - h_eulicid_max) / h_eulicid_max), vect_weight, ret_v + vect_weight
    return ret_v + vect_weight
############## END HEURISTICS


class NodeState(object):
    """ A class to store the state of a node with it's associated state
    variables such as position, parent node, total dimensions, whether
    or not it has been closed and it's cost from a source node.
    """
    def __init__(self, key, dimensions):
        self.position = key
        self.parent = {}
        self.x_size, self.y_size = dimensions
        self.closed = {}
        self.cost = {}
    def __repr__(self):
        return '<NodeState>{}'.format(self.position)

    def set_parent(self, dest_key, parent, cost):
        """ A function to set the parent of a node with it's cost from a source.
        """
        self.parent[dest_key] = parent
        self.cost[dest_key] = cost

    def get_parent(self, dest_key):
        """ A function to get the position of the parent node. """
        if not self.parent.has_key(dest_key):
            return None
        cp_p = copy.copy(self.parent[dest_key])
        return cp_p

    def get_cost(self, dest_key):
        """ A fucntion to return the cost to get to this node from a source
        node on the way to a destination node with position dest_key.
        """
        if not self.cost.has_key(dest_key):
            self.cost[dest_key] = None
        return self.cost[dest_key]

    def is_closed(self, dest_key):
        """ A function to check if this node is closed for the trip to the
        destination node with position dest_key
        """
        if self.closed.has_key(dest_key):
            return self.closed[dest_key]
        else:
            return False

    def close_node(self, dest_key):
        """ A function to close a node from a source to a particular destination
        with position dest_key.
        """
        if self.closed.has_key(dest_key):
            rval = self.closed[dest_key]
        else:
            rval = False
        self.closed[dest_key] = True
        return rval
    def open_node(self, dest_key):
        """ Why would I re-open the node?  Maybe I was being GREEDY.
        NOTE: Not in use currently..
        """
        if self.closed.has_key(dest_key):
            rval = self.closed[dest_key]
        else:
            rval = False
        self.closed[dest_key] = False
        return rval

    def get_heuristic(self, heur_func, dest_key, parent=None):
        """ A function to return the heuristic value for this position from
        a source node to a destination node with position dest_key.
        """
        return heur_func(self.position, dest_key, parent)

    def get_neighbors(self):
        """ A function to return a list of neighboring nodes for this position.
        """
        x_index, y_index = self.position
        neighbor_list = []
        if x_index + 1 < self.x_size:
            neighbor_list.append((x_index + 1, y_index))
        if x_index - 1 >= 0:
            neighbor_list.append((x_index - 1, y_index))
        if y_index + 1 < self.y_size:
            neighbor_list.append((x_index, y_index + 1))
        if y_index - 1 >= 0:
            neighbor_list.append((x_index, y_index - 1))
        return neighbor_list


class MapState(object):
    """ A class to store the state of the map according to global parameters
    and NodeState objects for each position in the cartesian graph.
    """

    def __init__(self, xy_sizes, init_pos=None):
        self.x_size, self.y_size = xy_sizes
        self.items = self.x_size * self.y_size
        self.airports = []
        self.initial_pos = init_pos
        self.current_state = self.initial_pos
        self.weather_state_map = {}
        self.closed_nodes = {}

    def set_init_pos(self, init_pos):
        """ A function to set the  initial position of the Map.
        """
        self.initial_pos = init_pos
        self.current_state = init_pos
        if not self.weather_state_map.has_key(init_pos):
            start_node_obj = NodeState(init_pos, (self.x_size, self.y_size))
            self.weather_state_map[init_pos] = start_node_obj

    def add_airport(self, airport_pos):
        """ A function to add an airport into the airports list.
        """
        if not self.closed_nodes.has_key(airport_pos):
            self.closed_nodes[airport_pos] = 0
        self.airports.append(airport_pos)

    def close_node(self, airport_pos, position):
        """ A function to close a node for a particular postion headed to an
        airport_pos
        """
        if not self.weather_state_map[position].close_node(airport_pos):
            self.closed_nodes[airport_pos] += 1
    def open_node(self, airport_pos, position):
        """ A function for the absurd programmer who opens his nodes
        after he's done with them.
        """
        if self.weather_state_map[position].open_node(airport_pos):
            self.closed_nodes[airport_pos] -= 1

    def get_node(self, node):
        """ A function to return a node state object from a dictionary.
        """
        if not self.weather_state_map.has_key(node):
            this_node = NodeState(node, (self.x_size, self.y_size))
            self.weather_state_map[node] = this_node
        return self.weather_state_map[node]
    def next_state(self, next_state):
        """ A function to set the current state of the map from a next_state.
        """
        self.current_state = next_state

class NavigatorPro(object):
    """ A class to handle the navigation of the map from a given source to
    one or many destination airports.  The class initializes all required
    state parameters for the map and nodes as needed.
    """
    unoc_ap, oc_ap = 'P', 'J'
    weight_mapping = {unoc_ap: 0, oc_ap: 0}
    airport_char_map = {unoc_ap: None, oc_ap: None}
    plane_char_map = {}
    mapping_init = False
    heuristic_map = {'manhattan': MANHATTAN, 'euclidean': EULICID, 'made_up': MADE_UP}
    def __init__(self, matrix, fuel, heuristic, test=False, verbose=False):
        self.i_time, self.fuel, self.test, self.verbose = time.time(), fuel, test, verbose
        self.reachable = {}
        self.x_size, self.y_size = len(matrix[0]), len(matrix)
        self.weather_matrix = matrix
        try:
            self.heuristic_func = self.heuristic_map[heuristic]
        except:
            print 'INVALID ARGUMENT'
            sys.exit(1)
        self.map_state_obj = MapState((len(matrix[0]), len(matrix)))
        # Set global dictionaries to map the weights from nodes to ints
        # Using this dictionary to get the numerical value of weight from
        # the A,B,C,D,E,F,G,H,I
        if not self.mapping_init:
            self._init_map()
            self.mapping_init = True
        # Scan the map for the airports and plane
        self._init_map_state()
        if test:
            print 'initialize time:', time.time() - self.i_time
            print 'Map Dimensions: width:', self.x_size, 'height:', self.y_size

    def find_paths(self):
        """ A function to find the path to the airport.  Used in plane_agent.py
        """
        self.search_A_star()

    def print_map_path(self, steps):
        """ A Function to produce the output described in the assignment.
        """
        step_list = copy.copy(steps)
        the_map = {'1':'A', '2':'B', '3':'C', '4':'D', '5':'E', '6':'F', 
            '7':'G', '8':'H', '9':'I', 'P':'J', 'A':'A', 'B':'B',
            'C':'C', 'D':'D', 'E':'E', 'F':'F', 'G':'G', 'H':'H', 'I':'I'}
        row_str = ''
        fuel = self.fuel
        for m_cnt, step in enumerate(step_list):
            if m_cnt == len(step_list)-1:
                print 'Map:{}  Fuel:{} Found an Airport!'.format(m_cnt, fuel)
            else:
                print 'Map:{}  Fuel:{}'.format(m_cnt, fuel)
            for y_cnt, row in enumerate(self.weather_matrix):
                for x_cnt, item in enumerate(row):
                    cord = self._get_cord((y_cnt, x_cnt))
                    if cord == step:
                        weight = self.weather_matrix[y_cnt][x_cnt]
                        row_str += the_map[self.weather_matrix[y_cnt][x_cnt]]
                        fuel -= self.weight_mapping[weight]
                    else:
                        weight = self.weather_matrix[y_cnt][x_cnt]
                        if weight == 'P':
                            row_str += weight
                        else:
                            row_str += str(self.weight_mapping[weight])
                print row_str
                row_str = ''
            #print ''


    def walk_tree(self, w_matrix, loud=False):
        """ A function to traverse the Tree from the airports to the source
        nodes following the parent pointers set from the A star search.
        """
        #print ''
        map_state = self.map_state_obj
        state_0 = map_state.initial_pos
        airports = map_state.airports
        paths = defaultdict(list)
        path_len, trip_cost = 0, {}
        y_indx_s0, x_indx_s0 = self._get_indices(state_0)
        # if any_path, we can make it to an airport.  else tragedy.
        any_path = False
        path_steps = []
        for airport in airports:
            # Skip airports that are far.
            if not self.reachable[airport]:
                continue
            any_path = True
            if not trip_cost.has_key(airport):
                trip_cost[airport] = self.weight_mapping[w_matrix[y_indx_s0][x_indx_s0]]
            path_len = 0
            state = airport
            #Back trace the airport who has the shortest path.
            while state != state_0:
                # Reverse the order while backtracing the tree from airport
                path_steps.insert(0, state)
                paths[airport].insert(0, state)
                y_neindex, x_neindex = self._get_indices(state)
                if state != airport:
                    trip_cost[airport] += self.weight_mapping[w_matrix[y_neindex][x_neindex]]
                this_node = map_state.get_node(state)
                path_len += 1
                next_state = this_node.get_parent(airport)
                state = next_state
            path_len = 0
        path_steps.insert(0, state_0)
        paths[airport].insert(0, state_0)
        #print 'path_steps', path_steps
        if not any_path:
            print 'No route exists.'
        else:
            self.print_map_path(path_steps)

    def print_parents(self, w_matrix):
        """ A test function to print the parents of nodes to see what watch
        for bugs.  Not in use.
        """
        map_state = self.map_state_obj
        for y_cnt, row in enumerate(w_matrix):
            for x_cnt, val in enumerate(row):
                cord = self._get_cord((y_cnt, x_cnt))
                this_node = map_state.get_node(cord)
                print this_node,
                print this_node.get_parent((4, 0))

    def search_A_star(self):
        """ A function to perform A Star search on the Map state object and
        Node State Objects.  The search begins for a source node and an airport
        trying all acceptable paths for each airport.  If no path is found, the
        navigation return False.
        """
        i_time = time.time()
        map_state, w_matrix, state_0, state_index = self._init_A_star()
        state, init_time = state_0, time.time()
        priority_queue = []
        airports, airport_index = map_state.airports, 0
        h_eulicid = {}
        for airport in airports:
            h_eulicid[airport] = EULICID(state_0, airport, None)

        not_found = True
        while not_found:
            #print 'state:', state, ':',
            # Loop over the states of discovery in f(g,h)=g()+h() priority
            h_arg = (state, h_eulicid[airport])
            # Get all these references and stuff.
            this_node = map_state.get_node(state)
            airport = airports[airport_index]
            this_cost = this_node.get_cost(airport)
            #this_h = this_node.get_heuristic(self.heuristic_func, airport, h_arg)
            neighbors = this_node.get_neighbors()
            # state, airport <x, y> coordinates quad I alignment.
            if state == airport: # If we get to the airport we made it.
                # Stop using stuff and set flags.
                self.reachable[airport] = True
                priority_queue, neighbors = [], []
                break
            for neighbor_key in neighbors: # Check each neighbor (x,y)
                neighbor_node = map_state.get_node(neighbor_key)
                # If i've been here don't come back.
                if neighbor_node.is_closed(airport):
                    continue
                y_neindex, x_neindex = self._get_indices(neighbor_key)
                weight_neighbor = self.weight_mapping[w_matrix[y_neindex][x_neindex]]
                h_neighbor_val = neighbor_node.get_heuristic(self.heuristic_func, airport, h_arg)
                # Convert (x,y) to nested lists using quadrent I.
                his_cost = neighbor_node.get_cost(airport)
                if his_cost is None or (this_cost + weight_neighbor) < his_cost:
                    his_cost = this_cost + weight_neighbor
                    neighbor_node.set_parent(airport, state, his_cost)
                    # Dont push to heap if it's too far.
                    if self.fuel >= his_cost:
                        heapq.heappush(priority_queue, (his_cost + h_neighbor_val, neighbor_key))
            map_state.close_node(airport, state)
            air_package = (airports, airport_index)
            # Pop off the queue the next state
            package = self._search_next_state(priority_queue, air_package)
            # next state <state>, STOP CONDITION <not_found>,
            # key'd airport cordinate <airport_index>
            state, not_found, airport_index = package
        #ENDWHILE
        self.walk_tree(w_matrix)
        if self.test:
            print 'path search time:', time.time() - init_time
            #print 'Time:', time.time() - i_time

    def _search_next_state(self, priority_queue, air_pack):
        """ Get the next state from the priority_queue skipping closed nodes.
        and checking for the ending search conditions.
        """
        map_state = self.map_state_obj
        state_0 = map_state.initial_pos
        airports, airport_index = air_pack
        not_found = True
        if len(priority_queue) <= 0:
            if airport_index + 1 >= len(airports):
                not_found = False
            airport_index += 1
            next_state = state_0
        else:
            while True:
                h_val, next_state = heapq.heappop(priority_queue)
                if not map_state.get_node(next_state).is_closed(airports[airport_index]):
                    break
                if len(priority_queue) <= 0:
                    if airport_index + 1 >= len(airports):
                        not_found = False
                    airport_index += 1
                    next_state = state_0
                    break
        return next_state, not_found, airport_index

    def _get_cord(self, (ylist_0, xlist_1)):
        """ From the nested lists indexes return the x_cord, y_cord.
        Remapping the nested lists into the first quadrant of the
        cartesian system.
        """
        return (xlist_1, self.y_size - 1 - ylist_0)
    def _get_indices(self, (x_index, y_index)):
        """ Give an (X_cord, Y_cord) return the indicies for the nested
        lists. Remapping the nested lists into the first quadrant of the
        cartesian system.
        """
        return (self.y_size - 1 - y_index, x_index)
    def _init_A_star(self):
        """ A function to initialize the A star search parameters.  This includes
        setting the parent of the starting position to himself and setting his cost.
        """
        map_state = self.map_state_obj
        w_matrix = self.weather_matrix
        state_0 = map_state.initial_pos
        this_node = map_state.get_node(state_0)
        state_index = self._get_indices(state_0)
        for dest_key in map_state.airports:
            cost = self.weight_mapping[w_matrix[state_index[0]][state_index[1]]]
            this_node.set_parent(dest_key, state_0, cost)
        return (map_state, w_matrix, state_0, state_index)
    def _init_map_state(self):
        """ A function to initialize the state of the map including
        finding the airports and starting positions.
        """
        map_state = self.map_state_obj
        unsort_airports = []
        for y_index in range(len(self.weather_matrix)):
            for x_index in range(len(self.weather_matrix[0])):
                weight = self.weather_matrix[y_index][x_index]
                x_cord, y_cord = self._get_cord((y_index, x_index))
                if self.airport_char_map.has_key(weight):
                    unsort_airports.append((x_cord, y_cord))
                    self.reachable[(x_cord, y_cord)] = False
                if self.plane_char_map.has_key(weight):
                    map_state.set_init_pos((x_cord, y_cord))
        init_pos, weighted_airports = map_state.initial_pos, []
        for airport in unsort_airports:
            heuristic = self.heuristic_func(init_pos, airport, (init_pos, EULICID(init_pos, airport, None)))
            weighted_airports.append((heuristic, airport))
        sorted_airports = [val[1] for val in sorted(weighted_airports, key=lambda heur: heur[0])]
        for airport in sorted_airports:
            map_state.add_airport(airport)

    def _init_map(self):
        """ A Function to initialize the weight_mapping of weights for squares.
        """
        # ascii_char: [A, I] -> ascii_decimal [65, 65 + 9)
        # ascii_decimal - ascii_shift -> weight: [1, 9]
        lower, upper, ascii_shift = (65, 65 + 9, 64)
        for val in range(lower, upper):
            weight = val - ascii_shift
            self.weight_mapping[chr(val)] = weight
            self.weight_mapping[str(weight)] = weight
            self.weight_mapping[weight] = weight
            self.plane_char_map[chr(val)] = None
        self.weight_mapping['0'] = 0
        self.weight_mapping[0] = 0

if __name__ == '__main__':
    _TEST_val = EULICID((0, 3), (3, 1), None)
    print 'val:', _TEST_val
    print MADE_UP((1, 2), (3, 1), ((0, 2), _TEST_val))
    print MADE_UP((0, 2), (3, 1), ((1, 2), _TEST_val))






