"""
A Module to handle command line arguments and handle error checking
from human inputs very generically.  This module instianiates
a NavigatorPro object to handle navigation mappings etc.

Not intended to be scripted on top of using stderr/stdin/stdout.
Use import NavigatorPro in your module instead or import
parse_arguments() directly.

MATTHEW CARLIS
"""
from navigator import NavigatorPro
import copy, time
import sys

# Use global to store arguments IF run
# as the top/__main__ file.  Else unused.
ARGS = []
TEST = False
VERBOSE = False
ARG_TUP = None
WEATHER_MAP = ''
HEURISTIC = ''
FUEL = 0


def parse_arguments(argument_tuple, test=False, verbose=False):
    """ A Function to parse a TUPLE with format
    ARG_TUP: (weather_map, heuristic, fuel), and
    run the associated simulation. use test=True to enable
    NavigatorPro's debug printing for the mapping objects.
    """
    weather_map = argument_tuple[0]
    heuristic = argument_tuple[1]
    fuel = argument_tuple[len(argument_tuple)-1]
    navi_sim_obj = NavigatorPro(weather_map, fuel, heuristic, test, verbose)
    navi_sim_obj.find_paths()


def arg_parser(args):
    """A Function to build the argument tuple object and configure
    it's arguments.
    DISCLAIMER: Extremely generic error handling.
                Intended for __name__==__main__: only!"""
    try:
        size = len(args)
        input_file = args[1]
        i_time = time.time()
        with open(input_file, 'r') as map_file:
            weather_map = [list(val) for val in map_file.read().rstrip('\n').splitlines()]
        heuristic = args[2]
        fuel = int(args[size-1])
        return (weather_map, heuristic, fuel)
    except Exception: # Don't try to figure out what's wrong
        print '\nERROR with input. Killing Program.'
        print 'Ex:  > python plane_agent.py <hueristic> <fuel>'
        print '<fuel>: integer number'
        print '<hueristic>: manhattan, eculidean, made_up\n'
        sys.exit(1)


if __name__ == '__main__':
    ARGS = copy.deepcopy(sys.argv)
    if '--test' in sys.argv:
        ARGS.remove('--test')
        TEST = True
    if '-v' in ARGS:
        ARGS.remove('-v')
        VERBOSE = True
    ARG_TUP = arg_parser(ARGS)
    WEATHER_MAP = ARG_TUP[1]
    HEURISTIC = ARG_TUP[2]
    FUEL = ARG_TUP[len(ARG_TUP)-1]
    parse_arguments(ARG_TUP, TEST, VERBOSE)
