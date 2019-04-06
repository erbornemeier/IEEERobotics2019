from collections import deque

#block information
current_letter = -1
current_block = 0

num_blocks = -1
x_coords = []
y_coords = []
block_queue = deque()
current_block = 0

#mothership position and mothership safety diamond
mothership_x = -1
mothership_y = -1
mothership_theta = -1

abc_x = -1
abc_y = -1
def_x = -1
def_y = -1
af_x = -1
af_y = -1
cd_x = -1
cd_y = -1

abc_bb_x = -1
abc_bb_y = -1
def_bb_x = -1
def_bb_y = -1
af_bb_x = -1
af_bb_y = -1
cd_bb_x = -1
cd_bb_y = -1

abc_approach_x = -1
abc_approach_y = -1
def_approach_x = -1
def_approach_y = -1

bad_points = set()
mothership_bad_points = set()
block_attempts = dict()
max_attempts = 5
detected_letters = dict()

# Letters of blocks currently placed
placed_blocks = []
tmp_slot = -1

def letterToCharacter(letter):
    return chr(ord('A') + letter)
