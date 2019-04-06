import random

fout = 'usb/mars1.json'
num_blocks = input('number of blocks: ')
with open(fout, 'w') as f:
    f.write('{\n')
    f.write("\"size\": {},\n".format(num_blocks))
    rand_x = []
    rand_y = []
    for _ in range(num_blocks):
        rand_x.append(random.randint(0,7))
        rand_y.append(random.randint(0,7))
    f.write("\"x coords\" : {},\n".format(rand_x))
    f.write("\"y coords\" : {}\n".format(rand_y))
    f.write('}')

with open(fout, 'r') as f:
    print(''.join(f.readlines()))
