from pypaths import astar

RESOLUTION = 4
MARGIN = 6


bad_points = set()
for i in range(16, 25, 4):
    for j in range(16,25,4):
        bad_points.add((i,j))

def custom_neighbors( height, width ):
    def func( coord ):
        neighbor_list = [(coord[0]+i,coord[1]+j) 
                            for i in range(-RESOLUTION, RESOLUTION+1, RESOLUTION)
                            for j in range(-RESOLUTION, RESOLUTION+1, RESOLUTION) ]
        return [ c for c in neighbor_list
                 if c != coord
                 and c not in bad_points
                 and c[0] >= MARGIN and c[0] <= width-MARGIN
                 and c[1] >= MARGIN and c[1] <= height-MARGIN ]
    return func

def avg(p1, p2):
    return ((p1[0] + p2[0])//2 , (p1[1] + p2[1])//2)

def optimize_path(path):
    #optimized_path = [path[0]]
    optimized_path = []
    for i, p in enumerate(path[1:-1]):
        if p != avg(path[i], path[i+2]):
            optimized_path.append(p)
            pass
    #optimized_path.append(path[-1])
    return optimized_path
    
finder = astar.pathfinder(neighbors=custom_neighbors(12*8, 12*8),
                    distance=astar.absolute_distance,
                    cost=astar.fixed_cost(1))

def get_path(from_pt, to_pt):
    from_pt = tuple([int(round(a/RESOLUTION)*RESOLUTION) for a in from_pt])
    to_pt_approx = tuple([int(round(a/RESOLUTION)*RESOLUTION) for a in to_pt])
    path = finder(from_pt, to_pt_approx)[1]
    path = optimize_path(path) 
    path.append(to_pt)
    return path

print(get_path((11,11), (31,36)))
    
