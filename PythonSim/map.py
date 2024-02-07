import math

from lib.settings import lane_width, turn_radius, arm_len, NS_lane_count, EW_lane_count, veh_gen_rule_table

class Map:
    _instance = None

    @staticmethod 
    def getInstance():
        if Map._instance == None:
            Map()
        return Map._instance

    def __init__(self):
        if Map._instance != None:
            raise Exception("This class is a singleton, but more than one objects are created.")
        else:
            Map._instance = self

        self.lw = lane_width
        self.tr = turn_radius
        self.al = arm_len
        self.NSl = NS_lane_count
        self.EWl = EW_lane_count

        self.ju_track_table = self.gen_ju_track_table()
        self.ex_arm_table = {
            'Nl': 'E', 'Nt': 'S', 'Nr': 'W', 
            'Sl': 'W', 'St': 'N', 'Sr': 'E', 
            'El': 'S', 'Et': 'W', 'Er': 'N',
            'Wl': 'N', 'Wt': 'E', 'Wr': 'S'
        }
    
    def get_ex_arm(self, ap_arm, turn_dir):
        return self.ex_arm_table[str(ap_arm) + str(turn_dir)]
    
    def get_ju_track(self, ap_arm, turn_dir, ap_lane, ex_lane):
        return self.ju_track_table[str(ap_arm) + str(turn_dir) + str(ap_lane) + str(ex_lane)]

    def gen_ju_track_table(self):
        '''generate vehicle tracks in junction area.'''
        x1 = self.lw * self.NSl
        x2 = x1 + self.tr 
        y1 = self.lw * self.EWl
        y2 = y1 + self.tr

        ju_track_table = {}
        # Turn situation
        for i in range(self.EWl): # from/to something
            for j in range(self.NSl): # from/to north and south
                x4 = self.lw / 2 + self.lw * j   # The absolute value of the x coordinate of the beginning/end position
                y4 = self.lw / 2 + self.lw * i   # Absolute value of y coordinate of the beginning/end position
                # From east-west to north-south
                ju_track_table['Wl'+str(i)+str(j)] = self.gen_ju_track(xa=-x2, ya=y4, xb=x4, yb=-y2, ap_arm='W', dir='l')
                ju_track_table['Wr'+str(i)+str(j)] = self.gen_ju_track(xa=-x2, ya=y4, xb=-x4, yb=y2, ap_arm='W', dir='r')
                ju_track_table['El'+str(i)+str(j)] = self.gen_ju_track(xa=x2, ya=-y4, xb=-x4, yb=y2, ap_arm='E', dir='l')
                ju_track_table['Er'+str(i)+str(j)] = self.gen_ju_track(xa=x2, ya=-y4, xb=x4, yb=-y2, ap_arm='E', dir='r')
                # From north-south to east-west
                ju_track_table['Nl'+str(j)+str(i)] = self.gen_ju_track(xa=-x4, ya=-y2, xb=x2, yb=y4, ap_arm='N', dir='l')
                ju_track_table['Nr'+str(j)+str(i)] = self.gen_ju_track(xa=-x4, ya=-y2, xb=-x2, yb=-y4, ap_arm='N', dir='r')
                ju_track_table['Sl'+str(j)+str(i)] = self.gen_ju_track(xa=x4, ya=y2, xb=-x2, yb=-y4, ap_arm='S', dir='l')
                ju_track_table['Sr'+str(j)+str(i)] = self.gen_ju_track(xa=x4, ya=y2, xb=x2, yb=y4, ap_arm='S', dir='r')
        # Go straight from east to west
        for i in range(self.EWl): # From i
            for j in range(self.EWl): # Go to j range
                y_start = self.lw / 2 + self.lw * i # Absolute value of y coordinate of starting position
                y_end = self.lw / 2 + self.lw * j   # Absolute value of the y coordinate of the end position
                ju_track_table['Wt'+str(i)+str(j)] = self.gen_ju_track(xa=-x2, ya=y_start, xb=x2, yb=y_end, ap_arm='W', dir='t')
                ju_track_table['Et'+str(i)+str(j)] = self.gen_ju_track(xa=x2, ya=-y_start, xb=-x2, yb=-y_end, ap_arm='E', dir='t')
        # North-South straight line
        for i in range(self.NSl): # From i
            for j in range(self.NSl): # Go to j range
                x_start = self.lw / 2 + self.lw * i # The absolute value of the x coordinate of the starting position
                x_end = self.lw / 2 + self.lw * j   # The absolute value of the x coordinate of the end position
                ju_track_table['Nt'+str(i)+str(j)] = self.gen_ju_track(xa=-x_start, ya=-y2, xb=-x_end, yb=y2, ap_arm='N', dir='t')
                ju_track_table['St'+str(i)+str(j)] = self.gen_ju_track(xa=x_start, ya=y2, xb=x_end, yb=-y2, ap_arm='S', dir='t')
        table_filtered = {}
        for dir, shape_list in ju_track_table.items():
            if veh_gen_rule_table[dir[0:2]][int(dir[2])] != 0: # Complies with vehicle generation rules
                table_filtered[dir] = shape_list

        # print(table_filtered)
        return table_filtered

    def gen_ju_track(self, xa, ya, xb, yb, ap_arm, dir):
        '''
        Generate the starting point is A(xa, ya), the end point is B(xb, yb), and the trajectory of the vehicle from the ap_arm direction in the junction zone
            A - starting point
            B - end point
            ap_arm - approach arm, 'NSEW'
            dir - direction, 'lrt'
        '''
        # Turn, the trajectory consists of tangent straight lines and circular curves
        if dir == 'l' or dir == 'r':
            # First determine the angle range of the circular curve according to the direction of the turn. The angle regulations are the same as in mathematics.
            if ap_arm == 'N': 
                if dir == 'l':
                    angle = (180, 270)
                else:
                    angle = (360, 270)
            elif ap_arm == 'S': 
                if dir == 'l':
                    angle = (0, 90)
                else:
                    angle = (180, 90)
            elif ap_arm == 'E':
                if dir == 'l':
                    angle = (90, 180)
                else:
                    angle = (270, 180)
            elif ap_arm == 'W':
                if dir == 'l': 
                    angle = (270, 360)
                else:
                    angle = (90, 0)

            x_diff = abs(xb - xa)
            y_diff = abs(yb - ya)

            # For a car that turns from EW to NS, the straight segment appears near the end point when x_diff is small, and when y_diff is small, it appears near the starting point.
            if ap_arm == 'E' or ap_arm == 'W':
                # Only one circular curve
                if abs(x_diff - y_diff) < 1e-5:
                    center = [xa, yb]
                    return [
                        ['arc', (xa, ya), (xb, yb), center, x_diff, angle] # Circular curve, starting point xy, end point xy, center, radius, angle start and end
                    ]
                # A - Straight line - Circular curve - B
                elif x_diff > y_diff:
                    if xa < xb:
                        junc = [xb - y_diff, ya]
                    else:
                        junc = [xb + y_diff, ya]
                    center = [junc[0], yb]
                    r = y_diff
                    return [
                        ['line', (xa, ya), junc], # Straight line, starting point xy, end point xy
                        ['arc', junc, (xb, yb), center, r, angle] # Circular curve, starting point xy, end point xy, center point, radius, angle start and end
                    ]
                # A - Circular curve - Straight line - B
                else:
                    if ya < yb:
                        junc = [xb, ya + x_diff]
                    else:
                        junc = [xb, ya - x_diff]
                    center = [xa, junc[1]]
                    r = x_diff
                    return [
                        ['arc', (xa, ya), junc, center, r, angle], # Circular curve, starting point xy, end point xy, center point, radius, angle start and end
                        ['line', junc, (xb, yb)] # Straight line, starting point xy, end point xy
                    ]
            # For a car that turns NS to EW, the straight segment appears near the starting point when x_diff is small, and near the end when y_diff is small.
            elif ap_arm == 'N' or ap_arm == 'S':
                # Only one circular curve
                if abs(x_diff - y_diff) < 1e-5:
                    center = [xb,ya]
                    return [
                        ['arc', (xa, ya), (xb, yb), center, x_diff, angle] # Circular curve, starting point xy, end point xy, center, radius, angle start and end
                    ]
                # A - Circular curve - Straight line - B
                elif x_diff > y_diff:
                    if xa < xb:
                        junc = [xa + y_diff, yb]
                    else:
                        junc = [xa - y_diff, yb]
                    center = [junc[0], ya]
                    r = y_diff
                    return [
                        ['arc', (xa, ya), junc, center, r, angle], # Circular curve, starting point xy, end point xy, center point, radius, angle start and end
                        ['line', junc, (xb, yb)] # Straight line, starting point xy, end point xy
                    ]
                # A - Straight line - Circular curve - B
                else:
                    if ya < yb:
                        junc = [xa, yb - x_diff]
                    else:
                        junc = [xa, yb + x_diff]
                    center = [xb, junc[1]]
                    r = x_diff
                    return [
                        ['line', (xa, ya), junc], # Straight line, starting point xy, end point xy
                        ['arc', junc, (xb, yb), center, r, angle] # Circular curve, starting point xy, end point xy, center point, radius, angle start and end
                    ]
        # Go straight, the trajectory consists of a straight line or two tangent circular curves
        elif dir == 't':
            # Go straight to the opposite lane and follow a straight path
            if abs(xa - xb) < 1e-5 or abs(ya - yb) < 1e-5:
                return [
                    ['line', (xa, ya), (xb, yb)]
                ]
            # The lane does not correspond and is composed of two circular curves
            elif ap_arm == 'N' or ap_arm == 'S':
                half_x_diff = abs(xa - xb) / 2
                half_y_diff = abs(ya - yb) / 2
                r = (half_x_diff ** 2 + half_y_diff ** 2) / (2 * half_x_diff)
                alpha = math.asin(half_y_diff / r) / math.pi * 180
                if ap_arm == 'S' and xb > xa:
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa + r, ya), r, (180, 180 - alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb - r, yb), r, (360 - alpha, 360)]
                    ]
                elif ap_arm == 'S' and xb < xa:
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa - r, ya), r, (0, alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb + r, yb), r, (180 + alpha, 180)]
                    ]
                elif ap_arm == 'N' and xb > xa: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa + r, ya), r, (180, 180 + alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb - r, yb), r, (alpha, 0)]
                    ]
                elif ap_arm == 'N' and xb < xa: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa - r, ya), r, (360, 360 - alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb + r, yb), r, (180 - alpha, 180)]
                    ]
            elif ap_arm == 'E' or ap_arm == 'W':
                half_x_diff = abs(xa - xb) / 2
                half_y_diff = abs(ya - yb) / 2
                r = (half_x_diff ** 2 + half_y_diff ** 2) / (2 * half_y_diff)
                alpha = math.asin(half_x_diff / r) / math.pi * 180
                if ap_arm == 'E' and ya > yb: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa, ya - r), r, (270, 270 - alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb, yb + r), r, (90 - alpha, 90)]
                    ]
                elif ap_arm == 'E' and ya < yb: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa, ya + r), r, (90, 90 + alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb, yb - r), r, (270 + alpha, 270)]
                    ]
                elif ap_arm == 'W' and ya > yb: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa, ya - r), r, (270, 270 + alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb, yb + r), r, (90 + alpha, 90)]
                    ]
                elif ap_arm == 'W' and ya < yb: 
                    return [
                        ['arc', (xa, ya), ((xa+xb)/2, (ya+yb)/2), (xa, ya + r), r, (90, 90 - alpha)],
                        ['arc', ((xa+xb)/2, (ya+yb)/2), (xb, yb), (xb, yb - r), r, (270 - alpha, 270)]
                    ]

class Track:
    def __init__(self, ap_arm, ap_lane, turn_dir):
        # approach arm ('NSEW') and lane 
        self.ap_arm = ap_arm
        self.ap_lane = ap_lane

        # turning direction ('lrt') and track in junction area
        self.turn_dir = turn_dir
        self.ju_track = None

        # exit arm and lane
        self.ex_arm = Map.getInstance().get_ex_arm(ap_arm, turn_dir)
        self.ex_lane = None
        
        self.is_complete = False

        self.ju_shape_end_x = []
    
    def confirm_ex_lane(self, ex_lane):
        self.ex_lane = ex_lane
        self.ju_track = Map.getInstance().get_ju_track(self.ap_arm, self.turn_dir, self.ap_lane, self.ex_lane)
        self.ju_shape_end_x = Track.cal_ju_shape_end_x(self.ju_track)
        self.is_complete = True

    @staticmethod
    def cal_ju_shape_end_x(ju_track):
        sum = 0
        ju_shape_end_x = []
        for seg in ju_track:
            if seg[0] == 'line': 
                sum += math.sqrt((seg[1][0] - seg[2][0]) ** 2 + (seg[1][1] - seg[2][1]) ** 2)
            else: 
                sum += seg[4] * abs(seg[5][0] - seg[5][1]) * math.pi / 180
            ju_shape_end_x.append(sum)
        return ju_shape_end_x


