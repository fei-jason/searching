import matplotlib.pyplot as plt

MAX = 50

class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.children = []
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        self.parent = None
        self.heuristic = 0

    def __str__(self):
        return f'({self.x}, {self.y})'
    
    def __eq__(self, other) :
        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False
        
    def to_tuple(self):
        return self.x, self.y
    
    def set_children(self, children):
        for child in children:
            child.parent = self
            self.children.append(child)

    def get_children(self):
        return self.children

def draw_board():
    # create a figure to draw the board
    fig = plt.figure(figsize=[8,8])
    # set the background color
    #fig.patch.set_facecolor((0.85,0.64,0.125))
    ax = fig.add_subplot(111)
    # turn off the axes
    ax.set_axis_off()
    return fig, ax
def draw_grids(ax):
    # draw the vertical lines
    for x in range(MAX):
        ax.plot([x, x], [0,MAX-1], color = '0.75', linestyle='dotted')
    # draw the horizontal lines
    for y in range(MAX):
        ax.plot([0, MAX-1], [y,y], color = '0.75', linestyle='dotted')
    ax.set_position([0,0.02,1,1])

def draw_point(ax, x, y):
    ax.plot(x,y,'o',markersize=4,
        markeredgecolor=(0,0,0),
        markerfacecolor='k',
        markeredgewidth=1)

def draw_source(ax, x, y):
    ax.plot(x,y,'o',markersize=4,
        markeredgecolor='b',
        markerfacecolor='b',
        markeredgewidth=1)

def draw_dest(ax, x, y):
    ax.plot(x,y,'o',markersize=4,
        markeredgecolor='r',
        markerfacecolor='r',
        markeredgewidth=1)

def draw_red_point(ax, x, y):
    ax.plot(x,y,'o',markersize=4,
        markeredgecolor='r',
        markerfacecolor='r',
        markeredgewidth=1)

def draw_green_point(ax, x, y):
    ax.plot(x,y,'o',markersize=4,
        markeredgecolor='g',
        markerfacecolor='g',
        markeredgewidth=1)

def draw_line(ax, xs, ys):
    ax.plot(xs, ys, color='k')

def draw_result_line(ax, xs, ys):
    ax.plot(xs, ys, color='r')

def draw_green_line(ax, xs, ys):
    ax.plot(xs, ys, color='g')
