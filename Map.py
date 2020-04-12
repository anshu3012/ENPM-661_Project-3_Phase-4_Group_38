import matplotlib.pyplot as plt
import math

clr = 0.354

plt.axes()


def map():

    centres = [(0, 0), (2, 3), (-2, -3), (2, -3)]

    for centre in centres:
        circle_with_clear = plt.Circle(centre, radius=1 + clr, fc='r')
        plt.gca().add_patch(circle_with_clear)

        circle = plt.Circle(centre, radius=1, fc='g')
        plt.gca().add_patch(circle)

    corners = [(-4.75, -0.25), (-2.75, 3.25), (3.25, -0.25)]

    for corner in corners:

        rect_with_clr = plt.Rectangle((corner[0] - clr, corner[1] - clr), 1.5 + 2 * clr, 1.5 + 2 * clr, fc='r')
        plt.gca().add_patch(rect_with_clr)

        rect = plt.Rectangle(corner, 1.5, 1.5, fc='g')
        plt.gca().add_patch(rect)

    line1 = plt.Line2D((-5.1, -5.1), (5.1, -5.1), lw=0.2 + clr)
    plt.gca().add_line(line1)

    line2 = plt.Line2D((5.1, -5.1), (5.1, 5.1), lw=0.2 + clr)
    plt.gca().add_line(line2)

    line3 = plt.Line2D((5.1, 5.1), (-5.1, 5.1), lw=0.2 + clr)
    plt.gca().add_line(line3)

    line4 = plt.Line2D((-5.1, 5.1), (-5.1, -5.1), lw=0.2 + clr)
    plt.gca().add_line(line4)

    plt.axis('scaled')
    plt.show()


def check_obstacle(x, y):

    x = x - 5
    y = y - 5

    centres = [(0, 0), (2, 3), (-2, -3), (2, -3)]
    corners = [(-4.75, -0.25), (-2.75, 3.25), (3.25, -0.25)]
    for centre in centres:
        d = math.sqrt(((x - centre[0])**2) + ((y - centre[1])**2))
        if d < 1 + clr:
            print("Object in a circle")

        else:
            print("Clear to proceed")

    for corner in corners:
        if x > (corner[0] - clr) and y > (corner[1] - clr) and y < (corner[1] + clr + 0.5) and x < (corner[0] + 1.5 + clr):
            print("In rectange")

map()
check_obstacle(1, 5)
