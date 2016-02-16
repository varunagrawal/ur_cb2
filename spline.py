import numpy as np
import scipy.interpolate as sci
from scipy.misc import comb

class Spline():

    def __init__(self, smoothness=3, order=3, nest=-1):
        self.smoothness = smoothness
        self.order = order
        self.nest = nest    # Estimate of number of knots needed

        
    def bernstein_polynomial(self, i, n, t):
        """
        The Bernstein polynomial of n, i as a function of t
        """        
        return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


    def bezier_curve(self, points, n=1000):
        """
        Given a set of control points, return the bezier curve defined by the control points.
        
        @points: A list of lists that should have atleast 3 points, the current point, the current goal and the next point. These are the control points needed for the bezier curve.
        @n: The number of points to interpolate.
        
        See http://processingjs.nihongoresources.com/bezierinfo/
        """

        n_points = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])
        zPoints = np.array([p[2] for p in points])

        t = np.linspace(0.0, 1.0, n)

        polynomial_array = np.array([ self.bernstein_polynomial(i, n_points-1, t) for i in range(0, n_points)  ])

        print(polynomial_array.shape)
        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)
        zvals = np.dot(zPoints, polynomial_array)

        return xvals, yvals, zvals


    def simple_curve(self, points, n):
        """
        Get a spline curve that passes through all the points specified in points.
        @points: A list of lists that should have atleast 3 points, the current point, the current goal and the next point.
        @n: The number of points to interpolate.
        """
        # get the knot points
        tckp, u = sci.splprep([points[0], points[1], points[2]], s=self.smoothness, k=self.order, nest=self.nest)
        tckp, u = sci.splprep([points[0], points[1], points[2]], s=self.smoothness, k=self.order, nest=self.nest)
        #tck, fp, ler, msg = sci.bisplrep(path[0], path[1], path[2], kx=self.order, ky=self.order, s=self.smoothness)
        
        # evaluate the spline, including interpolated points
        xnew, ynew, znew = sci.splev(np.linspace(0, 1, n), tckp)
        
        return xnew, ynew, znew
        

    def get_path(self, cur_pt, goal_pt, next_pt, n=10, bezier=True):
        """
        Get spline path given 3 points
        @cur_pt: The current point of the robot
        @goal_pt: The current goal to which the robot is trying to move
        @next_pt: The point the robot will move to next after the goal point
        """
        path = []
        for i in range(len(cur_pt)):
            path.append([cur_pt[i], goal_pt[i], next_pt[i]])
            
        if bezier:
            xnew, ynew, znew = self.bezier_curve(path, n)            
        else:
            xnew, ynew, znew = self.simple_curve(path, n)

        return xnew, ynew, znew


if __name__ == "__main__":
    
    cur_pt = [5, 7, 13]
    goal_pt = [1, 9, 0]
    next_pt = [15, 2, 11]

    spline = Spline(order=2)
    path = spline.get_path(cur_pt, goal_pt, next_pt, n=30, bezier=True)
    
    for axis in range(len(path)):
        print(path[axis])

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = Axes3D(fig, elev=-150, azim=110)
    ax.plot(path[0], path[1], path[2])
    plt.show()
