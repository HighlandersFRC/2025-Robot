package frc.robot.tools.math;

import java.util.Arrays;

public class Spline {
    QuiticHermiteSpline x, y, theta;
    Waypoint[] waypoints;
    Derivatives[] xderivatives, yderivatives, thetaderivatives;

    /**
     * Constructs a spline from the given waypoints.
     *
     * @param waypoints An array of Waypoint objects representing the path.
     */
    public Spline(Waypoint[] waypoints) {
        Arrays.sort(waypoints, (w1, w2) -> Double.compare(w1.t, w2.t));
        this.waypoints = waypoints;
        optimizeRotation();
        xderivatives = new Derivatives[waypoints.length];
        yderivatives = new Derivatives[waypoints.length];
        thetaderivatives = new Derivatives[waypoints.length];

        for (int i = 0; i < waypoints.length; i++) {
            Waypoint w = waypoints[i];
            xderivatives[i] = new Derivatives(w.t, w.x, w.dx, w.d2x);
            yderivatives[i] = new Derivatives(w.t, w.y, w.dy, w.d2y);
            thetaderivatives[i] = new Derivatives(w.t, w.theta, w.dtheta, w.d2theta);
        }

        x = new QuiticHermiteSpline(xderivatives);
        y = new QuiticHermiteSpline(yderivatives);
        theta = new QuiticHermiteSpline(thetaderivatives);
    }

    /**
     * Optimizes the rotation of the waypoints to ensure smooth transitions.
     * This method adjusts the theta values of the waypoints to minimize abrupt
     * changes in direction.
     */
    void optimizeRotation() {
        for (int i = 1; i < waypoints.length; i++) {
            var p1 = waypoints[i - 1];
            waypoints[i].theta %= (2 * Math.PI);
            while (true) {
                if (waypoints[i].theta - p1.theta > Math.PI) {
                    waypoints[i].theta -= 2 * Math.PI;
                } else if (waypoints[i].theta - p1.theta < -Math.PI) {
                    waypoints[i].theta += 2 * Math.PI;
                } else {
                    break;
                }
            }
        }
    }

    /**
     * Interpolates the waypoint at a given time using the spline.
     *
     * @param time The time at which to interpolate the waypoint.
     * @return The interpolated Waypoint at the specified time.
     * @throws IndexOutOfBoundsException if the time is out of bounds for the spline
     *                                   segments.
     */
    public Waypoint interpolate(double time) {
        if (time < waypoints[0].t || time > waypoints[waypoints.length - 1].t) {
            throw new IndexOutOfBoundsException("Time out of bounds for spline segments");
        }

        int segmentIndex = 0;
        for (int i = 0; i < waypoints.length - 1; i++) {
            if (time < waypoints[i + 1].t) {
                segmentIndex = i;
                break;
            }
        }

        double dt = time - waypoints[segmentIndex].t;
        Derivatives x = this.x.interpolate(dt);
        Derivatives y = this.y.interpolate(dt);
        Derivatives theta = this.theta.interpolate(dt);

        return new Waypoint(time, x.d0, y.d0, theta.d0, x.d1, y.d1, theta.d1, x.d2, y.d2, theta.d2);
    }
}
