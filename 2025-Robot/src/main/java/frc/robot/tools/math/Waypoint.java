package frc.robot.tools.math;

public class Waypoint {
    public double t, x, y, theta, dx, dy, dtheta, d2x, d2y, d2theta;

    public Waypoint(double t, double x, double y, double theta, double dx, double dy, double dtheta, double d2x,
            double d2y,
            double d2theta) {
        this.t = t;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
        this.d2x = d2x;
        this.d2y = d2y;
        this.d2theta = d2theta;
    }

    @Override
    public String toString() {
        return "Waypoint{" +
                "t=" + t +
                ", x=" + x +
                ", y=" + y +
                ", theta=" + theta +
                ", dx=" + dx +
                ", dy=" + dy +
                ", dtheta=" + dtheta +
                ", d2x=" + d2x +
                ", d2y=" + d2y +
                ", d2theta=" + d2theta +
                '}';
    }
}
