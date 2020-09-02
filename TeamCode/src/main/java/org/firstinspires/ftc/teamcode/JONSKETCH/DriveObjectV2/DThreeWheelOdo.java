package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class DThreeWheelOdo implements Odometry {

    public DOdometryPod[] odoPods;

    double x, y, heading, inchesPerTick;

    //First val is trackwith, second value is forward offset, third is auxiliary trackwidth
    double[] dimensions;
    DOThread thread = new NullThread();

    ValueStorage vals;

    public DThreeWheelOdo(double x, double y, double heading, ValueStorage vals, DOdometryPod[] odoPods, double inchesPerTick, double... dimensions){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.odoPods = odoPods;
        this.vals = vals;
        this.dimensions = dimensions;
        this.inchesPerTick = inchesPerTick;
    }

    public double[] podPositions() {
        return new double[]{odoPods[0].get()[0], odoPods[1].get()[0], odoPods[2].get()[0]};
    }

    public double[] get() {
        return new double[]{x, y, heading};
    }

    public Point getPosition() {
        return new Point(x, y, heading);
    }

    public void setPosition(Point newPos) {
        x = newPos.x;
        y = newPos.y;
        heading = newPos.heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void beginTracking() {
        thread = new OdometryThread(this, vals, dimensions, x, y, heading, inchesPerTick);
        thread.start();
    }
}
