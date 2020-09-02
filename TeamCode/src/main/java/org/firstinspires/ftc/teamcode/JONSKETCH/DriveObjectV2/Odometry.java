package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public interface Odometry {

    Point getPosition();
    void setPosition(Point newPos);
    double[] podPositions();
    double[] get();
    double getX();
    double getY();
    double getHeading();
    void beginTracking();
}
