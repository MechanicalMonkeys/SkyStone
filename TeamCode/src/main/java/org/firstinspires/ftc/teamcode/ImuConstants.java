package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public interface ImuConstants {
    int LEFT = 1;
    int RIGHT = -1;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    void init(OpMode opmode, boolean initmotors);

    double getHeading();

    void driveForwardCorrection(double power, double initialAngle, double delta);

    void turnToPosition(double power, double angle, double direction);

    void turnDegrees(double power, double angle, int direction);

    void stopMotors();
}
