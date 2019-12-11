package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS

public class DistanceSensorMethods implements DistanceSensorConstants {
    public Imu imu = new Imu();
    public Robot robot = new Robot();

    @Override
    public void init(OpMode opmode) throws InterruptedException {
        robot.init(opmode, false);
        imu.init(opmode, true);
    }

    @Override
    public double convertUnits(DistanceUnit unit1, DistanceUnit unit2, double input) { // A useful method
        double out = 0.0;
        switch (unit1) {
            case MM:
                switch (unit2) {
                    case MM: // mm to mm
                        out = input;
                        break;case CM: // mm to cm
                        out = input / 10;
                        break;case METER: // mm to m
                        out = input / 1000;
                        break;case INCH: // mm to in
                        out = input / 25.4;
                        break;
                }
                break;
                case CM:
                    switch (unit2) {
                        case MM: // cm to mm
                         out = input * 10;
                        break;case CM: // cm to cm
                    out = input;
                    break;
                    case METER: // cm to m
                        out = input / 100;
                        break;case INCH: // cm to in
                    out = input / 2.54;
                    break;
                }
                break;
                    case METER:
                        switch (unit2) {
                    case MM: // m to mm
                        out = input * 1000;
                        break;case CM: // m to cm
                    out = input * 100;
                    break;
                    case METER: // m to m
                        out = input;
                        break;case INCH: // m to in
                    out = input * 39.37;
                    break;
                }
                break;
                    case INCH:
                        switch (unit2) {
                    case MM: // in to mm
                        out = input * 25.4;
                        break;case CM: // in to cm
                    out = input / 2.54;
                    break;
                    case METER: // in to m
                        out = input / 39.37;
                        break;case INCH: // in to in
                    out = input;
                    break;
                }
                break;
        }
        return out;
    }

    @Override
    public double[] readSensor(DistanceSensor distSense, int timesPerSecond, DistanceUnit unit) throws InterruptedException{
        double[] returnVals = new double[timesPerSecond];

        for (int i = 0; i < timesPerSecond; i++) {
            returnVals[i] = distSense.getDistance(unit);
            Thread.sleep(1000/timesPerSecond);
        }
        return returnVals;
    }

    @Override
    public double getDistFilter(double[] values) { // Find the average of all values in an array of any length
        int len = values.length;
        double sum = 0.0;

        for (double num : values) {
            sum += num;
        }
        return sum / len;
    }

    @Override
    @SuppressLint("DefaultLocale")
    public void outputValues(OpMode opmode, Robot robot) {
        opmode.telemetry.addData("Forward Distance", String.format("%.01f cm", robot.frontDistance.getDistance(DistanceUnit.CM)));
        opmode.telemetry.addData("Back Distance", String.format("%.01f cm", robot.rearDistance.getDistance(DistanceUnit.CM)));
        opmode.telemetry.addData("Left Distance", String.format("%.01f cm", robot.leftDistance.getDistance(DistanceUnit.CM)));
        //opmode.telemetry.addData("Right Distance", String.format("%.01f cm", robot.rightDistance.getDistance(DistanceUnit.CM)));

        opmode.telemetry.update();
    }

    @Override
    public void driveWithDistanceSensor(double speed, double distance, DistanceUnit u, ModernRoboticsI2cRangeSensor ds) {
        if (speed > 0.0) {
            while (ds.getDistance(u) > distance) {
                robot.drive.setDrivePower(speed);
            }
        } else {
            while (ds.getDistance(u) < distance) {
                robot.drive.setDrivePower(speed);
            }
            imu.stopMotors();
        }
    }

    @Override
    public void driveWithDistanceSensor(double speed, double distance, double heading, DistanceUnit u, ModernRoboticsI2cRangeSensor ds) {
        if (speed > 0.0) {
            while (ds.getDistance(u) > distance) {
                imu.driveForwardCorrection(speed, heading, 3.0);
            }
        } else {
            while (ds.getDistance(u) < distance) {
                imu.driveForwardCorrection(speed, heading, 3.0);
            }
            imu.stopMotors();
        }
    }

    @Override
    public double readSensorWithConstraints(ModernRoboticsI2cRangeSensor sense, DistanceUnit unit, double lowerBound, double upperBound) {
        double testVal = sense.getDistance(unit);
        if (isInRange(testVal, lowerBound, upperBound)) {
            return testVal;
        }
        return -1.0;
    }

    @Override
    public double readSensorWithConstraints(ModernRoboticsI2cRangeSensor sense, DistanceUnit unit, double lowerBound, double upperBound, double errorReturn) {
        double testVal = sense.getDistance(unit);
        if (isInRange(testVal, lowerBound, upperBound)) {
            return testVal;
        }
        return errorReturn;
    }

    private boolean isInRange(double val, double lower, double upper) {
        return ((val <= upper) && (val >= lower));
    }
}
