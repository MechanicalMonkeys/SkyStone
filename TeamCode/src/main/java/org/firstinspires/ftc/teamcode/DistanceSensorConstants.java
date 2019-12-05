package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface DistanceSensorConstants {
    DistanceUnit mm = DistanceUnit.MM;
    DistanceUnit cm = DistanceUnit.CM;
    DistanceUnit m = DistanceUnit.METER;
    DistanceUnit in = DistanceUnit.INCH;

    void init(OpMode opmode) throws InterruptedException;

    double convertUnits(DistanceUnit unit1, DistanceUnit unit2, double input);

    double[] readSensor(DistanceSensor distSense, int timesPerSecond, DistanceUnit unit) throws InterruptedException;

    double getDistFilter(double[] values);

    void outputValues(OpMode opmode, Robot robot);

    void driveWithDistanceSensor(double speed, double distance, DistanceUnit u, ModernRoboticsI2cRangeSensor ds);

    void driveWithDistanceSensor(double speed, double distance, double heading, DistanceUnit u, ModernRoboticsI2cRangeSensor ds);

    double readSensorWithConstraints(ModernRoboticsI2cRangeSensor sense, DistanceUnit unit, double lowerBound, double upperBound);
    
    double readSensorWithConstraints(ModernRoboticsI2cRangeSensor sense, DistanceUnit unit, double lowerBound, double upperBound, double errorReturn);
}
