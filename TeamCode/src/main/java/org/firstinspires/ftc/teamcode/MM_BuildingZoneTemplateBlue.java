package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("StatementWithEmptyBody")
public class MM_BuildingZoneTemplateBlue extends DistanceSensorMethods {
    enum ParkingPosition {FAR, CLOSE} // far or close to center
    private ParkingPosition parkingPos;
    enum AllianceColor {RED, BLUE} // color of alliance
    private AllianceColor allianceColor;
    private String stringColor;
    private int colorCoefficient;
    private double speed = 0.4;
    private LinearOpMode opmode;

    public MM_BuildingZoneTemplateBlue(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
        this.opmode = opmode;
        this.robot = robot;
        this.parkingPos = parkingPos;
        this.allianceColor = color;
        switch(this.allianceColor) {
            case BLUE:
                this.stringColor = "blue";
                break;
            case RED:
                this.stringColor = "red";
                break;
        }
        switch(this.allianceColor) {
            case BLUE:
                this.colorCoefficient = 1;
                break;
            case RED:
                this.colorCoefficient = -1;
                break;
        }
    }

    public void run() throws InterruptedException {
        init(this.opmode);
        this.opmode.waitForStart();

        double initialHeading = imu.getHeading();

        // Prepare the manipulator
        robot.moveWaffleMover(false);

        // Go forward to prevent rubbing with wall
        while (readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 0.0, 255.0) < 60.0) {
            robot.drive.setDrivePower(-speed);
        }

        // Strafe to center with foundation
        robot.drive.setStrafe(speed * colorCoefficient);
        Thread.sleep(2500);

        // Gyro correction
        imu.turnToPosition(0.2, initialHeading, imu.RIGHT);

        // Go forward to touch foundation

        while (readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 0.0, 255.0) < 80.0) {
            robot.drive.setDrivePower(-0.2);
        }

        // Grab foundation
        robot.moveWaffleMover(true);

        // Pull foundation into building site
        //driveWithDistanceSensor(0.2, 15.0, DistanceUnit.CM, robot.frontRange);
        while (readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 2.0, 255.0, 10.0) > 8.0) {
            robot.drive.setDrivePower(0.3);
        }

        // Debug stuff
        robot.drive.stopDrive();
        Thread.sleep(1000);

        robot.moveWaffleMover(false);

        // Gyro correction
        imu.turnToPosition(0.2, initialHeading, imu.LEFT);

        if (parkingPos == ParkingPosition.CLOSE) {
            robot.drive.setStrafe(-speed * colorCoefficient);
            while (readSensorWithConstraints(robot.leftRange, DistanceUnit.CM, 0.0, 255.0) < 110.0);

            imu.turnToPosition(0.2, initialHeading, imu.LEFT);
            robot.drive.driveForwardDistance(18, -0.4, opmode);
            robot.moveWaffleMover(false);
        }

        // Strafe 80 cm, then drive forward 0.5 inch
        robot.drive.setStrafe(-speed * colorCoefficient);
        while (robot.leftRange.getDistance(DistanceUnit.CM) < 80.0);
        robot.drive.driveForwardDistance(0.5, -0.2, this.opmode);

        // Set up the fail-safe timer
        double nanoFactor = 1e+9;
        long startTime = System.nanoTime();
        long elapsed;
        double maxTime; // maximum time for strafe (seconds)

        if (parkingPos == ParkingPosition.FAR) {
            maxTime = 6.2;
        } else {
            maxTime = 2.0;
        }

        // Strafe until detected line
        while (robot.insideColor.blue() < 4000) {
            //robot.driveUntilColor("strafe", -speed * colorCoefficient, this.stringColor, opmode); // Michael's code
            // Check timer
            elapsed = System.nanoTime() - startTime;
            this.opmode.telemetry.addData("Time", elapsed);
            this.opmode.telemetry.update();

            if (elapsed > (maxTime * nanoFactor)) {
                break; // the line is not found, stop driving
            }
            robot.drive.setStrafe(-speed * colorCoefficient);
        }

        // Retract manipulator
        if (parkingPos == ParkingPosition.FAR) {
            robot.moveWaffleMover(false);
        }
    }

    public void runOpMode() {
        try {
            this.run();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        finally {
            robot.stopEverything();
        }
    }
}
