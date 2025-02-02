package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("StatementWithEmptyBody")
public class MM_BuildingZoneTemplateRed extends DistanceSensorMethods {
    enum ParkingPosition {FAR, CLOSE} // far or close to center
    private ParkingPosition parkingPos;
    enum AllianceColor {RED, BLUE} // color of alliance
    private AllianceColor allianceColor;
    private String stringColor;
    private int colorCoefficient;
    private double speed = 0.4;
    private LinearOpMode opmode;

    public MM_BuildingZoneTemplateRed(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
        this.opmode = opmode;
        //this.robot = robot;
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
        /*while (this.opmode.opModeIsActive() && readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 0.0, 255.0) < 60.0) {
            robot.drive.setDrivePower(-speed);
        }*/
        robot.drive.driveWithDistanceSensor("drive", 30, 0.3, robot.frontRange, this.opmode);

        // Strafe to center with foundation
        robot.drive.strafeTime(-speed, 2000, this.opmode);
        /*robot.drive.setStrafe(speed * colorCoefficient);
        Thread.sleep(2000);*/

        // Gyro correction
        imu.turnToPosition(0.2, initialHeading, imu.LEFT);

        // Go forward to touch foundation

        /*while (this.opmode.opModeIsActive() && readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 0.0, 255.0) < 80.0) {
            robot.drive.setDrivePower(-0.2);
        }*/
        robot.drive.driveWithDistanceSensor("drive", 31.5, 0.2, robot.frontRange, this.opmode);

        // Grab foundation
        robot.moveWaffleMover(true);

        // Pull foundation into building site
        //driveWithDistanceSensor(0.2, 15.0, DistanceUnit.CM, robot.frontRange);
        /*while (this.opmode.opModeIsActive() && readSensorWithConstraints(robot.frontRange, DistanceUnit.CM, 2.0, 255.0, 10.0) > 8.0) {
            robot.drive.setDrivePower(0.7);
        }*/
        robot.drive.driveWithDistanceSensor("drive", 3.15, 0.7, robot.frontRange, this.opmode);

        // Debug stuff
        robot.drive.stopDrive();
        Thread.sleep(1000);

        robot.moveWaffleMover(false);

        // Gyro correction
        imu.turnToPosition(0.2, initialHeading, imu.RIGHT);

        if (parkingPos == ParkingPosition.CLOSE) {
            /*robot.drive.setStrafe(-speed * colorCoefficient);
            while (this.opmode.opModeIsActive() && readSensorWithConstraints(robot.leftRange, DistanceUnit.CM, 0.0, 255.0) < 110.0);*/
            robot.drive.driveWithDistanceSensor("strafe", 43.31, -speed, robot.leftRange, this.opmode);

            imu.turnToPosition(0.2, initialHeading, imu.RIGHT);
            robot.drive.driveForwardDistance(22, -0.4, opmode);
            robot.moveWaffleMover(false);
        }
        // Strafe until see line
        //robot.drive.driveUntilColor("strafe", -speed * colorCoefficient, this.stringColor, opmode); // Michael's code
        robot.drive.strafeTime(-0.4 * colorCoefficient, 6000, this.opmode);
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