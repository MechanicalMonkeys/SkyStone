package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_LoadingZoneTemplate {

    enum ParkingPosition {FAR, CLOSE} // far or close to center
    private ParkingPosition parkingPos;
    enum AllianceColor {RED, BLUE} // color of alliance
    private AllianceColor allianceColor;
    private String stringColor;
    private int colorCoefficient;
    private double speed = 0.55;
    private LinearOpMode opmode;
    private Robot robot;
    enum Skystone {LEFT, CENTER, RIGHT, UNKNOWN}
    private Skystone skystonePos = Skystone.LEFT;
    private double distanceToBuildZone; // distance to bridge tape from close edge of block
    private double distanceToGoBack;
    private double distanceToFoundation = 35; // distance to skybridge from bridge tape
    private int stepNumber = 1;
    private DistanceSensor frontDistanceSensor;

    public MM_LoadingZoneTemplate(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
        this.parkingPos = parkingPos;
        this.allianceColor = color;
        this.opmode = opmode;
        this.robot = robot;
    }

    public void runOpMode() throws InterruptedException {
        robot.init(this.opmode/*, true*/, true);
        switch(this.allianceColor) {
            case BLUE:
                this.stringColor = "blue";
                this.colorCoefficient = 1;
                break;
            case RED:
                this.stringColor = "red";
                this.colorCoefficient = -1;
                break;
        }
        robot.arm.gripBlock();
        frontDistanceSensor = robot.frontDistance;

        int position;
        Skystone lastPosition = Skystone.UNKNOWN;
        while (!this.opmode.isStarted()) {
            position = robot.detectSkystone(this.opmode);
            //position = robot.detectImage();
            //position = 0;
            /*if (timer.time(TimeUnit.SECONDS) > 5) {
                break;
            } else */if (position == 0) {
                skystonePos = Skystone.LEFT;
            } else if (position == 1) {
                skystonePos = Skystone.CENTER;
            } else if (position == 2) {
                skystonePos = Skystone.RIGHT;
            } else {
                skystonePos = Skystone.UNKNOWN;
            }
            if (skystonePos != Skystone.UNKNOWN) {
                lastPosition = skystonePos;
            }
            this.opmode.telemetry.addData("Skystone Position", skystonePos + " " + lastPosition);
            this.opmode.telemetry.update();
            //this.opmode.idle();
        }

        //robot.closeTFLite();
        skystonePos = lastPosition;
        if (skystonePos == Skystone.UNKNOWN) {
            skystonePos = Skystone.CENTER;
        }

        //skystonePos = Skystone.LEFT;
        this.opmode.telemetry.addData("Skystone Position", skystonePos);
        this.opmode.telemetry.update();

        // stuff to do after detection but before the state machine
        // wait for start
        this.opmode.waitForStart();

        try {
            while (this.opmode.opModeIsActive()) {
                this.linearOpmodeSteps();
                this.opmode.idle();
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            this.onRobotStopOrInterrupt();
        }
    }

    private void linearOpmodeSteps() throws InterruptedException {
        switch (stepNumber) {
            case 1:
                /*robot.setDrivePower(speed);
                double distanceToBlock = robot.frontDistance.getDistance(DistanceUnit.INCH);
                while (distanceToBlock > 11.5) {
                    telemetry.addData("Distance", robot.frontDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                    distanceToBlock = robot.frontDistance.getDistance(DistanceUnit.INCH);
                }
                robot.stopDrive();
                 */
                // put arm down
                robot.drive.driveForwardDistance(4, 0.5, this.opmode);
                robot.arm.bringArmDown(this.opmode);
                robot.arm.rotateGripper(1.0);
                robot.arm.releaseBlock();
                this.stepNumber++;
                break;
            case 2:
                if (this.frontDistanceSensor.getDistance(DistanceUnit.INCH) == 0) {
                    this.driveWithoutDistanceSensor();
                } else {
                    robot.drive.driveWithDistanceSensor("drive",15.1, 0.25, this.frontDistanceSensor, this.opmode);
                }
                Thread.sleep(0);
                switch (skystonePos) {
                    case LEFT:
                        distanceToGoBack = 30 - (colorCoefficient + 1) * 3;
                        distanceToBuildZone = 30 - colorCoefficient * 6;
                        // strafe to block
                        switch(allianceColor) {
                            case RED:
                                robot.drive.driveWithDistanceSensor("strafe", 20, 0.4 * colorCoefficient, robot.leftRange, this.opmode);
                                break;
                            case BLUE:
                                robot.drive.driveWithDistanceSensor("strafe", 34, 0.4 * colorCoefficient, robot.leftRange, this.opmode);
                                break;
                        }

                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                    case CENTER:
                        distanceToGoBack = 30;
                        distanceToBuildZone = 30;
                        robot.drive.driveWithDistanceSensor("strafe",29.5, 0.4 * colorCoefficient, robot.leftRange, this.opmode);
                        break;
                    case RIGHT:
                        distanceToGoBack = 30 + (colorCoefficient - 1) * 3;
                        distanceToBuildZone = 30 + colorCoefficient * 6;
                        // strafe to block
                        robot.drive.driveWithDistanceSensor("strafe",36 - (colorCoefficient + 1) * 8, 0.4 * colorCoefficient, robot.leftRange, this.opmode);
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                }
                robot.drive.turnToGlobalPosition(0.25, 0, this.opmode);
                if (this.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 13 && this.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 15) {
                    robot.drive.driveWithDistanceSensor("drive", 13, 0.25, this.frontDistanceSensor, this.opmode);
                } else {
                    robot.drive.driveForwardDistance(2, 0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 3:
                Thread.sleep(0);
                robot.arm.grabBlockAuto();
                this.stepNumber++;
                break;
            case 4:
                // back up
                robot.drive.driveForwardDistance(5, -speed, this.opmode);
                // turn towards skybridge
                robot.drive.turnToGlobalPosition(0.35, 90 * colorCoefficient, this.opmode);
                robot.arm.rotateGripper(1.0);
                robot.drive.strafeTime(0.4 * colorCoefficient, 400, this.opmode);
                robot.drive.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                // drive to foundation
                robot.drive.driveForwardDistance(distanceToFoundation + distanceToBuildZone - 8, speed, this.opmode);
                this.stepNumber++;
                break;
            case 5:
                Thread.sleep(0);
                // drop block
                robot.arm.releaseBlock();
                this.stepNumber++;
                break;
            case 6:
                Thread.sleep(0);
                //robot.drive.driveForwardDistance(3, -speed, this.opmode);
                // correct position - obviously its in the code
                robot.drive.turnToGlobalPosition(0.15, 90 * colorCoefficient, this.opmode);
                robot.drive.strafeTime(-0.4 * colorCoefficient, 500, this.opmode);
                // drive to second Skystone
                switch(skystonePos) {
                    case LEFT:
                        switch(allianceColor) {
                            case RED:
                                robot.drive.driveForwardDistance(distanceToGoBack + distanceToFoundation + 15, -speed, this.opmode);
                                break;
                            case BLUE:
                                robot.drive.driveForwardDistance(distanceToGoBack + distanceToFoundation + 17, -speed, this.opmode);
                                robot.drive.strafeTime(-0.4, 500, this.opmode);
                                break;
                        }
                        break;
                    case CENTER:
                        robot.drive.driveForwardDistance(distanceToGoBack + distanceToFoundation + 17, -speed, this.opmode);
                        break;
                    case RIGHT:
                        robot.drive.driveForwardDistance(distanceToGoBack + distanceToFoundation + 19, -speed, this.opmode);
                }
                robot.arm.rotateGripper(1.0);
                this.stepNumber++;
                break;
            case 7:

                // turn
                robot.drive.turnToGlobalPosition(0.25, 0, this.opmode);


                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.drive.turnToGlobalPosition(0.25, -20 * colorCoefficient, this.opmode);
                    if (this.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 14) {
                        robot.drive.driveWithDistanceSensor("drive",14, 0.25, this.frontDistanceSensor, this.opmode);
                    }
                    robot.drive.driveForwardDistance(1, 0.25, this.opmode);
                }
                if (this.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 14) {
                    robot.drive.driveWithDistanceSensor("drive",14, 0.25, this.frontDistanceSensor, this.opmode);
                }
                this.stepNumber++;
                break;
            case 8:
                // grab block
                Thread.sleep(0);
                robot.arm.grabBlockAuto();
                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.drive.driveForwardDistance(4, -0.25, this.opmode);
                } else {
                    robot.drive.driveForwardDistance(3, -0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 9:
                // drive to foundation to drop the block off -  aight bruh im bouta head out now
                Thread.sleep(0);
                robot.drive.driveForwardDistance(3, -speed, this.opmode);
                robot.drive.turnToGlobalPosition(0.4, 90 * colorCoefficient, this.opmode);
                robot.arm.rotateGripper(1.0);
                robot.drive.strafeTime(0.4 * colorCoefficient, 400, this.opmode);
                robot.drive.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                robot.drive.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 9, speed, this.opmode);
                robot.arm.releaseBlock();
                this.stepNumber++;
                break;
            case 10:
                // park
                Thread.sleep(500);
                switch(this.parkingPos) {
                    case FAR:
                        robot.drive.driveForwardDistance(25, -speed, this.opmode);
                        robot.drive.strafeTime(-0.6 * colorCoefficient, 2000, this.opmode);
                        break;
                    case CLOSE:
                        //robot.drive.strafeTime(0.6 * colorCoefficient, 700, this.opmode);
                        robot.drive.strafeTime(-0.4 * colorCoefficient, 150, this.opmode);
                        robot.drive.driveForwardDistance(25, -speed, this.opmode);
                        break;
                }
                //robot.drive.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                Thread.sleep(500);
                this.stepNumber++;
                break;
            case 11: // everything is stopped - thanos c. 2019
                onRobotStopOrInterrupt();
                break;
        }
    }

    void onRobotStopOrInterrupt() {
        robot.stopEverything();
    }

    void driveWithoutDistanceSensor() throws InterruptedException {
        robot.drive.driveForwardDistance(4, 0.3, this.opmode);
        Thread.sleep(250);
    }
}
