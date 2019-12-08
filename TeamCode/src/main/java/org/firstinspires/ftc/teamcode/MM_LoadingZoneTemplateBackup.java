package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

public class MM_LoadingZoneTemplateBackup {

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
    private double distanceToFoundation = 33; // distance to skybridge from bridge tape
    private int stepNumber = 1;

    public MM_LoadingZoneTemplateBackup(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
        this.parkingPos = parkingPos;
        this.allianceColor = color;
        this.opmode = opmode;
        this.robot = robot;
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

    public void runOpMode() throws InterruptedException {
        robot.init(this.opmode);
        robot.arm.gripBlock();

        // timer
        ElapsedTime timer = new ElapsedTime();
        // Detect skystone with camera
        timer.reset();
        int position;
        while (!this.opmode.isStarted()) {
            position = robot.detectSkystone();
            if (timer.time(TimeUnit.SECONDS) > 5) {
                break;
            } else if (position == -1) {
                skystonePos = Skystone.LEFT;
                break;
            } else if (position == 0) {
                skystonePos = Skystone.CENTER;
                break;
            } else if (position == 1) {
                skystonePos = Skystone.RIGHT;
                break;
            }
            this.opmode.telemetry.addData("Skystone Position", skystonePos);
            this.opmode.telemetry.update();
            this.opmode.idle();
        }

        if (!opmode.isStarted()) {
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
                    //this.opmode.idle();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                this.onRobotStopOrInterrupt();
            }
        }
    }

    private void linearOpmodeSteps() throws InterruptedException {
        switch (stepNumber) {
            case 1:
                this.opmode.telemetry.addData("Skystone Position", skystonePos);
                this.opmode.telemetry.update();
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
                robot.drive.driveForwardDistance(6, 0.5, this.opmode);
                robot.arm.bringArmDown(this.opmode);
                robot.arm.rotateGripper(0.9);
                robot.arm.releaseBlock();
                this.stepNumber++;
                break;
            case 2:
                if (robot.frontDistance.getDistance(DistanceUnit.INCH) == 0) {
                    this.driveWithoutDistanceSensor();
                } else {
                    robot.drive.driveWithDistanceSensor("drive",15.1, 0.25, this.robot.frontDistance, this.opmode);
                }
                Thread.sleep(0);
                switch (skystonePos) {
                    case LEFT:
                        distanceToGoBack = 30 - (colorCoefficient + 1) * 3;
                        distanceToBuildZone = 30 - colorCoefficient * 6;
                        // strafe to block
                        switch(allianceColor) {
                            case RED:
                                robot.drive.strafeTime(-0.4, 1530, this.opmode);
                                break;
                            case BLUE:
                                robot.drive.strafeTime(-0.4, 2000, this.opmode);
                                break;
                        }
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                    case CENTER:
                        distanceToGoBack = 30;
                        distanceToBuildZone = 30;
                        switch(allianceColor) {
                            case RED:
                                robot.drive.strafeTime(-0.4, 588, this.opmode);
                                break;
                            case BLUE:
                                robot.drive.strafeTime(-0.4, 750, this.opmode);
                                break;
                        }
                        break;
                    case RIGHT:
                        distanceToGoBack = 30 + (colorCoefficient - 1) * 3;
                        distanceToBuildZone = 30 + colorCoefficient * 6;
                        // strafe to block
                        switch(allianceColor) {
                            case RED:
                                robot.drive.strafeTime(0.4, 950, this.opmode);
                                break;
                            case BLUE:
                                robot.drive.strafeTime(0.4, 950, this.opmode);
                                break;
                        }
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                }
                robot.drive.driveWithDistanceSensor("drive",13, 0.25, this.robot.frontDistance, this.opmode);
                this.stepNumber++;
                break;
            case 3:
                Thread.sleep(0);
                robot.arm.grabBlockAuto();
                this.stepNumber++;
                break;
            case 4:
                // back up
                robot.drive.driveForwardDistance(0, -speed, this.opmode);
                // turn towards skybridge
                robot.drive.turnToGlobalPosition(0.35, 90 * colorCoefficient, this.opmode);
                robot.arm.rotateGripper(1.0);
                robot.drive.strafeTime(0.4 * colorCoefficient, 500, this.opmode);
                // drive to foundation
                robot.drive.driveForwardDistance(distanceToFoundation + distanceToBuildZone - 12, speed, this.opmode);
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
                robot.drive.driveForwardDistance(3, -speed, this.opmode);
                // correct position - obviously its in the code
                robot.drive.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                // drive to second Skystone
                robot.drive.driveForwardDistance(distanceToGoBack + distanceToFoundation + 12, -speed, this.opmode);
                robot.arm.rotateGripper(0.9);
                // turn
                robot.drive.turnToGlobalPosition(0.25, 0, this.opmode);
                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.drive.turnToGlobalPosition(0.25, -20 * colorCoefficient, this.opmode);
                    robot.drive.driveForwardDistance(6, 0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 7:
                // go to block
                if (!((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE))) {
                    robot.drive.driveWithDistanceSensor("drive",13, 0.25, this.robot.frontDistance, this.opmode);
                }
                this.stepNumber++;
                break;
            case 8:
                // grab block
                Thread.sleep(0);
                robot.arm.grabBlockAuto();
                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.drive.driveForwardDistance(6, -0.25, this.opmode);
                } else {
                    robot.drive.driveForwardDistance(3, -0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 9:
                // drive to foundation to drop the block off -  aight bruh im bouta head out now
                Thread.sleep(0);
                robot.drive.driveForwardDistance(0, -speed, this.opmode);
                robot.drive.turnToGlobalPosition(0.4, 90 * colorCoefficient, this.opmode);
                robot.arm.rotateGripper(1.0);
                robot.drive.strafeTime(0.4 * colorCoefficient, 750, this.opmode);
                robot.drive.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 9, speed, this.opmode);
                robot.arm.releaseBlock();
                this.stepNumber++;
                break;
            case 10:
                // park
                Thread.sleep(500);
                switch(this.parkingPos) {
                    case FAR:
                        robot.drive.strafeTime(-0.6 * colorCoefficient, 2300, this.opmode);
                        break;
                    case CLOSE:
                        robot.drive.strafeTime(0.6 * colorCoefficient, 700, this.opmode);
                        break;
                }
                robot.drive.driveForwardDistance(20, -speed, this.opmode);
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
