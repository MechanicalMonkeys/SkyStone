package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

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
    private double distanceToFoundation = 33; // distance to skybridge from bridge tape
    private int stepNumber = 1;

    public MM_LoadingZoneTemplate(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
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
        robot.gripBlock();

        // timer
        ElapsedTime timer = new ElapsedTime();
        // Detect skystone with camera
        timer.reset();
        int position;
        while (!this.opmode.isStarted()) {
            position = robot.detectSkystone(this.opmode);
            if (timer.time(TimeUnit.SECONDS) > 5) {
                skystonePos = Skystone.CENTER;
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

            // stuff to do after detection but before the state machine
            // wait for start
            this.opmode.waitForStart();


        /*this.opmode.telemetry.addData("Opmode", "Ready to start.");
        this.opmode.telemetry.update();*/

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
                robot.driveForwardDistance(6, 0.5, this.opmode);
                robot.bringArmDown(this.opmode);
                robot.rotateGripper(0.9);
                robot.releaseBlock(this.opmode);
                this.stepNumber++;
                break;
            case 2:
                if (robot.frontDistance.getDistance(DistanceUnit.INCH) == 0) {
                    this.driveWithoutDistanceSensor();
                } else {
                    robot.driveWithDistanceSensor(15, 0.25, this.robot.frontDistance, this.opmode);
                }
                Thread.sleep(0);
                switch (skystonePos) {
                    case LEFT:
                        distanceToBuildZone = 30 - colorCoefficient * 6;
                        // strafe to block
                        robot.strafeTime(-0.4, 1500, this.opmode);
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                    case CENTER:
                        distanceToBuildZone = 30;
                        robot.strafeTime(-0.4, 250, this.opmode);
                        break;
                    case RIGHT:
                        distanceToBuildZone = 30 + colorCoefficient * 6;
                        // strafe to block
                        robot.strafeTime(0.4, 1250, this.opmode);
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                }
                this.stepNumber++;
                break;
            case 3:
                Thread.sleep(0);
                robot.grabBlockAuto();
                this.stepNumber++;
                break;
            case 4:
                // back up
                robot.driveForwardDistance(0, -speed, this.opmode);
                // turn towards skybridge
                robot.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                robot.rotateGripper(1.0);
                // drive to foundation
                robot.driveForwardDistance(distanceToFoundation + distanceToBuildZone - 16, speed, this.opmode);
                this.stepNumber++;
                break;
            case 5:
                Thread.sleep(0);
                // drop block
                robot.releaseBlock(this.opmode);
                this.stepNumber++;
                break;
            case 6:
                Thread.sleep(0);
                // correct position - obviously its in the code
                robot.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                // drive to second Skystone
                robot.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 12, -speed, this.opmode);
                robot.rotateGripper(0.9);
                // turn
                robot.turnToGlobalPosition(0.25, 0, this.opmode);
                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.turnToGlobalPosition(0.25, -15 * colorCoefficient, this.opmode);
                    robot.driveForwardDistance(3, 0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 7:
                // go to block
                robot.driveWithDistanceSensor(13.5, 0.25, this.robot.frontDistance, this.opmode);
                this.stepNumber++;
                break;
            case 8:
                // grab block
                Thread.sleep(0);
                robot.grabBlockAuto();
                if ((this.skystonePos == Skystone.LEFT && this.allianceColor == AllianceColor.RED) ||
                        (this.skystonePos == Skystone.RIGHT && this.allianceColor == AllianceColor.BLUE)) {
                    robot.driveForwardDistance(6, -0.25, this.opmode);
                } else {
                    robot.driveForwardDistance(3, -0.25, this.opmode);
                }
                this.stepNumber++;
                break;
            case 9:
                // drive to foundation to drop the block off -  aight bruh im bouta head out now
                Thread.sleep(0);
                robot.driveForwardDistance(0, -speed, this.opmode);
                robot.turnToGlobalPosition(0.4, 90 * colorCoefficient, this.opmode);
                robot.rotateGripper(1.0);
                robot.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 3, speed, this.opmode);
                robot.releaseBlock(this.opmode);
                this.stepNumber++;
                break;
            case 10:
                // park
                Thread.sleep(0);
                switch(this.parkingPos) {
                    case FAR:
                        robot.strafeTime(-0.6 * colorCoefficient, 2300, this.opmode);
                        break;
                    case CLOSE:
                        robot.strafeTime(0.6 * colorCoefficient, 500, this.opmode);
                        break;
                }
                robot.driveForwardDistance(distanceToFoundation - 12, -speed, this.opmode);
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
        this.robot.driveForwardDistance(4, 0.3, this.opmode);
        Thread.sleep(250);
    }
}
