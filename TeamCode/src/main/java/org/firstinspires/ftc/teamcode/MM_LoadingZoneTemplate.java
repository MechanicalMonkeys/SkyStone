package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    enum Skystone {LEFT, CENTER, RIGHT}
    private Skystone skystonePos = Skystone.LEFT;
    private double distanceToBuildZone; // distance to bridge tape from close edge of block
    private double distanceToFoundation = 33; // distance to skybridge from bridge tape
    private int stepNumber;

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
        robot.releaseBlock(this.opmode);
        // timer
        ElapsedTime timer = new ElapsedTime();
        // Detect skystone with camera that doesn't even work properly
        int position;
        timer.reset();
        while (true) {
            position = robot.detectSkystone(this.opmode);
            if (position == -1) {
                skystonePos = Skystone.LEFT;
                break;
            } else if (position == 0) {
                skystonePos = Skystone.CENTER;
                break;
            } else if (position == 1) {
                skystonePos = Skystone.RIGHT;
                break;
            }
            this.opmode.idle();
        }

        this.opmode.telemetry.addData("Opmode", "Ready to start.");
        this.opmode.telemetry.update();

        // wait for start
        this.opmode.waitForStart();

        this.stepNumber = 1;
        try {
            while (this.opmode.opModeIsActive()) {
                this.linearOpmodeSteps();
                this.opmode.idle();
            }
        } catch (InterruptedException e) {
            //Thread.currentThread().interrupt();
        } finally {
            this.onRobotStopOrInterrupt();
        }
    }

    private void linearOpmodeSteps() throws InterruptedException {
        switch (stepNumber) {
            case 1:
                if (robot.frontDistance.getDistance(DistanceUnit.INCH) == 0) {
                    this.driveWithoutDistanceSensor();
                } else {
                    robot.setDrivePower(0.25);
                    double distanceToBlock = robot.frontDistance.getDistance(DistanceUnit.INCH);
                    while (distanceToBlock > 15) {
                        this.opmode.telemetry.addData("Distance", robot.frontDistance.getDistance(DistanceUnit.INCH));
                        this.opmode.telemetry.update();
                        distanceToBlock = robot.frontDistance.getDistance(DistanceUnit.INCH);
                    }
                    robot.stopDrive();
                }
                this.stepNumber++;
                break;
            case 2:
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
                robot.bringArmDown(this.opmode);
                robot.rotateGripper(0.8);
                Thread.sleep(500);
                switch (skystonePos) {
                    case LEFT:
                        distanceToBuildZone = 30 - colorCoefficient * 6;
                        // strafe to block
                        robot.strafeTime(-0.4, 1500);
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;
                    case CENTER:
                        distanceToBuildZone = 30;
                        robot.strafeTime(-0.4, 250);
                        break;
                    case RIGHT:
                        distanceToBuildZone = 30 + colorCoefficient * 6;
                        // strafe to block
                        robot.strafeTime(0.4, 1250);
                        // correct for the strafe
                        //robot.turnRight(-0.25, 250);
                        break;

                }
                this.stepNumber++;
                break;
            case 3:
                Thread.sleep(500);
                robot.grabBlockAuto();
                this.stepNumber++;
                break;
            case 4:
                // back up
                robot.driveForwardDistance(8, -speed, this.opmode);
                // turn towards skybridge
                robot.turnWithImu(0.3, 90 * colorCoefficient, this.opmode);
                // drive to foundation
                robot.driveForwardDistance(distanceToFoundation + distanceToBuildZone - 12, speed, this.opmode);
                this.stepNumber++;
                break;
            case 5:
                Thread.sleep(500);
                // drop block
                robot.rotateGripper(0.5);
                Thread.sleep(500);
                robot.releaseBlock(this.opmode);
                Thread.sleep(500);
                robot.rotateGripper(1.0);
                this.stepNumber++;
                break;
            case 6:
                Thread.sleep(500);
                // correct position - obviously its in the code
                robot.turnToGlobalPosition(0.25, 90 * colorCoefficient, this.opmode);
                // drive to second Skystone
                robot.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 24, -speed, this.opmode);
                // turn
                robot.turnToGlobalPosition(0.3, 0, this.opmode);
                this.stepNumber++;
                break;
            case 7:
                robot.rotateGripper(0.8);
                // go to block
                robot.setDrivePower(0.25);
                double distanceToBlock2 = robot.frontDistance.getDistance(DistanceUnit.INCH);
                while (distanceToBlock2 > 15) {
                    this.opmode.telemetry.addData("Distance", robot.frontDistance.getDistance(DistanceUnit.INCH));
                    this.opmode.telemetry.update();
                    distanceToBlock2 = robot.frontDistance.getDistance(DistanceUnit.INCH);
                }
                robot.stopDrive();
                this.stepNumber++;
                break;
            case 8:
                // grab block
                Thread.sleep(500);
                robot.grabBlockAuto();
                this.stepNumber++;
                break;
            case 9:
                // drive to foundation to drop the block off -  aight bruh im bouta head out now
                Thread.sleep(500);
                robot.driveForwardDistance(15, -speed, this.opmode);
                robot.turnWithImu(0.3, 90 * colorCoefficient, this.opmode);
                robot.driveForwardDistance(distanceToBuildZone + distanceToFoundation + 18, speed, this.opmode);
                robot.releaseBlock(this.opmode);
                this.stepNumber++;
                break;
            case 10:
                // park
                Thread.sleep(500);
                robot.driveForwardDistance(10, -0.6, this.opmode);
                switch(this.parkingPos) {
                    case FAR:
                        robot.strafeTime(speed, 2800);
                        break;
                    case CLOSE:
                        robot.strafeTime(-speed, 1250);
                        break;
                }
                this.stepNumber++;
                break;
            case 11: // everything is stopped
                onRobotStopOrInterrupt();
                break;
        }
    }

    void onRobotStopOrInterrupt() {
        robot.stopEverything();
    }

    void driveWithoutDistanceSensor() throws InterruptedException {
        this.robot.driveForwardDistance(16, 0.3, this.opmode);
        Thread.sleep(500);
    }
}
