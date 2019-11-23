package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_BuildingZoneTemplate {
    enum ParkingPosition {FAR, CLOSE} // far or close to center
    private ParkingPosition parkingPos;
    enum AllianceColor {RED, BLUE} // color of alliance
    private AllianceColor allianceColor;
    private String stringColor;
    private int colorCoefficient;
    private double speed = 0.4;
    private LinearOpMode opmode;
    private Robot robot;

    public MM_BuildingZoneTemplate(ParkingPosition parkingPos, AllianceColor color, LinearOpMode opmode, Robot robot) {
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
        robot.init(this.opmode);
        this.opmode.waitForStart();
        robot.driveForwardDistance(48, -speed, opmode);
        robot.strafeTime(speed * colorCoefficient, 2000);
        robot.strafeTime(-speed * colorCoefficient, 250);
        robot.driveForwardDistance(18, speed, opmode);
        robot.moveWaffleMover();
        robot.strafeTime(speed * colorCoefficient, 2000);
        // correction for strafe
        //robot.turnWithImu(0.25, -90, this);
        robot.driveForwardDistance(8, -0.25, opmode);
        robot.moveWaffleMover();
        robot.driveForwardDistance(34, 0.25, opmode);
        robot.moveWaffleMover();
        robot.driveUntilColor("strafe", -speed * colorCoefficient, this.stringColor, opmode);
        if (parkingPos == ParkingPosition.CLOSE) {
            robot.driveForwardDistance(22, -0.4, opmode);
        }
    }

    public void runOpMode() {
        try {
            this.run();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            robot.stopEverything();
        }
    }
}
