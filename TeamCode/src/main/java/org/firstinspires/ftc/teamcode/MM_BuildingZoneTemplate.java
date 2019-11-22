package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(group = "Blue", name = "Building Zone Blue Close")
public class MM_BuildingZoneTemplate extends LinearOpMode {
    private Robot robot = new Robot();
    enum ParkingPosition {FAR, CLOSE} // far or close to center
    private ParkingPosition parkingPos;
    enum AllianceColor {RED, BLUE} // color of alliance
    private AllianceColor allianceColor;
    private int colorCoefficient;
    private double speed = 0.4;

    public MM_BuildingZoneTemplate(ParkingPosition parkingPos, AllianceColor color) {
        this.parkingPos = parkingPos;
        this.allianceColor = color;
        switch(this.allianceColor) {
            case BLUE:
                this.colorCoefficient = 1;
                break;
            case RED:
                this.colorCoefficient = -1;
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        waitForStart();
        robot.driveForwardDistance(48, -speed, this);
        robot.strafeTime(speed * colorCoefficient, 2000);
        robot.strafeTime(-speed * colorCoefficient, 250);
        robot.driveForwardDistance(18, speed, this);
        robot.moveWaffleMover();
        robot.strafeTime(speed * colorCoefficient, 2000);
        // correction for strafe
        //robot.turnWithImu(0.25, -90, this);
        robot.driveForwardDistance(8, -0.25, this);
        robot.moveWaffleMover();
        robot.driveForwardDistance(34, 0.25, this);
        robot.moveWaffleMover();
        robot.driveUntilColor("strafe", -speed * colorCoefficient, "blue", this);
        if (parkingPos == ParkingPosition.CLOSE) {
            robot.driveForwardDistance(22, -0.4, this);
        }
    }
}
