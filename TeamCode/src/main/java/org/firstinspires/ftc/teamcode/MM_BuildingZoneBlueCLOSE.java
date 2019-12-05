package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Blue", name = "Building Zone Blue Close")
public class MM_BuildingZoneBlueCLOSE extends LinearOpMode {
    private Robot robot = new Robot();

    MM_BuildingZoneTemplateBlue opmode = new MM_BuildingZoneTemplateBlue(MM_BuildingZoneTemplateBlue.ParkingPosition.CLOSE,
            MM_BuildingZoneTemplateBlue.AllianceColor.BLUE, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}