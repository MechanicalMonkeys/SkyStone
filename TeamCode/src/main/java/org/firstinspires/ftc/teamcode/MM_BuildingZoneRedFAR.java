package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Red", name = "Building Zone Red Far")
public class MM_BuildingZoneRedFAR extends LinearOpMode {
    private Robot robot = new Robot();

    MM_BuildingZoneTemplateRed opmode = new MM_BuildingZoneTemplateRed(MM_BuildingZoneTemplateRed.ParkingPosition.FAR,
            MM_BuildingZoneTemplateRed.AllianceColor.RED, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}