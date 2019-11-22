package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Blue", name = "Building Zone Blue Far")
public class MM_BuildingZoneBlueFAR extends LinearOpMode {

    MM_BuildingZoneTemplate opmode = new MM_BuildingZoneTemplate(MM_BuildingZoneTemplate.ParkingPosition.FAR,
            MM_BuildingZoneTemplate.AllianceColor.BLUE);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}