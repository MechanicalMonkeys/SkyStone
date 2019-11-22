package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Blue", name = "Building Zone Blue Close")
public class MM_BuildingZoneBlueCLOSE extends LinearOpMode {

    MM_BuildingZoneTemplate opmode = new MM_BuildingZoneTemplate(MM_BuildingZoneTemplate.ParkingPosition.CLOSE,
            MM_BuildingZoneTemplate.AllianceColor.BLUE);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}
