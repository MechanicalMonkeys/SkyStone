package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Red", name = "Building Zone Red Close")
public class MM_BuildingZoneRedCLOSE extends LinearOpMode {

    MM_BuildingZoneTemplate opmode = new MM_BuildingZoneTemplate(MM_BuildingZoneTemplate.ParkingPosition.CLOSE,
            MM_BuildingZoneTemplate.AllianceColor.RED);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}