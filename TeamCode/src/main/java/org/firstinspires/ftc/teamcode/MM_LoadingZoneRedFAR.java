package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group =  "Red", name = "Loading Zone Red Far")
public class MM_LoadingZoneRedFAR extends LinearOpMode {

    private Robot robot = new Robot();
    MM_LoadingZoneTemplate opmode = new MM_LoadingZoneTemplate(MM_LoadingZoneTemplate.ParkingPosition.FAR,
            MM_LoadingZoneTemplate.AllianceColor.RED, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}