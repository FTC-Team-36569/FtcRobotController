package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro TeleOp Red")
public class PedroTeleOpRed extends PedroTeleOp{


    @Override
    protected String getTeamColor() {
        return "RED";
    }
}
