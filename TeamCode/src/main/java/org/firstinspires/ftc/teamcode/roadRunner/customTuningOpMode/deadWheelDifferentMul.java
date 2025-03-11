package org.firstinspires.ftc.teamcode.roadRunner.customTuningOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadRunner.util.Encoder;

@TeleOp
@Disabled
public class deadWheelDifferentMul extends OpMode{

    Encoder r;
    Encoder l;

    Encoder y;
    int ri;
    int le;
    int ya;
    @Override
    public void init(){
        r = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        l = new Encoder(hardwareMap.get(DcMotorEx.class, "LSlide"));
        y = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));
        ri = r.getCurrentPosition();
        le = l.getCurrentPosition();
        ya = y.getCurrentPosition();
    }

    @Override
    public void loop() {
        telemetry.addData("right", ri-r.getCurrentPosition());
        telemetry.addData("left", le-l.getCurrentPosition());
        telemetry.addData("y", ya-y.getCurrentPosition());
    }
}
