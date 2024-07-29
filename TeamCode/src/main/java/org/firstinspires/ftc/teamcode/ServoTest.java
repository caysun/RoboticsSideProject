package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo Test", group="Robot")
@Disabled
public class ServoTest extends LinearOpMode{
    RobotHardware robot = new RobotHardware();
    public Servo servo1 = null;

    public CRServo servo2 = null;

    public void runOpMode(){
        robot.init(hardwareMap);
        servo1 = hardwareMap.get(Servo.class, "servo1"); //TODO: Change name "servo1" to proper name
        servo2 = hardwareMap.get(CRServo.class, "servo2"); //TODO: Change name "servo2" to proper name

        while(opModeIsActive()){
            if(gamepad1.cross){
                servo1.setPosition(1.0);
            }

            while(gamepad1.square){
                servo2.setPower(1.0);
            }
            servo2.setPower(0.0);

            while(gamepad1.circle){
                servo2.setPower(-1.0);
            }
            servo2.setPower(0.0);
        }
    }

}
