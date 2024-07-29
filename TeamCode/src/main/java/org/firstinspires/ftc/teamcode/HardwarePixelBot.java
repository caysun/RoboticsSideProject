/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwarePixelBot {

    /* Declare OpMode members. */
    protected HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    protected BNO055IMU imu = null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx leftFront   = null;
    public DcMotorEx rightFront  = null;
    public DcMotorEx leftBack    = null;
    public DcMotorEx rightBack   = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    // Define a constructor
    public HardwarePixelBot() {

    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = hwMap.get(DcMotorEx.class, "FrontLeft");
        rightFront = hwMap.get(DcMotorEx.class, "FrontRight");
        leftBack   = hwMap.get(DcMotorEx.class, "RearLeft");
        rightBack  = hwMap.get(DcMotorEx.class, "RearRight");

        // Setting direction
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        setMotorPowers(0.0);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json";
        imu_params.loggingEnabled = true;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(imu_params);
    }

    /**
     * Sets all motor powers to specified values
     * @param lfPower
     * @param rfPower
     * @param lbPower
     * @param rbPower
     */
    public void setMotorPowers(double lfPower, double lbPower, double rfPower, double rbPower){
        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);
    }//setMotorPowers multiple parameters
    public void setMotorPowers(double power){
        setMotorPowers(power, power , power, power);
    } //setMotorPowers one parameter
    public void moveByTime(double lfPower, double lbPower, double rfPower, double rbPower,
                           ElapsedTime rt, double time, LinearOpMode opMode, Telemetry telemetry){
        setMotorPowers(lfPower, lbPower, rfPower, rbPower);
        rt.reset();
        while(opMode.opModeIsActive() && rt.milliseconds() < time){
            telemetry.addData("moveByTime Running", "for " + time/1000 + " seconds");
            telemetry.addData("Left Front Power: ", lfPower);
            telemetry.addData("Left Back Power: ", lbPower);
            telemetry.addData("Right Front Power: ", rfPower);
            telemetry.addData("Right Back Power: ", rbPower);
            telemetry.update();
        }
        setMotorPowers(0);
        opMode.sleep(1000);
    }// moveByTime all powers
    public void moveByTime(double power, ElapsedTime rt, double time, LinearOpMode opMode,
                           Telemetry telemetry) {
        moveByTime(power, power, power, power, rt, time, opMode, telemetry);
    }// moveByTime one power

    public void setDrivetrainMode(DcMotorEx.RunMode mode){
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
}// RobotHardware
