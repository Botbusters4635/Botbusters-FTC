package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.Arm
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.EctoOpMode
import org.firstinspires.ftc.teamcode.controllers.chassis.Chassis
import org.firstinspires.ftc.teamcode.controllers.chassis.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.chassis.PositionChassis


@TeleOp(name = "TeleOp")
class TeleOp : EctoOpMode() {
    val chassis = Chassis()
    val arm = SynchronizedArm()
//    val arm = Arm()

    val intake = Intake()
    val trayHolder = TrayHolder()

    var targetHeading = 0.0
    val maxTargetHeadingRate = 150.0

    var usePID = false

    var currentSpeedLimiter = 0.0


    override fun init_impl() {
        controllers.add(chassis)
        controllers.add(arm)
        controllers.add(intake)
        controllers.add(trayHolder)
    }

    override fun startMode() {
        arm.moveToPosition(ArmPosition.PASSBRIDGE)
    }

    override fun update(timeStep: Double) {

        if (gamepad1.right_trigger > 0.3) {
            currentSpeedLimiter = 0.9
        } else if (gamepad1.left_trigger > 0.3) {
            currentSpeedLimiter = 0.6
        } else {
            currentSpeedLimiter = 0.0
        }


        if(gamepad1.y){
            chassis.pidActive = true
        }

        if(gamepad1.b){
            chassis.pidActive = false
        }

        if(chassis.pidActive){
            val desiredChange = -gamepad1.right_stick_x.toDouble()
            targetHeading += (desiredChange * maxTargetHeadingRate * (1.0 - currentSpeedLimiter)) * timeStep



            if (targetHeading > 180.0) {
                targetHeading -= 360
            }

            if (targetHeading < -180.0) {
                targetHeading += 360
            }


            val moveCommand = MecanumMoveCommand(vx = -gamepad1.left_stick_y.toDouble() * chassis.maxV * (1.0 - currentSpeedLimiter), vy = -gamepad1.left_stick_x.toDouble() * chassis.maxV * (1.0 - currentSpeedLimiter), theta = targetHeading)
            chassis.movementTarget = moveCommand

        }else{
            targetHeading = chassis.heading

            val moveCommand = MecanumMoveCommand(vx = -gamepad1.left_stick_y.toDouble() * chassis.maxV * (1.0 - currentSpeedLimiter), vy = -gamepad1.left_stick_x.toDouble() * chassis.maxV * (1.0 - currentSpeedLimiter))
            chassis.angularSpeedTarget = -gamepad1.right_stick_x.toDouble() * Math.PI * 1.5
            chassis.movementTarget = moveCommand
        }

//

//
//
        telemetry.addData("timeStep", timeStep)
//

//
        val intakePower = gamepad2.left_trigger - gamepad2.right_trigger.toDouble()
        intake.power = intakePower

//
        if (gamepad1.a) {
           arm.moveToPosition(ArmPosition.PASSBRIDGE)


//            // Get the calibration data
//            val calibrationData = chassis.imu.readCalibrationData()
//
//            // Save the calibration data to a file. You can choose whatever file
//            // name you wish here, but you'll want to indicate the same file name
//            // when you initialize the IMU in an opmode in which it is used. If you
//            // have more than one IMU on your robot, you'll of course want to use
//            // different configuration file names for each.
//            val filename = "AdafruitIMUCalibration.json"
//            val file = AppUtil.getInstance().getSettingsFile(filename)
//            ReadWriteFile.writeFile(file, calibrationData.serialize())
//            telemetry.log().add("saved to '%s'", filename)


        } else if (gamepad1.x) {
            arm.moveToPosition(ArmPosition.HOGAR)
        } else {
//
            if (gamepad2.a)
                arm.moveToPosition(ArmPosition.FIRST_LEVEL)
            else if (gamepad2.b)
                arm.moveToPosition(ArmPosition.SECOND_LEVEL)
            else if (gamepad2.y)
                arm.moveToPosition(ArmPosition.THIRD_LEVEL)
            else if (gamepad2.right_bumper)
                arm.moveToPosition(ArmPosition.FOURTH_LEVEL)
            else if (gamepad2.x)
                arm.moveToPosition(ArmPosition.HOGAR)
            else if (gamepad2.left_bumper || (arm.targetCoordinate == ArmPosition.HOGAR.coordinate && intakePower != 0.0))
                arm.moveToPosition(ArmPosition.INTAKE)
            else if(gamepad2.right_stick_button)
                arm.moveToPosition(ArmPosition.HOME_CAP)
        }


        if(gamepad1.right_bumper){
            trayHolder.setPosition(TrayHolderPosition.Grab)
        }

        if(gamepad1.left_bumper){
            trayHolder.setPosition(TrayHolderPosition.Release)
        }
//
        if (gamepad2.dpad_up) {
            arm.clamp.power = 1.0
        }

        if (gamepad2.dpad_down) {
            arm.clamp.power = 0.0
        }

    }

}

//
