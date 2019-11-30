package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.Arm
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.PositionArm
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.EctoOpMode

@TeleOp(name = "TeleOp")
class TeleOp : EctoOpMode() {
    val chassis = Chassis()
    val arm = SynchronizedArm()
    val intake = Intake()

    var targetHeading = 0.0
    val maxTargetHeadingRate = 720.0

    init {
        addController(chassis)
        addController(arm)
        addController(intake)
    }

    override fun update(timeStep: Double) {

        val desiredChange = -gamepad1.right_stick_x.toDouble()
        targetHeading += (desiredChange * maxTargetHeadingRate) * timeStep



        if(targetHeading > 180.0){
            targetHeading -= 360
        }

        if(targetHeading < -180.0){
            targetHeading += 360
        }

        telemetry.addData("timeStep", timeStep)

        val moveCommand = MecanumMoveCommand(vx = -gamepad1.left_stick_y.toDouble() * chassis.maxV, vy = -gamepad1.left_stick_x.toDouble() * chassis.maxV, theta = targetHeading)
        chassis.movementTarget = moveCommand

        val intakePower = gamepad2.left_trigger - gamepad2.right_trigger.toDouble()
        intake.power = intakePower


        if (gamepad2.y)
            arm.moveToPosition(ArmPosition.TOP)
        else if (gamepad2.a)
            arm.moveToPosition(ArmPosition.LOW)
        else if (gamepad2.b)
            arm.moveToPosition(ArmPosition.MEDIUM)
        else if (gamepad2.x)
            arm.moveToPosition(ArmPosition.HOME)
        else if (gamepad2.right_bumper || (arm.targetCoordinate == ArmPosition.HOME.coordinate && intakePower != 0.0))
            arm.moveToPosition(ArmPosition.INTAKE)


        if(gamepad2.right_stick_button){
            arm.clamp.power = 1.0
        }

        if(gamepad2.left_stick_button){
            arm.clamp.power = 0.0
        }
//
//        // Show the elapsed game time and wheel power.
//
//        lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

    }

}

//
