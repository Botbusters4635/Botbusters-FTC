package org.firstinspires.ftc.teamcode

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.Twist2D
import org.firstinspires.ftc.teamcode.core.EctoOpMode

import org.firstinspires.ftc.teamcode.Controllers.Chassis
import org.firstinspires.ftc.teamcode.Controllers.Arm
import org.firstinspires.ftc.teamcode.Controllers.ArmPosition
import org.firstinspires.ftc.teamcode.Controllers.Intake
import kotlin.math.absoluteValue

@TeleOp(name = "TeleOp")
class TeleOp : EctoOpMode() {
    val chassis = Chassis()
    val arm = Arm()
    val intake = Intake()

    var targetHeading = 0.0
    val maxTargetHeadingRate = 90


    init {
        addController(chassis)
        addController(arm)
        addController(intake)
    }

    override fun loop() {


        val targetVelocity = -gamepad1.right_stick_x.toDouble()



        val twist = Twist2D(vx = -gamepad1.left_stick_y.toDouble(), vy = -gamepad1.left_stick_x.toDouble(), w = targetVelocity)
        chassis.movementTarget = twist


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
        else if (gamepad2.right_bumper || (arm.targetCoordinates == ArmPosition.HOME.coordinates && intakePower != 0.0))
            arm.moveToPosition(ArmPosition.EXCHANGE)

        arm.setClampPower(gamepad2.left_stick_y.absoluteValue.toDouble())
        SystemClock.sleep(20)
    }

}

//
