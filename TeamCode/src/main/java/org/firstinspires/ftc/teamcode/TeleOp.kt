package org.firstinspires.ftc.teamcode

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.Arm
import org.firstinspires.ftc.teamcode.Controllers.ArmPosition
import org.firstinspires.ftc.teamcode.Controllers.Chassis
import org.firstinspires.ftc.teamcode.Controllers.Intake
import org.firstinspires.ftc.teamcode.core.EctoOpMode
import org.firstinspires.ftc.teamcode.core.Twist2D
import kotlin.math.absoluteValue

@TeleOp(name = "TeleOp")
class TeleOp : EctoOpMode() {
    val chassis = Chassis()
    val arm = Arm()
    val intake = Intake()

    var targetHeading = 0.0
    val maxTargetHeadingRate = 90

    private var lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

    init {
        addController(chassis)
        addController(arm)
        addController(intake)
    }

    override fun update() {
        var timeStep = SystemClock.elapsedRealtime() / 1000.0 - lastTimeRun


        val desiredChange = -gamepad1.right_stick_x.toDouble()

        targetHeading += desiredChange * 0.1 * maxTargetHeadingRate

        if(targetHeading > 180.0){
            targetHeading -= 360
        }

        if(targetHeading < -180.0){
            targetHeading += 360
        }


        val twist = Twist2D(vx = -gamepad1.left_stick_y.toDouble(), vy = -gamepad1.left_stick_x.toDouble(), w = targetHeading)
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
        else if (gamepad2.right_bumper || (arm.targetCoordinate == ArmPosition.HOME.coordinate && intakePower != 0.0))
            arm.moveToPosition(ArmPosition.EXCHANGE)

        arm.setClampPower(gamepad2.left_stick_y.absoluteValue.toDouble())
        telemetry.addData("odometry", chassis.getCurrentCords())
        telemetry.addData("Timestep",timeStep)

        // Show the elapsed game time and wheel power.

        lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

    }

}

//
