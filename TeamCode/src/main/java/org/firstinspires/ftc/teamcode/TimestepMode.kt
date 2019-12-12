package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.Intake
import org.firstinspires.ftc.teamcode.controllers.chassis.Chassis
import org.firstinspires.ftc.teamcode.controllers.TimestepTester
import org.firstinspires.ftc.teamcode.core.EctoOpMode

@Autonomous(name="timestep")
class TimestepMode: EctoOpMode() {
    var targetHeading = 0.0
    val maxTargetHeadingRate = 45
    var rotations = 0.0
    val intake = Intake()

    init {
        controllers.add(intake)
    }

    override fun update(timeStep: Double) {

        intake.power = gamepad1.right_stick_x.toDouble()

        telemetry.addData("currentHeading" , targetHeading)
        telemetry.addData("rotations" , rotations)
        telemetry.addData("opMode timestep", timeStep * 1000)

    }
}