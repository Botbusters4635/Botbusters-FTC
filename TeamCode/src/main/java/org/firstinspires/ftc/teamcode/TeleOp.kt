package org.firstinspires.ftc.teamcode

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Core.EctoOpMode
import org.firstinspires.ftc.teamcode.systems.Chassis
import org.firstinspires.ftc.teamcode.systems.Twist2D
import kotlin.math.*

@TeleOp(name = "TeleOp")
class TeleOp: EctoOpMode() {
    val chassis = Chassis()
    init {
        addController(chassis)
    }
    var targetHeading = 0.0
    val maxTargetHeadingRate = 360
    var lastTimeRun = SystemClock.elapsedRealtime() / 1000.0
    var lastError = 0.0
    var kP = 0.016
    var kD = 0.00135
    var fieldOrientedEnabled = false

    override fun init_loop() {
        telemetry.msTransmissionInterval = 20
    }
    override fun loop() {
        val timeStep = (SystemClock.elapsedRealtime() / 1000.0) - lastTimeRun
//
//
        val targetHeadingDelta = -gamepad1.right_stick_x * maxTargetHeadingRate
//
        targetHeading += targetHeadingDelta * timeStep
//
//
        if(targetHeading > 180){
            targetHeading -= 360
        }
        if(targetHeading < -180){
            targetHeading += 360
        }
//
//
        var error = targetHeading - chassis.getHeading()
        val headingRadians = chassis.getHeading() * PI / 180.0
//
        if(error > 180){
            error -= 360
        }
        if(error < -180){
            error += 360
        }
//
        val errorDelta = (error - lastError) / timeStep
//
        val output = error * kP + errorDelta * kD

        var topLeftMotor = chassis.topLeftMotor.power
        var topRightMotor = chassis.topRightMotor.power
        var downLeftMotor = chassis.downLeftMotor.power
        var downRightMotor = chassis.downRightMotor.power


        val twist = Twist2D(vx = -gamepad1.left_stick_y.toDouble(), vy = -gamepad1.left_stick_x.toDouble(), w = -gamepad1.right_stick_x.toDouble())

//        if(twist.vx.absoluteValue < twist.vy.absoluteValue){
            twist.w = output
//        }

//        if(fieldOrientedEnabled){
//            val temp = twist.vx * cos(headingRadians) + twist.vy * sin(headingRadians)
//            twist.vy = twist.vy * cos(headingRadians) - twist.vx * sin(headingRadians)
//            twist.vx = temp
//        }
////
        if(gamepad1.right_bumper){
            fieldOrientedEnabled = true
        }

        if(gamepad1.left_bumper){
            fieldOrientedEnabled = false
        }
//
        chassis.move(twist)
        lastTimeRun = SystemClock.elapsedRealtime() / 1000.0
        lastError = error
//        telemetry.addData("Heading", chassis.getHeading())
//        telemetry.addData("Error Heading", error)
        telemetry.addData("topRight",  chassis.topRightMotor.currentPosition)
        telemetry.addData("topLeft",  chassis.topLeftMotor.currentPosition)
        telemetry.addData("downRight",  chassis.downRightMotor.currentPosition)
        telemetry.addData("downLeft",  chassis.downLeftMotor.currentPosition)
//
        SystemClock.sleep(20)
    }

}