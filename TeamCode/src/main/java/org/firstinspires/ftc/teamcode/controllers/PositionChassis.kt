package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import kotlinx.coroutines.isActive
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.core.Path
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sqrt

class PositionChassis : Chassis() {

    var targetCoords = Coordinate(0.0, 0.0)

    val xPID = PID(PIDSettings(2.5, 0.0, 0.0))
    val yPID = PID(PIDSettings(2.5, 0.0, 0.0))

    var maxAutoVx = 0.4
    var maxAutoVy = 0.4
    var followingPath = false

    val distanceToTarget
        get() = sqrt((targetCoords.x - currentCoords.x).pow(2.0) + Math.pow(targetCoords.y - currentCoords.y, 2.0))

    val onTarget
        get() = distanceToTarget < 0.05

    override fun update(timeStep: Double) {
        angularPID.maxOutput = 0.7

        super.update(timeStep)

        if(followingPath){
            if(onTarget){
                xPID.clear()
                yPID.clear()
            }else{
                xPID.target = targetCoords.x
                yPID.target = targetCoords.y
                var targetVx = xPID.update(currentCoords.x, timeStep)
                var targetVy = yPID.update(currentCoords.y, timeStep)

                targetVx = targetVx.coerceIn(-maxAutoVx, maxAutoVx)
                targetVy = targetVy.coerceIn(-maxAutoVy, maxAutoVy)

                val headingInRadians = degreesToRadians(heading)

                movementTarget.vx = targetVx * Math.cos(headingInRadians) + targetVy * Math.sin(headingInRadians)
                movementTarget.vy = targetVy * Math.cos(headingInRadians) - targetVx * Math.sin(headingInRadians)
            }
        }
        telemetry.addData("x, y", "%.2f %.2f", currentCoords.x, currentCoords.y)
    }

    fun runToPositionBlocking(target: Coordinate) = runBlocking{
        targetCoords = target
        followingPath = true

        while(!onTarget && isActive){

        }
        movementTarget.vx = 0.0
        movementTarget.vy = 0.0
        followingPath = false
    }

    fun turnToAngleBlocking(targetAngle: Double) = runBlocking {
        val startTime = SystemClock.elapsedRealtime() / 1000.0
        var currentTime: Double
        movementTarget.theta = targetAngle

        var error = (targetAngle - heading)

        while(error.absoluteValue > 5 && isActive){
            error = (targetAngle - heading)
            if(error > 180.0){
                error -= 360
            }

            if(error < -180.0){
                error += 360
            }

            telemetry.addData("error Angle", (targetAngle - heading).absoluteValue)
            telemetry.addData("Target Angle", targetAngle)
            telemetry.addData("heading", heading)

            currentTime = SystemClock.elapsedRealtime() / 1000.0 - startTime

            if(currentTime > 10.0){
                break
            }
        }
    }
    fun moveTimed(command: MecanumMoveCommand, seconds: Double) = runBlocking{
        val startTime = SystemClock.elapsedRealtime() / 1000.0
        var currentTime = 0.0
        movementTarget = command

        while(currentTime < seconds && isActive){
            currentTime = SystemClock.elapsedRealtime() / 1000.0 - startTime
        }
        movementTarget.vx = 0.0
        movementTarget.vy = 0.0
        angularPID.maxOutput = 1.5

    }
    fun followPathBlocking(path: Path) = runBlocking {
        for(waypoint in path){
            runToPositionBlocking(waypoint)

        }
    }

}