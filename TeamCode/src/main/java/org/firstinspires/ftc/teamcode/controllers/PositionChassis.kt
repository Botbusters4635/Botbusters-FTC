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

    val maxAutoVx = 0.15
    val maxAutoVy = 0.1
    var followingPath = false

    val distanceToTarget
        get() = sqrt((targetCoords.x - currentCoords.x).pow(2.0) + Math.pow(targetCoords.y - currentCoords.y, 2.0))

    val onTarget
        get() = distanceToTarget < 0.05

    override fun update(timeStep: Double) {
        movementTarget.vx = 0.0
        movementTarget.vy = 0.0
        angularPID.maxOutput = 1.5

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


                telemetry.addData("currentVx", targetVx)
                telemetry.addData("currentVy", targetVy)
                val headingInRadians = degreesToRadians(heading)

                movementTarget.vx = targetVx * Math.cos(headingInRadians) + targetVy * Math.sin(headingInRadians)
                movementTarget.vy = targetVy * Math.cos(headingInRadians) - targetVx * Math.sin(headingInRadians)
            }
        }
        super.update(timeStep)
    }

    fun runToPosition(target: Coordinate) = runBlocking{
        targetCoords = target
        followingPath = true

        while(!onTarget && isActive){

        }
        followingPath = false
    }

    fun turnToAngle(targetAngle: Double) = runBlocking {
        val startTime = SystemClock.elapsedRealtime() / 1000.0
        var currentTime: Double
        movementTarget.theta = targetAngle

        var error = (heading - targetAngle).absoluteValue

        while(error > 3 && isActive){
            error = (heading - targetAngle).absoluteValue
            if(error > 180.0){
                error -= 360
            }

            if(error < -180.0){
                error += 360
            }

            telemetry.addData("error Angle", (heading - targetAngle).absoluteValue)
            telemetry.addData("Target Angle", targetAngle)
            telemetry.addData("heading", heading)

            currentTime = SystemClock.elapsedRealtime() / 1000.0 - startTime

            if(currentTime > 3.0){
                break
            }
        }
    }

    fun followPath(path: Path) = runBlocking {
        for(waypoint in path){
            runToPosition(waypoint)

        }
    }

}