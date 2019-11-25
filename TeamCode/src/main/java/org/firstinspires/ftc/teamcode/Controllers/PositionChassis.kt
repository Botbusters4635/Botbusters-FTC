package org.firstinspires.ftc.teamcode.Controllers

import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings

class PositionChassis : Chassis() {

    var targetCoords = Coordinate(0.0, 0.0)

    val xPID = PID(PIDSettings(1.5, 0.0, 0.0))

    val maxVelocityChange = 1.0

    var currentVx = 0.0

    var maxVx = 0.5

    override fun update() {
        super.update()

        val distanceToTarget = Math.sqrt(Math.pow(targetCoords.x - currentCoords.x, 2.0) + Math.pow(targetCoords.y - currentCoords.y, 2.0))

        if(distanceToTarget < 0.05){
            movementTarget.vx = 0.0
            movementTarget.vy = 0.0
            movementTarget.theta = getHeading()
            return
        }

        xPID.target = 0.0
        var targetVx = xPID.update(distanceToTarget)

        targetVx = targetVx.coerceIn(-maxVx, maxVx)

        if(Math.abs(targetVx - currentVx) * timeStep > maxVelocityChange * timeStep){
            currentVx += Math.copySign(maxVelocityChange * timeStep, targetVx)
        }else{
            currentVx = targetVx
        }


        val angle = Math.atan2(targetCoords.y - currentCoords.y, targetCoords.x - currentCoords.x) * 180.0 / Math.PI
        var targetHeading = getHeading() + angle

        if(targetHeading > 180.0){
            targetHeading -= 360.0
        }

        if(targetHeading < -180.0){
            targetHeading += 360.0
        }

        movementTarget.vx = currentVx
        movementTarget.vy = 0.0
        movementTarget.theta = targetHeading
    }
}
