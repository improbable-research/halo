import io.improbable.keanu.plating.PlateBuilder
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class HelioStat (var expectedLocation: Vector3D,
                 var calibrationObservations: List<Observation>) {

    // Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

    var heliostatOffsetFromPivot = GaussianVertex(100.0, 10.0)

    var pivotPoint = findPivotPoint()

    // Assume linear map from servoInput, x,  to corresponding heliostat rotation, y: y = mx + c
    var mPitch = GaussianVertex(1.0, 1.0)
    var cPitch = GaussianVertex(1.0, 1.0)
    var mRotation = GaussianVertex(1.0, 1.0)
    var cRotation = GaussianVertex(1.0, 1.0)


    fun findPivotPoint(): Vector3D {
        return Vector3D()
    }
//
//        var m = GaussianVertex(1.0, 1.0)
//        var n = GaussianVertex(1.0, 1.0)
//        var o = GaussianVertex(1.0, 1.0)
//
//        var plates = PlateBuilder<Observation>()
//                .fromIterator(calibrationObservations.iterator())
//                .withFactory( {plate, data ->
//
//                    // Placeholder function for trial
//                    var one = m * data.planeX + n * data.planeY + o * data.planeZ
//                    one.observe(1.0)
//                } )
//
//    }

    fun computeHeliostatNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var heliostatPitch = mPitch * servoPitchSignal + cPitch
        var heliostatRotation = mRotation * servoRotationSignal + cRotation
        return Vector3D(heliostatPitch.cos() * heliostatRotation.cos(),
                        heliostatPitch.sin() * heliostatRotation.cos(),
                           heliostatRotation.sin())
    }

    fun computeHeliostatCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        var vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computeDesiredHeliostatPlane (incomingSunDirection: Vector3D, targetPoint: Vector3D): Vector3D {
        var pivotToTargetDirection = (targetPoint - pivotPoint).unit()
        return ((incomingSunDirection + pivotToTargetDirection) / 2.0).cross(incomingSunDirection.cross(pivotToTargetDirection))
    }

    fun computeDesiredControlParameters (incomingSunDirection: Vector3D, targetPoint: Vector3D) {
        var desiredHeliostatPlane = computeDesiredHeliostatPlane(incomingSunDirection, targetPoint)

    }


}