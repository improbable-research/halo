import io.improbable.keanu.plating.PlateBuilder
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class HelioStat (var expectedLocation: Vector3D,
                 var calibrationObservations: List<Observation>) {

    // Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

    var heliostatOffsetFromPivot = GaussianVertex(100.0, 10.0)

    // TODO
    var pivotPoint = findPivotPoint()

    // Assume linear map from servoInput, x,  to corresponding heliostat rotation, y: y = mx + c
    var mPitch = GaussianVertex(1.0, 1.0)
    var cPitch = GaussianVertex(1.0, 1.0)
    var mRotation = GaussianVertex(1.0, 1.0)
    var cRotation = GaussianVertex(1.0, 1.0)

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

    fun computeTargetPoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                            incomingSunDirection: Vector3D, distance: DoubleVertex): Vector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatPlane = computeHeliostatPlane(servoPitchSignal, servoRotationSignal)
        var unitNormal = heliostatPlane.unit()
        var reflectedDirection = incomingSunDirection - unitNormal * incomingSunDirection.dot(unitNormal) * 2.0
        return heliostatCentreLocation + reflectedDirection * distance
    }

}