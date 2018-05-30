import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class HelioStat (var pivotPoint: ProbabilisticVector3D, // expected location of pivot point
                 var calibrationObservations: List<Observation>) {

    // Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

    var heliostatOffsetFromPivot = GaussianVertex(100.0, 10.0)

    // Assume linear map from servoInput, x,  to corresponding heliostat rotation, y: y = mx + c
    var mPitch = GaussianVertex(1.0, 1.0)
    var cPitch = GaussianVertex(1.0, 1.0)
    var mRotation = GaussianVertex(1.0, 1.0)
    var cRotation = GaussianVertex(1.0, 1.0)

    fun computeHeliostatNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatPitch = mPitch * servoPitchSignal + cPitch
        var heliostatRotation = mRotation * servoRotationSignal + cRotation
        return ProbabilisticVector3D(heliostatPitch.cos() * heliostatRotation.cos(),
                        heliostatPitch.sin() * heliostatRotation.cos(),
                           heliostatRotation.sin())
    }

    fun computeHeliostatCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        var vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computeTargetPoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                            incomingSunDirection: ProbabilisticVector3D, distance: DoubleVertex): ProbabilisticVector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatPlane = computeHeliostatPlane(servoPitchSignal, servoRotationSignal)
        var unitNormal = heliostatPlane.unit()
        var reflectedDirection = incomingSunDirection - unitNormal * incomingSunDirection.dot(unitNormal) * 2.0
        return heliostatCentreLocation + reflectedDirection * distance
    }

}