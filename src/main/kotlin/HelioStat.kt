import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

// Note: Sign convention:
// Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

class HelioStat (var params: ProbabilisticHelioStatParameters) {

    var heliostatOffsetFromPivot = 0.144 // measured with caliper, variance sub-millimetre

    fun computeHeliostatNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatPitch = params.mPitch * servoPitchSignal + params.cPitch
        var heliostatRotation = params.mRotation * servoRotationSignal + params.cRotation
        return ProbabilisticVector3D(
                heliostatPitch.cos() * heliostatRotation.cos(),
                heliostatPitch.sin(),
                heliostatPitch.cos() * heliostatRotation.sin()
        )
    }

    fun computeHeliostatCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        var vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return params.pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computePlaneNorm (normal: ProbabilisticVector3D): DoubleVertex {
        return params.pivotPoint.dot(normal) + heliostatOffsetFromPivot
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