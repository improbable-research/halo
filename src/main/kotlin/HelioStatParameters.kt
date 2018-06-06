import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class HelioStatParameters(var pivotPoint: Vector3D,
                          var pitchParameters: HelioStatParameters.ServoParameters,
                          var rotationParameters: HelioStatParameters.ServoParameters) {

    class ServoParameters(var m: Double, var c: Double, var pitch: Double, var rotation: Double) {

        constructor(m : Double, c: Double, pitch : Double) : this(m, c, pitch, -Math.PI/2.0) {}

        override fun toString() : String {
            return "{m: $m, c: $c, pitch: $pitch, rotation: $rotation}"
        }
    }

    override fun toString() : String {
        return "PivotPoint: $pivotPoint,  PitchParams: $pitchParameters, RotationParams:$rotationParameters"
    }
}
