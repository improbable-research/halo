import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class HelioStatParameters(var pivotPoint: Vector3D, var pitchParameters: HelioStatParameters.ServoParameters,
                          var rotationParameters: HelioStatParameters.ServoParameters) {
    class ServoParameters(var m: Double, var c: Double)
}