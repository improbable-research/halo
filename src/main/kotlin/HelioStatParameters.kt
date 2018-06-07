import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class HelioStatParameters(var pivotPoint: Vector3D,
                          var pitchParameters: HelioStatParameters.ServoParameters,
                          var rotationParameters: HelioStatParameters.ServoParameters) {

    class ServoParameters(var m: Double, var c: Double, var pitch: Double, var rotation: Double) {

        constructor(m : Double, c: Double, pitch : Double) : this(m, c, pitch, -Math.PI/2.0) {}

        override fun toString() : String {
            return "{m: $m, c: $c, pitch: $pitch, rotation: $rotation}"
        }

        companion object {
            fun defaultParams(isOnTower : Boolean, isPitch : Boolean) : ServoParameters {
                return if(isOnTower) {
                    ServoParameters(-0.001534,
                            if(isPitch) 3.146 else 4.707 - Math.PI/2.0,
                            if(isPitch) Math.PI/2.0 else Math.PI,
                            if(isPitch) -Math.PI/2.0 else 0.0)
                } else {
                    ServoParameters(-0.001534,
                            if(isPitch) 3.146 else 4.707,
                            if(isPitch) Math.PI/2.0 else Math.PI,
                            if(isPitch) -Math.PI/2.0 else 0.0)
                }
            }
        }
    }

    companion object {
        fun defaultParams(pivotPoint : Vector3D) : HelioStatParameters {
            return if(pivotPoint.y > 0.5) {
                HelioStatParameters(pivotPoint,
                        ServoParameters.defaultParams(true, true),
                        ServoParameters.defaultParams(true, false)
                )
            } else {
                HelioStatParameters(pivotPoint,
                        ServoParameters.defaultParams(false, true),
                        ServoParameters.defaultParams(false, false)
                )
            }
        }
    }


    override fun toString() : String {
        return "PivotPoint: $pivotPoint,  PitchParams: $pitchParameters, RotationParams:$rotationParameters"
    }
}
