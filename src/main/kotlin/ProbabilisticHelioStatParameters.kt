import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import io.improbable.keanu.vertices.dbl.probabilistic.SmoothUniformVertex

class ProbabilisticHelioStatParameters(var pivotPoint: ProbabilisticVector3D,
                                       var pitchParameters: ServoParameters,
                                       var rotationParameters: ServoParameters) {

    class ServoParameters(val m : DoubleVertex, val c : DoubleVertex, var axisPitch: DoubleVertex, var axisRotation: DoubleVertex) {
        constructor(params : HelioStatParameters.ServoParameters) : this(ConstantDoubleVertex(params.m), ConstantDoubleVertex(params.c), ConstantDoubleVertex(params.pitch), ConstantDoubleVertex(params.rotation))

        fun getValue() : HelioStatParameters.ServoParameters {
            return HelioStatParameters.ServoParameters(m.value, c.value, axisPitch.value, axisRotation.value)
        }
    }

    constructor(): this(ProbabilisticVector3D(),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0), GaussianVertex(Math.PI / 2.0, 0.04), ConstantDoubleVertex(-Math.PI / 2.0)),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0), GaussianVertex(Math.PI, 0.04), SmoothUniformVertex(-Math.PI, Math.PI))) {

        rotationParameters.axisRotation.value = 0.0
        rotationParameters.axisPitch.value = Math.PI
        pitchParameters.axisRotation.value = -Math.PI/2.0
        pitchParameters.axisPitch.value = Math.PI/2.0
    }

    constructor(params: HelioStatParameters):
            this(ProbabilisticVector3D(params.pivotPoint), ServoParameters(params.pitchParameters), ServoParameters(params.rotationParameters))


    fun fixAxisParametersToSpherical() {
        pitchParameters.axisPitch = ConstantDoubleVertex(Math.PI/2.0)
        pitchParameters.axisRotation = ConstantDoubleVertex(-Math.PI/2.0)
        rotationParameters.axisPitch = ConstantDoubleVertex(Math.PI)
        rotationParameters.axisRotation = ConstantDoubleVertex(0.0)
    }

    fun getValue() : HelioStatParameters {
        return HelioStatParameters(pivotPoint.getValue(), pitchParameters.getValue(), rotationParameters.getValue())
    }
}
