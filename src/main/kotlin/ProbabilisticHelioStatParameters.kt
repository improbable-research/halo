import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import io.improbable.keanu.vertices.dbl.probabilistic.SmoothUniformVertex

class ProbabilisticHelioStatParameters(var pivotPoint: ProbabilisticVector3D,
                                       var pitchParameters: ServoParameters,
                                       var rotationParameters: ServoParameters) {

    class ServoParameters(val m : DoubleVertex, val c : DoubleVertex, val axisPitch: DoubleVertex, val axisRotation: DoubleVertex) {
        constructor(params : HelioStatParameters.ServoParameters) : this(ConstantDoubleVertex(params.m), ConstantDoubleVertex(params.c), ConstantDoubleVertex(params.pitch), ConstantDoubleVertex(params.rotation))

        fun getValue() : HelioStatParameters.ServoParameters {
            return HelioStatParameters.ServoParameters(m.value, c.value, axisPitch.value, axisRotation.value)
        }
    }

    constructor(): this(ProbabilisticVector3D(),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0), GaussianVertex(Math.PI / 2.0, 0.02), GaussianVertex(0.0, 0.02)),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0), GaussianVertex(0.0, 0.02), SmoothUniformVertex(-2.0 * Math.PI, 2.0 * Math.PI))) {

        rotationParameters.axisRotation.value = 0.0
    }

    constructor(params: HelioStatParameters):
            this(ProbabilisticVector3D(params.pivotPoint), ServoParameters(params.pitchParameters), ServoParameters(params.rotationParameters))

    fun getValue() : HelioStatParameters {
        return HelioStatParameters(pivotPoint.getValue(), pitchParameters.getValue(), rotationParameters.getValue())
    }
}
