import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class ProbabilisticHelioStatParameters(var pivotPoint: ProbabilisticVector3D,
                                       var pitchParameters: ServoParameters,
                                       var rotationParameters: ServoParameters) {

    class ServoParameters(val m : DoubleVertex, val c : DoubleVertex) {
        constructor(params : HelioStatParameters.ServoParameters) : this(ConstantDoubleVertex(params.m), ConstantDoubleVertex(params.c))

        fun getValue() : HelioStatParameters.ServoParameters {
            return HelioStatParameters.ServoParameters(m.value, c.value)
        }
    }

    constructor(): this(ProbabilisticVector3D(),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0)),
                        ServoParameters(GaussianVertex(0.0, 1.0), GaussianVertex(0.0, 3.0)))

    constructor(params: HelioStatParameters):
            this(ProbabilisticVector3D(params.pivotPoint), ServoParameters(params.pitchParameters), ServoParameters(params.rotationParameters))

    fun getValue() : HelioStatParameters {
        return HelioStatParameters(pivotPoint.getValue(), pitchParameters.getValue(), rotationParameters.getValue())
    }
}