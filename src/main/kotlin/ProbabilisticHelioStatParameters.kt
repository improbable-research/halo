import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class ProbabilisticHelioStatParameters(var pivotPoint: ProbabilisticVector3D,
                                       var mPitch: DoubleVertex, var cPitch: DoubleVertex,
                                       var mRotation: DoubleVertex, var cRotation: DoubleVertex) {

    constructor(): this(ProbabilisticVector3D(),
                        GaussianVertex(0.0, 100.0), GaussianVertex(0.0, 100.0),
                        GaussianVertex(0.0, 100.0), GaussianVertex(0.0, 100.0))

    constructor(pivotPoint: Vector3D, mPitch: Double, cPitch: Double, mRotation: Double, cRotation: Double):
            this(ProbabilisticVector3D(pivotPoint),
                 ConstantDoubleVertex(mPitch), ConstantDoubleVertex(cPitch),
                 ConstantDoubleVertex(mRotation), ConstantDoubleVertex(cRotation))

    constructor(params: HelioStatParameters):
            this(params.pivotPoint, params.pitchParameters.m, params.pitchParameters.c,
                 params.rotationParameters.m, params.rotationParameters.c)

}