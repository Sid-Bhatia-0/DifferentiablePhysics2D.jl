module DifferentiablePhysics2D

const DP2D = DifferentiablePhysics2D
export DP2D

import PhysicsPrimitives2D
import PhysicsPrimitives2D: PP2D
import StaticArrays
const SA = StaticArrays
import LinearAlgebra
const LA = LinearAlgebra
import ForwardDiff
const FD = ForwardDiff
import Requires

function __init__()
    Requires.@require Makie = "ee78f7c6-11fb-53f2-987a-cfe4a2b5a57a" include("render.jl")
end

end
