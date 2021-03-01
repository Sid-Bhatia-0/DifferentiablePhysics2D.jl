import DifferentiablePhysics2D
import DifferentiablePhysics2D: DP2D
import PhysicsPrimitives2D
import PhysicsPrimitives2D: PP2D
import StaticArrays
const SA = StaticArrays
import LinearAlgebra
const LA = LinearAlgebra
import ForwardDiff
const FD = ForwardDiff
# import Makie

const T = Float32
const STEP_SIZE = one(T)

const RADIUS_BALL1 = one(T)
const SHAPE_BALL1 = PP2D.StdCircle(RADIUS_BALL1)
const POSITION_BALL1 = zero(SA.SVector{2, T})
const POSITION_CHANGE_BALL1 = zero(SA.SVector{2, T})
const VELOCITY_BALL1 = SA.SVector(one(T), zero(T))
const VELOCITY_CHANGE_BALL1 = zero(SA.SVector{2, T})
const MASS_BALL1 = one(T)
const INV_MASS_BALL1 = one(T)

const RADIUS_BALL2 = one(T)
const SHAPE_BALL2 = PP2D.StdCircle(RADIUS_BALL2)
const POSITION_BALL2 = SA.SVector(convert(T, 3.5), zero(T))
const POSITION_CHANGE_BALL2 = zero(SA.SVector{2, T})
const VELOCITY_BALL2 = SA.SVector(zero(T), zero(T))
const VELOCITY_CHANGE_BALL2 = zero(SA.SVector{2, T})
const MASS_BALL2 = one(T)
const INV_MASS_BALL2 = one(T)

function move(p1, v1, p2, v2)
    p1_new = p1 .+ (STEP_SIZE * v1)
    p2_new = p2 .+ (STEP_SIZE * v2)
    return p1_new, v1, p2_new, v2
end

p1 = POSITION_BALL1
v1 = VELOCITY_BALL1
p2 = POSITION_BALL2
v2 = VELOCITY_BALL2
t = 0

@show t
@show p1
@show v1
@show p2
@show v2
@show PP2D.is_colliding(SHAPE_BALL1, SHAPE_BALL2, p2 .- p1)
println("********************************************************************")

for i in 1:2
    global p1, v1, p2, v2 = move(p1, v1, p2, v2)
    global t += 1
    @show t
    @show p1
    @show v1
    @show p2
    @show v2
    @show PP2D.is_colliding(SHAPE_BALL1, SHAPE_BALL2, p2 .- p1)
    println("********************************************************************")
end
