import StaticArrays
const SA = StaticArrays
import LinearAlgebra
const LA = LinearAlgebra
import ForwardDiff
const FD = ForwardDiff

T = Float32
step_size = convert(T, 1)

r1 = convert(T, 1)
m1 = convert(T, 1)
p1 = zero(SA.SVector{2, T})
v1_mag = FD.Dual(convert(T, 2), (one(T), zero(T)))
v1_theta = FD.Dual(convert(T, pi / 6), (zero(T), one(T)))
v1 = SA.SVector(v1_mag * cos(v1_theta), v1_mag * sin(v1_theta))

d = convert(T, 4)
r2 = convert(T, 2)
m2 = convert(T, 2)
p2 = SA.SVector(d, zero(T))
v2 = zero(SA.SVector{2, T})

function move(position, velocity, step_size)
    position_new = position .+ (step_size * velocity)
    return position_new, velocity
end

t = 0
@show t
@show p1
@show v1
@show p2
@show v2
println("********************************************************************")

for i in 1:4
    global p1, v1 = move(p1, v1, step_size)
    global p2, v2 = move(p2, v2, step_size)
    global t += 1
    @show t
    @show p1
    @show v1
    @show p2
    @show v2
    println("********************************************************************")
end
