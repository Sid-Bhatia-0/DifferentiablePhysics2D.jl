import StaticArrays
const SA = StaticArrays
import ForwardDiff
const FD = ForwardDiff

function move(position, velocity, step_size)
    position_new = position + step_size * velocity
    return position_new, velocity
end

T = Float32
step_size = one(T)

radius = one(T)
position = zero(SA.SVector{2, T})

velocity_magnitude = FD.Dual(convert(T, 2), (one(T), zero(T)))
velocity_angle = FD.Dual(convert(T, pi / 6), (zero(T), one(T)))
velocity = SA.SVector(velocity_magnitude * cos(velocity_angle), velocity_magnitude * sin(velocity_angle))

t = 0
@show t
@show position
@show velocity
println("********************************************************************")

for i in 1:4
    global position, velocity = move(position, velocity, step_size)
    global t += 1
    @show t
    @show position
    @show velocity
    println("********************************************************************")
end
