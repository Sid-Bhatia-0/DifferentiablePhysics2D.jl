import StaticArrays
const SA = StaticArrays
import LinearAlgebra
const LA = LinearAlgebra
import ForwardDiff
const FD = ForwardDiff
import PhysicsPrimitives2D
import PhysicsPrimitives2D: PP2D

T = Float32
step_size = convert(T, 1 / 60)
e = one(T)

r1 = convert(T, 1)
m1 = convert(T, 1)
c1 = PP2D.StdCircle(r1)
p1 = zero(SA.SVector{2, T})
v1_mag = FD.Dual(convert(T, 2), (one(T), zero(T)))
v1_theta = FD.Dual(convert(T, pi / 6), (zero(T), one(T)))
v1 = SA.SVector(v1_mag * cos(v1_theta), v1_mag * sin(v1_theta))

d = convert(T, 4)
r2 = convert(T, 2)
c2 = PP2D.StdCircle(r2)
m2 = convert(T, 2)
p2 = SA.SVector(d, zero(T))
v2 = zero(SA.SVector{2, T})

function move(position, velocity, step_size)
    position_new = position .+ (step_size * velocity)
    return position_new, velocity
end

function PP2D.is_colliding(a::PP2D.StdCircle, b::PP2D.StdCircle, pos_ba)
    r_a = PP2D.get_radius(a)
    r_b = PP2D.get_radius(b)
    r = r_a + r_b
    return LA.dot(pos_ba, pos_ba) < r * r
end

function get_separation_data(a::PP2D.StdCircle{T}, b::PP2D.StdCircle{T}, pos_ba) where {T}
    r_a = PP2D.get_radius(a)
    r_b = PP2D.get_radius(b)
    d = LA.norm(pos_ba)
    penetration = r_a + r_b - d
    normal = pos_ba / d
    if all(isfinite.(normal))
        return penetration, normal
    else
        normal = SA.SVector(FD.Dual(one(T), (zero(T), zero(T))), FD.Dual(one(T), (zero(T), zero(T))))
        return penetration, normal
    end
end

function PP2D.get_normal_impulse(inv_mass_a, inv_mass_b, initial_velocity_ao, initial_velocity_bo, e, normal_o)
    initial_velocity_ba = initial_velocity_bo .- initial_velocity_ao
    initial_velocity_ba_normal_o = LA.dot(initial_velocity_ba, normal_o)
    temp = -e * initial_velocity_ba_normal_o
    final_velocity_ba_normal_o = max(zero(temp), temp)
    j_ao_normal_o = (initial_velocity_ba_normal_o - final_velocity_ba_normal_o) / (inv_mass_a + inv_mass_b)
    j_bo_normal_o = -j_ao_normal_o
    return j_ao_normal_o, j_bo_normal_o
end

t = 0
@show t
@show p1
@show v1
@show p2
@show v2
@show PP2D.is_colliding(c1, c2, p2 .- p1)
println("********************************************************************")

for i in 1:120
    global p1, v1 = move(p1, v1, step_size)
    global p2, v2 = move(p2, v2, step_size)
    global t += 1
    @show t
    @show p1
    @show v1
    @show p2
    @show v2
    @show PP2D.is_colliding(c1, c2, p2 .- p1)
    if PP2D.is_colliding(c1, c2, p2 .- p1)
        @info "collision occured!"
        penetration, normal = get_separation_data(c1, c2, p2 .- p1)
        j1, j2 = PP2D.get_normal_impulse(inv(m1), inv(m2), v1, v2, e, normal)
        global v1 = v1 .+ inv(m1) * j1 * normal
        global v2 = v2 .+ inv(m2) * j2 * normal
        @show j1
        @show j2
        @show penetration
        @show normal
    end
    println("********************************************************************")
end
