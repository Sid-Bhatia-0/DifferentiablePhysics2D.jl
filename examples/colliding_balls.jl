import StaticArrays
const SA = StaticArrays
import LinearAlgebra
const LA = LinearAlgebra
import ForwardDiff
const FD = ForwardDiff
import PhysicsPrimitives2D
import PhysicsPrimitives2D: PP2D

T = Float32
step_size = convert(T, 1 / 5040)
n = 5040
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

move(position, velocity, step_size) = position .+ (step_size * velocity)

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

get_v2_theta(v1_theta, d, r1, r2) = asin(d * sin(v1_theta) / (r1 + r2)) - v1_theta
get_v2_mag(v1_mag, v1_theta, v2_theta, m1, m2, e) = (((one(e) + e) * m1 * v1_mag * cos(v1_theta + v2_theta)) / (m1 + m2))

function get_v2(v1_mag, v1_theta, d, r1, r2, m1, m2, e)
    v2_theta = get_v2_theta(v1_theta, d, r1, r2)
    v2_mag = get_v2_mag(v1_mag, v1_theta, v2_theta, m1, m2, e)
    return SA.SVector(v2_mag * cos(v2_theta), -v2_mag * sin(v2_theta))
end

function simulate(c1, c2, p1, p2, v1, v2, m1, m2, e, n)
    t = 0
    for i in 1:n
        p1 = move(p1, v1, step_size)
        p2 = move(p2, v2, step_size)
        collision = PP2D.is_colliding(c1, c2, p2 .- p1)
        t += 1
        if collision
            @info "collision occured!"
            penetration, normal = get_separation_data(c1, c2, p2 .- p1)
            j1, j2 = PP2D.get_normal_impulse(inv(m1), inv(m2), v1, v2, e, normal)
            v1 = v1 .+ inv(m1) * j1 * normal
            v2 = v2 .+ inv(m2) * j2 * normal
            @show t
            @show p1
            @show p2
            @show v1
            @show v2
            @show j1
            @show j2
            @show penetration
            @show normal
            println("********************************************************************")
        end
    end
    return p1, p2, v1, v2
end

p1_final, p2_final, v1_final, v2_final = simulate(c1, c2, p1, p2, v1, v2, m1, m2, e, n)

@show p1_final
@show p2_final
@show v1_final
@show v2_final
v2_final_mag = LA.norm(v2_final)
v2_final_theta = atan(v2_final[2], v2_final[1])
@show v2_final_mag
@show v2_final_theta

v2_analytical_theta = get_v2_theta(v1_theta, d, r1, r2)
v2_analytical_mag = get_v2_mag(v1_mag, v1_theta, v2_analytical_theta, m1, m2, e)
v2_analytical = SA.SVector(v2_analytical_mag * cos(v2_analytical_theta), -v2_analytical_mag * sin(v2_analytical_theta))
@show v2_analytical_mag
@show v2_analytical_theta
@show v2_analytical
