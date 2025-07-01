using Test
using LinearAlgebra

using r3r
using r3r.R3RUtils: solve_quadratic, compute_intersection_point

@testset "solve_quadratic tests" begin
    # Test case 1: Two real solutions
    # x^2 - 5x + 6 = 0, solutions: x = 2, x = 3
    a, b, c = 1, -5, 6
    t1, t2 = solve_quadratic(a, b, c)
    @test t1 ≈ 3.0
    @test t2 ≈ 2.0

    # Test case 2: One real solution (discriminant = 0)
    # x^2 - 4x + 4 = 0, solution: x = 2
    a, b, c = 1, -4, 4
    t1, t2 = solve_quadratic(a, b, c)
    @test t1 ≈ 2.0
    @test t2 ≈ 2.0

    # Test case 3: No real solutions (discriminant < 0)
    # x^2 + x + 1 = 0, no real solutions
    a, b, c = 1, 1, 1
    result = solve_quadratic(a, b, c)
    @test result === nothing

    # Test case 4: Linear equation (a = 0)
    # This case might cause division by zero, let's see how it behaves
    a, b, c = 0, 2, -4  # 2x - 4 = 0, solution: x = 2
    # Note: This function doesn't handle linear equations properly
end

@testset "compute_intersection_point tests" begin
    # Test case 1: Line segment intersects circle
    # Horizontal line from (0,0) to (4,0), circle at (2,0) with radius 1
    x1 = [0.0, 0.0]
    x2 = [4.0, 0.0]
    pos = [2.0, 0.0]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection !== nothing
    # Should intersect at (1,0) or (3,0), let's check distance from center
    @test abs(norm(intersection - pos) - r) < 1e-10
    @test 0 <= (intersection[1] - x1[1]) / (x2[1] - x1[1]) <= 1  # t parameter in [0,1]

    # Test case 2: Line segment doesn't reach the circle
    # Short segment from (0,0) to (0.5,0), circle at (2,0) with radius 1
    x1 = [0.0, 0.0]
    x2 = [0.5, 0.0]
    pos = [2.0, 0.0]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection === nothing

    # Test case 3: Line passes through circle but segment doesn't intersect
    # Segment from (4,0) to (5,0), circle at (2,0) with radius 1
    x1 = [4.0, 0.0]
    x2 = [5.0, 0.0]
    pos = [2.0, 0.0]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection === nothing

    # Test case 4: Line is tangent to circle
    # Vertical line from (1,1) to (1,2), circle at (0,1.5) with radius 1
    x1 = [1.0, 1.0]
    x2 = [1.0, 2.0]
    pos = [0.0, 1.5]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection !== nothing
    @test abs(norm(intersection - pos) - r) < 1e-10

    # Test case 5: Diagonal line intersecting circle
    # Line from (0,0) to (4,4), circle at (2,2) with radius 1
    x1 = [0.0, 0.0]
    x2 = [4.0, 4.0]
    pos = [2.0, 2.0]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection !== nothing
    @test abs(norm(intersection - pos) - r) < 1e-10

    # Test case 6: Line doesn't intersect circle at all
    # Line from (0,0) to (1,0), circle at (0,2) with radius 1
    x1 = [0.0, 0.0]
    x2 = [1.0, 0.0]
    pos = [0.0, 2.0]
    r = 1.0

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection === nothing
end

# Let's also test some edge cases and potential issues
@testset "Edge cases and potential issues" begin
    # Test with very small radius
    x1 = [0.0, 0.0]
    x2 = [2.0, 0.0]
    pos = [1.0, 0.0]
    r = 1e-10

    intersection = compute_intersection_point(x1, x2, pos, r)
    @test intersection !== nothing
    @test abs(norm(intersection - pos) - r) < 1e-8

    # Test with zero-length segment (x1 == x2)
    x1 = [1.0, 1.0]
    x2 = [1.0, 1.0]
    pos = [1.0, 1.0]
    r = 0.5

    intersection = compute_intersection_point(x1, x2, pos, r)
    # This case might not work as expected since d = [0,0]
end

println("Running tests...")
