import dubins

start = (0.0, 0.0, 0.0)
end = (4.0, 4.0, 1.57)
turning_radius = 1.0

path = dubins.shortest_path(start, end, turning_radius)
configurations, _ = path.sample_many(0.1)

for q in configurations:
    print(q)  # (x, y, theta)
