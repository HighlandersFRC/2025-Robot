import json
import matplotlib.pyplot as plt

# === Load your path JSON ===
with open("src/main/deploy/Square.polarauto", "r") as f:
    data = json.load(f)

# === Pull out the first path's sampled points ===
sampled_points = data["paths"][0]["sampled_points"]

# === Extract values ===
times = []
curvatures = []

for pt in sampled_points:
    t = pt["time"]
    x_vel = pt["x_velocity"]
    y_vel = pt["y_velocity"]
    x_accel = pt["x_acceleration"]
    y_accel = pt["y_acceleration"]

    # Calculate curvature
    dx = x_vel
    dy = y_vel
    ddx = x_accel
    ddy = y_accel

    numerator = abs(dx * ddy - dy * ddx)
    denominator = (dx**2 + dy**2) ** 1.5

    curvature = numerator / denominator if denominator > 1e-6 else 0.0

    times.append(t)
    curvatures.append(curvature)

# === Plot ===
plt.figure(figsize=(10, 5))
plt.plot(times, curvatures, label="Curvature")
plt.xlabel("Time (s)")
plt.ylabel("Curvature (1/m)")
plt.title("Curvature vs Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
plt.savefig("curvature_plot.png")
