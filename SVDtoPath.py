from svgpathtools import svg2paths
import numpy as np
import matplotlib.pyplot as plt

# Load SVG and extract paths
paths, attributes = svg2paths('C:/Users/eweissm1/Visual Studio Projects/HARP Continuum Robot/RobotLineArt/ASUlogo.svg')

# Discretize the first path (you can loop through multiple if needed)
path = paths[0]

Scale = .1/200.0

# Sample N points along the path
N = 500
points = [path.point(t) for t in np.linspace(0, 1, N)]
x = [Scale*p.real-.05 for p in points]
y = [Scale*p.imag-.05 for p in points]


# Optional: flip y-axis if needed (SVG y-axis is top-down)
y = [-yi for yi in y]

# Plot
plt.plot(x, y)
plt.axis('equal')
plt.title("Discretized Path")
plt.show()

# Save as coordinates
coords = list(zip(x, y))