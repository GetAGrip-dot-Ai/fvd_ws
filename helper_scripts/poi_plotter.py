import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from sklearn.mixture import GaussianMixture
from sklearn.cluster import DBSCAN
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib as mpl

peduncles= np.genfromtxt(fname="assets/peduncles_10262023.csv", delimiter=",")
peppers= np.genfromtxt(fname="assets/peppers_10262023.csv", delimiter=",")

min_peppers, max_peppers = min(peppers[:, -1]), max(peppers[:, -1])
min_peduncles, max_peduncles = min(peduncles[:, -1]), max(peduncles[:, -1])

peduncles = peduncles[:, :-1].T
peppers = peppers[:, :-1].T

# import pdb; pdb.set_trace()
peduncles = peduncles[:, (peduncles[0] > 0.3)]

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_ellipsoid_3d(ax, center, cov_matrix, nstd=1., color='blue', alpha=0.3):
    """Plot 3D ellipsoid on a given ax."""
    # Find the eigenvalues and eigenvectors
    eigvals, eigvecs = np.linalg.eigh(cov_matrix)
    
    # Define a rotation based on eigenvectors
    rotation = np.dot(eigvecs, np.diag(np.sqrt(eigvals)))

    # Generate samples on the unit sphere
    phi = np.linspace(0, 2 * np.pi, 50)
    theta = np.linspace(0, np.pi, 50)
    PHI, THETA = np.meshgrid(phi, theta)
    X = np.sin(THETA) * np.cos(PHI)
    Y = np.sin(THETA) * np.sin(PHI)
    Z = np.cos(THETA)
    
    # Apply the rotations and scaling
    data = np.array([X.flatten(), Y.flatten(), Z.flatten()])
    rotated_data = np.dot(rotation, data * nstd)
    
    # Add the center offset and reshape
    rotated_data = rotated_data + center[:, np.newaxis]
    rotated_data = rotated_data.reshape(3, X.shape[0], X.shape[1])
    
    # Plot
    ax.plot_surface(rotated_data[0], rotated_data[1], rotated_data[2],
                    rstride=2, cstride=2, color=color, alpha=alpha, linewidth=0)
    # ax.scatter(*center, color='red', s=100, label='Center')

min_peduncle_clusters = GaussianMixture(n_components=int(min_peduncles))
max_peduncle_clusters = GaussianMixture(n_components=int(max_peduncles))

min_peduncle_clusters.fit(peduncles.T)
max_peduncle_clusters.fit(peduncles.T)

fig1 = plt.figure()
ax1 = fig1.add_subplot(projection='3d')
ax1.scatter(*peduncles)

lim = max(np.max(peduncles, axis=1))

print(
f"""
==================================================================
Range X: {np.ptp(peduncles[0])}
Range Y: {np.ptp(peduncles[1])}
Range Z: {np.ptp(peduncles[2])}
==================================================================
"""
)


print(
"""
==================================================================
                    GMM MINIMUM DETECTIONS       
=================================================================="""
)
for exp, cov in zip(min_peduncle_clusters.means_, min_peduncle_clusters.covariances_):
    plot_ellipsoid_3d(ax1, exp, cov, nstd=3, color='red', alpha=0.1)
    plot_ellipsoid_3d(ax1, exp, cov, nstd=2, color='grey',alpha=0.2)
    plot_ellipsoid_3d(ax1, exp, cov, nstd=1, color='green', alpha=0.3)
    print(f"Mean: \n{exp}")
    print(f"Cov: \n{np.matrix(cov)}")
    
    print("""===============================================================""")

ax1.set_title(f"Minimum Peduncles Detected {min_peduncles}")
ax1.set_xlabel("x-axis (m)")
ax1.set_ylabel("y-axis (m)")
ax1.set_zlabel("z-axis (m)")
ax1.set_xlim(0, lim)
ax1.set_ylim(0, lim)
ax1.set_zlim(0, lim)
ax1.set_aspect("equal")


fig2 = plt.figure()
ax2 = fig2.add_subplot(projection='3d')
ax2.scatter(*peduncles)

ax2.set_title(f"Maximum Peduncles Detected {max_peduncles}")
ax2.set_xlabel("x-axis (m)")
ax2.set_ylabel("y-axis (m)")
ax2.set_zlabel("z-axis (m)")
ax2.set_xlim(0, lim)
ax2.set_ylim(0, lim)
ax2.set_zlim(0, lim)
ax2.set_aspect("equal")

print(
"""
==================================================================
                    GMM MAXIMUM DETECTIONS       
=================================================================="""
)

for exp, cov in zip(max_peduncle_clusters.means_, max_peduncle_clusters.covariances_):
    plot_ellipsoid_3d(ax2, exp, cov, nstd=3, color='red', alpha=0.1)
    plot_ellipsoid_3d(ax2, exp, cov, nstd=2, color='grey', alpha=0.2)
    plot_ellipsoid_3d(ax2, exp, cov, nstd=1, color='green', alpha=0.3)

    print(f"Mean: \n{exp}")
    print(f"Cov: \n{np.matrix(cov)}")

    print("""===============================================================""")
    
    
# import pdb; pdb.set_trace()
db = DBSCAN(eps=0.1, min_samples=5).fit(peduncles.T)
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print(
"""
==================================================================
                    DBSCAN DETECTIONS       
=================================================================="""
)

print("\nEstimated number of clusters: %d" % n_clusters_)
print("Estimated number of noise points: %d\n" % n_noise_)

unique_labels = set(labels)
core_samples_mask = np.zeros_like(labels, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True

legend = []
for i in unique_labels:
    if i == -1:
        legend.append("Noise")
    else:
        legend.append(f"Cluster {i}")

for name, label in zip(legend, unique_labels):
    print(f"Mean {name}: {np.mean(peduncles.T[labels == label, :], axis=0)}")

fig3 = plt.figure()
ax3 = fig3.add_subplot(projection='3d')

c = db.labels_


scatter = ax3.scatter(peduncles.T[:,0], peduncles.T[:,1], peduncles.T[:,2], 
            c=db.labels_, cmap='jet', s=300)

ax3.set_title(f"DBSCAN Results")
ax3.set_xlabel("x-axis (m)")
ax3.set_ylabel("y-axis (m)")
ax3.set_zlabel("z-axis (m)")
ax3.set_xlim(0, lim)
ax3.set_ylim(0, lim)
ax3.set_zlim(0, lim)
ax3.set_aspect("equal")

cbar = fig3.colorbar(scatter)
cbar.set_ticks(tuple(unique_labels))
cbar.set_ticklabels(legend)
    
plt.show()