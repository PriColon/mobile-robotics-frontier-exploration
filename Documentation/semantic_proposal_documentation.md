## Pipeline Logic Implementations and Callback Flow

**1. Box Filter (Pass-through Filter)**
Removes points outside a 3D workspace box.

**CODE**

def box_filter(self, pts, colors):

    mask = np.all((pts >= self.cfg.box_min) & (pts <= self.cfg.box_max), axis=1)

    filtered_pts = pts[mask]
    filtered_colors = colors[mask]

    return filtered_pts, filtered_colors

**Explanation**
Keeping the points satisfying:
𝑏𝑜𝑥_𝑚𝑖𝑛 ≤ (𝑥,𝑦,𝑧) ≤ 𝑏𝑜𝑥_𝑚𝑎𝑥
box_min≤(x,y,z)≤box_max
Vectorized comparison avoids loops.

**2. Voxel Downsampling**
Reduces cloud density.

**Idea:**
Convert points to voxel indices then keep one point per voxel.

**CODE**

def downsample(self, pts, colors):

    voxel = self.cfg.voxel_size

    voxel_idx = np.floor(pts / voxel).astype(np.int32)
    
    unique_voxels, unique_indices = np.unique(voxel_idx, axis=0, return_index=True)
    
    return pts[unique_indices], colors[unique_indices]

**Explanation**
Voxel coordinate:

𝑣 = ⌊ 𝑝/𝑣𝑜𝑥𝑒𝑙_𝑠𝑖𝑧𝑒 ⌋
v=⌊ p/voxel_size ⌋

np.unique ensures one point per voxel.

**3. Normal Estimation using SVD**

*Steps*
1. Find k neighbors
2. Center neighbors
3. Compute SVD
4. Smallest singular vector = normal

**CODE**

def estimate_normals(self, pts, k=15):
    
    idxs = self.get_neighbors(pts, pts, k)
    
    normals = np.zeros_like(pts)
    
    for i in range(len(pts)):
    
        neighbors = pts[idxs[i]]
        
        centered = neighbors - neighbors.mean(axis=0)
        
        U, S, Vt = np.linalg.svd(centered)
        
        normal = Vt[-1]
        
        normals[i] = normal / np.linalg.norm(normal)
    
    return normals

*Mathematically*
SVD decomposition:

𝑋 = 𝑈Σ𝑉^𝑇
X=UΣV^T

Smallest eigenvector of covariance = surface normal.

**4. Plane RANSAC (Floor Removal)**

*Plane equation*
𝑎𝑥 + 𝑏𝑦 + 𝑐𝑧 + 𝑑 = 0
ax+by+cz+d=0

*Distance from point to plane:*
𝑑 = ∣ 𝑎𝑥 + 𝑏𝑦 + 𝑐𝑧 + 𝑑∣ / sqrt (𝑎^2 + 𝑏^2 + 𝑐^2
d = ∣ax+by+cz+d∣ / sqrt (a^2+b^2+c^2)

*Implementation:*

def find_plane_ransac(self, pts, iters=100):

    best_inliers = []
    
    best_model = None
    
    N = len(pts)
    
    for _ in range(iters):
    
        sample_idx = np.random.choice(N, 3, replace=False)
        
        p1, p2, p3 = pts[sample_idx]
        
        v1 = p2 - p1
        
        v2 = p3 - p1
        
        normal = np.cross(v1, v2)
        
        if np.linalg.norm(normal) == 0:
        
            continue
        normal = normal / np.linalg.norm(normal)
        # Check alignment with expected floor normal
        if np.abs(np.dot(normal, self.cfg.target_normal)) < self.cfg.normal_thresh:
            continue
        
        d = -np.dot(normal, p1)
        distances = np.abs(pts @ normal + d)
        inliers = np.where(distances < self.cfg.floor_dist)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = (normal, d)
    
    return best_model, best_inliers

**5. Euclidean Clustering**

Cluster points based on spatial proximity.

def euclidean_clusters(self, pts, dist_thresh=0.1):
    
    tree = cKDTree(pts)
    
    visited = np.zeros(len(pts), dtype=bool)
    
    clusters = []
    
    for i in range(len(pts)):
        if visited[i]:
            continue
        queue = [i]
        cluster = []
    
        while queue:
            idx = queue.pop()
            if visited[idx]:
                continue
        
            visited[idx] = True
            cluster.append(idx)
            neighbors = tree.query_ball_point(pts[idx], dist_thresh)
            
            for n in neighbors:
                if not visited[n]:
                    queue.append(n)
        clusters.append(np.array(cluster))
    
    return clusters

**6. Cylinder RANSAC**

*Cylinder axis from normals:*

𝑎𝑥𝑖𝑠 = 𝑛1 × 𝑛2
axis= n1 × n2

*Distance from point to axis:*

𝑑=∥ (𝑝−𝑐) − ((𝑝−𝑐)⋅𝑎)𝑎∥
d=∥(p−c)−((p−c)⋅a)a∥

*Implementation:*

def find_single_cylinder(self, pts, normals, iters=300):

    best_inliers = []
    
    best_model = None
    
    N = len(pts)
    
    for _ in range(iters):
        idx = np.random.choice(N, 2, replace=False)
        p1, p2 = pts[idx]
        n1, n2 = normals[idx]
        axis = np.cross(n1, n2)
        if np.linalg.norm(axis) < 1e-6:
            continue
        axis = axis / np.linalg.norm(axis)
    
        # enforce vertical axis
        if np.abs(axis[1]) < 0.8:
            continue
        center = (p1 + p2) / 2
        v = pts - center
        proj = v @ axis
        closest = center + np.outer(proj, axis)
        dist = np.linalg.norm(pts - closest, axis=1)
        inliers = np.where(np.abs(dist - self.cfg.cyl_radius) < 0.02)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = (center, axis, self.cfg.cyl_radius)
    return best_model, best_inliers

**7. Color Classification (HSV)**
Example thresholds.

def classify_color(self, rgb):
    h, s, v = self.rgb_to_hsv(rgb[0], rgb[1], rgb[2])

    if h < 20 or h > 340:
        return "red"
    
    if 80 < h < 160:
        return "green"
    
    if 200 < h < 260:
        return "blue"
    
    if 300 < h < 340:
        return "pink"
    
    return "unknown"

**8. Complete Pipeline Flow (Callback)**

Replace TODO with:

pts_box, colors_box = self.pipeline.box_filter(pts, raw_colors)
pts_v, colors_v = self.pipeline.downsample(pts_box, colors_box)
normals = self.pipeline.estimate_normals(pts_v)
plane_model, plane_inliers = self.pipeline.find_plane_ransac(pts_v)
mask = np.ones(len(pts_v), dtype=bool)
mask[plane_inliers] = False
pts_objects = pts_v[mask]
colors_objects = colors_v[mask]
clusters = self.pipeline.euclidean_clusters(pts_objects)
detected_cylinders = []
for cluster in clusters:
    cluster_pts = pts_objects[cluster]
    cluster_colors = colors_objects[cluster]
    normals_cluster = self.pipeline.estimate_normals(cluster_pts)
    model, inliers = self.pipeline.find_single_cylinder(cluster_pts, normals_cluster)
    if model is None:
        continue
    avg_color = cluster_colors.mean(axis=0)
    label = self.pipeline.classify_color(avg_color)
    detected_cylinders.append((model, avg_color, label))

**9. Expected Output in RViz**

*You should see:*
• Floor removed point cloud
• Separate clusters
• Cylinders drawn using Marker.CYLINDER
• Correct color markers

*Example detected structure:*
Cylinder 1 → Green
Cylinder 2 → Red
Cylinder 3 → Blue

*Bonus bag:*
Pink cylinder detected

**10. RViz Topics to Visualize**

*Add these displays:*
/pipeline/stage0_box
/pipeline/stage3_candidates
/viz/detections

*Fixed Frame:*
oakd_rgb_camera_optical_frame

**11. Runtime Performance Tips**

*To keep 0.5x rosbag speed:*

• Use voxel_size = 0.02
• Limit RANSAC iterations
• Avoid Python loops except small ones
• Prefer NumPy broadcasting
