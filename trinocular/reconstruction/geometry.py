"""Rigid registration with overlap gates and globally optimized object poses."""
import numpy as np
import open3d as o3d

REG = o3d.pipelines.registration


def rigid_transform(matrix):
    t = np.asarray(matrix, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("Pose must be a finite 4x4 matrix")
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError("Invalid homogeneous pose")
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1, atol=1e-5):
        raise ValueError("Pose must be rigid: scaling, shear, and reflection are forbidden")
    return t


def from_correspondences(sensor_points, object_points):
    """At least three noncollinear measured markers; preserves metric scale."""
    x, y = (np.asarray(a, dtype=float) for a in (sensor_points, object_points))
    if x.ndim != 2 or x.shape != y.shape or x.shape[1] != 3 or len(x) < 3:
        raise ValueError("Need at least three matching 3D markers")
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("Markers must be finite")
    a, b = x - x.mean(0), y - y.mean(0)
    if min(np.linalg.matrix_rank(a), np.linalg.matrix_rank(b)) < 2:
        raise ValueError("Markers must not be collinear")
    u, _, vt = np.linalg.svd(a.T @ b)
    fix = np.eye(3)
    fix[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ fix @ u.T
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, y.mean(0) - r @ x.mean(0)
    return rigid_transform(t)


def transform_points(points, pose):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]


def cloud(points, normals=None):
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    if normals is not None:
        p.normals = o3d.utility.Vector3dVector(normals)
    return p


def prepare(points, normals, voxel):
    p = cloud(points, normals).voxel_down_sample(voxel)
    if len(p.points) < 30:
        raise ValueError("Too few foreground samples at this voxel size")
    if not p.has_normals():
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=40))
        p.orient_normals_towards_camera_location(np.zeros(3))
    p.normalize_normals()
    return p


def pairwise(source, target, voxel, initial, min_overlap, max_rmse):
    """FPFH/RANSAC initialization when no object-pose seed was supplied."""
    if initial is None:
        def feature(p):
            return REG.compute_fpfh_feature(p, o3d.geometry.KDTreeSearchParamHybrid(radius=5 * voxel, max_nn=100))
        result = REG.registration_ransac_based_on_feature_matching(
            source, target, feature(source), feature(target), True, 2 * voxel,
            REG.TransformationEstimationPointToPoint(False), 3,
            [REG.CorrespondenceCheckerBasedOnEdgeLength(.9),
             REG.CorrespondenceCheckerBasedOnDistance(2 * voxel)],
            REG.RANSACConvergenceCriteria(30000, .999))
        initial = result.transformation
    pose = rigid_transform(initial)
    for radius in (3 * voxel, 1.5 * voxel, voxel):
        result = REG.registration_icp(
            source, target, radius, pose,
            REG.TransformationEstimationPointToPlane(REG.TukeyLoss(k=radius)),
            REG.ICPConvergenceCriteria(max_iteration=60))
        pose = rigid_transform(result.transformation)
    forward = REG.evaluate_registration(source, target, 1.5 * voxel, pose)
    backward = REG.evaluate_registration(target, source, 1.5 * voxel, np.linalg.inv(pose))
    overlap = min(forward.fitness, backward.fitness)
    rmse = max(forward.inlier_rmse, backward.inlier_rmse)
    count = min(len(forward.correspondence_set), len(backward.correspondence_set))
    accepted = overlap >= min_overlap and rmse <= max_rmse and count >= 30
    info = REG.get_information_matrix_from_point_clouds(source, target, 1.5 * voxel, pose)
    return pose, info, {"accepted": bool(accepted), "bidirectional_overlap": float(overlap),
                        "inlier_rmse_mm": float(rmse * 1000), "correspondences": count}


def align(clouds, voxel, seeds=None, min_overlap=.35, max_rmse_m=None):
    """Align masked object observations, anchor pass zero, reject disconnected sets.

    A supplied seed is object_from_sensor. Output poses are first_sensor_from_sensor.
    A marker frame attached to the stationary room is not an object-pose seed.
    """
    n = len(clouds)
    if n < 2:
        raise ValueError("Reconstruction requires at least two object orientations")
    if not 0 < min_overlap <= 1 or voxel <= 0:
        raise ValueError("Invalid registration configuration")
    seeds = seeds or [None] * n
    if len(seeds) != n:
        raise ValueError("One optional seed per pass required")
    max_rmse_m = max_rmse_m if max_rmse_m is not None else voxel
    if max_rmse_m <= 0:
        raise ValueError("Registration RMSE gate must be positive")
    edges, diagnostics = [], []
    for i in range(n):
        for j in range(i + 1, n):
            seed = None if seeds[i] is None or seeds[j] is None else np.linalg.inv(seeds[j]) @ seeds[i]
            t, info, quality = pairwise(clouds[i], clouds[j], voxel, seed, min_overlap, max_rmse_m)
            diagnostics.append({"source": i, "target": j, **quality})
            if quality['accepted']:
                edges.append((i, j, t, info, quality))
    # A quality-ordered spanning tree initializes global poses; other edges are
    # uncertain loop closures and may be pruned by the optimizer.
    poses, tree = {0: np.eye(4)}, set()
    for _ in range(n - 1):
        options = [e for e in edges if (e[0] in poses) != (e[1] in poses)]
        if not options:
            raise ValueError(f"Disconnected observations; rescan or supply object markers. Connected passes: {sorted(poses)}")
        i, j, t, _, _ = max(options, key=lambda e: e[4]['bidirectional_overlap'])
        if i in poses:
            poses[j] = poses[i] @ np.linalg.inv(t)
        else:
            poses[i] = poses[j] @ t
        tree.add((i, j))
    graph = REG.PoseGraph()
    for i in range(n):
        graph.nodes.append(REG.PoseGraphNode(poses[i]))
    for i, j, t, info, _ in edges:
        graph.edges.append(REG.PoseGraphEdge(i, j, t, info, uncertain=(i, j) not in tree))
    REG.global_optimization(graph, REG.GlobalOptimizationLevenbergMarquardt(),
                            REG.GlobalOptimizationConvergenceCriteria(),
                            REG.GlobalOptimizationOption(max_correspondence_distance=1.5 * voxel,
                                                         edge_prune_threshold=.25, reference_node=0))
    output = [rigid_transform(node.pose) for node in graph.nodes]
    # Recheck the accepted tree after optimization; an attractive pairwise fit
    # must not silently become a poor global fit.
    for i, j in sorted(tree):
        t = np.linalg.inv(output[j]) @ output[i]
        check = REG.evaluate_registration(clouds[i], clouds[j], 1.5 * voxel, t)
        reverse = REG.evaluate_registration(clouds[j], clouds[i], 1.5 * voxel, np.linalg.inv(t))
        if min(check.fitness, reverse.fitness) < min_overlap or max(check.inlier_rmse, reverse.inlier_rmse) > max_rmse_m:
            raise ValueError("Global optimization failed the registration quality gates")
    return output, diagnostics


def fuse(points, linear_rgb, pass_ids, voxel):
    """Voxel averaging in linear light; retain support count by independent pass."""
    if voxel <= 0:
        raise ValueError("Fusion voxel must be positive")
    keys = np.floor(np.asarray(points) / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse)
    def mean(values):
        return np.column_stack([np.bincount(inverse, weights=values[:, k]) / count for k in range(3)])
    pairs = np.unique(np.column_stack([inverse, pass_ids]), axis=0)
    views = np.bincount(pairs[:, 0], minlength=len(count))
    return mean(points), mean(linear_rgb), views


def mesh_observations(points, linear_rgb, voxel):
    """Ball pivoting leaves unsupported holes; return an observational mesh."""
    from .color import linear_to_srgb
    p = cloud(points)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=40))
    p.orient_normals_consistent_tangent_plane(min(30, len(points) - 1))
    p.colors = o3d.utility.Vector3dVector(linear_to_srgb(linear_rgb))
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        p, o3d.utility.DoubleVector([2 * voxel, 4 * voxel]))
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh
