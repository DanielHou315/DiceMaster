# smallest_enclosing_square_cvxpy_plot.py
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

# ---------- geometry helpers ----------
def basis(theta: float):
    """Return square's unit axes u, v and rotation matrix R=[u v]."""
    u = np.array([np.cos(theta), np.sin(theta)])
    v = np.array([-np.sin(theta), np.cos(theta)])
    R = np.column_stack((u, v))  # world_from_local
    return u, v, R

def square_corners(c: np.ndarray, s: float, R: np.ndarray):
    """Four square corners in CCW order (world coords)."""
    half = 0.5 * s
    locals_ = np.array([[-half, -half],
                        [ half, -half],
                        [ half,  half],
                        [-half,  half]])
    return c + locals_ @ R.T

def to_local(points: np.ndarray, c: np.ndarray, R: np.ndarray):
    """Local coords of points in the square frame (origin at center)."""
    return (np.asarray(points, float) - c) @ R

# ---------- (optional) convex hull ----------
def monotone_chain_hull(pts):
    """Convex hull in CCW; returns (m,2) ndarray (duplicates removed)."""
    P = np.unique(np.asarray(pts, dtype=float), axis=0)
    if len(P) <= 2:
        return P
    P = P[np.lexsort((P[:,1], P[:,0]))]
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in P:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(tuple(p))
    upper = []
    for p in P[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(tuple(p))
    return np.array(lower[:-1] + upper[:-1], dtype=float)

# ---------- CVXPY subproblem for fixed theta ----------
def minimize_square_fixed_theta(points, theta, solver="ECOS"):
    """
    Solve:  minimize s
            subject to |u·(p - c)| <= s/2 and |v·(p - c)| <= s/2 for all p.
    Returns (s, c, status).
    """
    u, v, R = basis(theta)
    c = cp.Variable(2)
    s = cp.Variable(nonneg=True)
    cons = []
    for p in np.asarray(points, float):
        cons += [cp.abs(u @ (p - c)) <= 0.5 * s,
                 cp.abs(v @ (p - c)) <= 0.5 * s]
    prob = cp.Problem(cp.Minimize(s), cons)
    prob.solve(solver=solver, verbose=False)
    return float(s.value), np.array(c.value), prob.status

# ---------- outer search over theta ----------
def smallest_enclosing_square(points,
                              use_hull=True,
                              coarse_steps=720,
                              refinements=3,
                              window=6,
                              solver="ECOS"):
    """
    Returns dict with:
      side, theta_rad/deg, center, R_world_from_local, corners_world_ccw,
      w, h, points_local_center_frame.
    """
    P_all = np.asarray(points, float)
    P = monotone_chain_hull(P_all) if use_hull else P_all

    # Only need theta ∈ [0, π/2) due to square symmetry.
    thetas = np.linspace(0.0, 0.5*np.pi, coarse_steps, endpoint=False)

    best = {"side": np.inf}
    def eval_list(theta_list):
        nonlocal best
        for th in theta_list:
            s, c, status = minimize_square_fixed_theta(P, th, solver=solver)
            if status not in ("optimal", "optimal_inaccurate"):
                continue
            if s < best["side"]:
                u, v, R = basis(th)
                # Spans of hull along the oriented axes (for labeling w,h)
                alphas = P @ u
                betas  = P @ v
                a_min, a_max = alphas.min(), alphas.max()
                b_min, b_max = betas.min(),  betas.max()
                w = a_max - a_min
                h = b_max - b_min

                best = {
                    "side": s,
                    "theta_rad": th,
                    "theta_deg": float(np.degrees(th)),
                    "center": c,
                    "R_world_from_local": R,
                    "w": float(w),
                    "h": float(h),
                }
        return best

    eval_list(thetas)
    # Coarse-to-fine refinements around current best
    for k in range(refinements):
        step = (0.5*np.pi) / coarse_steps / (10**k)
        th = best["theta_rad"]
        sweep = (th + step * np.arange(-window, window+1)) % (0.5*np.pi)
        eval_list(sweep)

    # Corners and local coordinates of all original points
    R = best["R_world_from_local"]
    c = best["center"]
    s = best["side"]
    best["corners_world_ccw"] = square_corners(c, s, R)
    best["points_local_center_frame"] = to_local(P_all, c, R)
    return best

# ---------- plotting ----------
def plot_square_and_points(points, res, title=None, show_axes=True, label_lengths=True):
    P = np.asarray(points, float)
    corners = res["corners_world_ccw"]
    c = res["center"]
    R = res["R_world_from_local"]
    u, v = R[:,0], R[:,1]
    s, w, h = res["side"], res["w"], res["h"]

    # midpoints in projection space for w/h labels
    alphas = P @ u
    betas  = P @ v
    a_min, a_max = alphas.min(), alphas.max()
    b_min, b_max = betas.min(),  betas.max()
    a_mid = 0.5*(a_min+a_max)
    b_mid = 0.5*(b_min+b_max)

    fig, ax = plt.subplots(figsize=(7,7))
    # Plot points in yellow
    ax.scatter(P[:,0], P[:,1], color='yellow', label="points")

    # Plot convex hull in yellow
    hull = monotone_chain_hull(P)
    hull_loop = np.vstack([hull, hull[0]])
    ax.plot(hull_loop[:,0], hull_loop[:,1], color='yellow', linewidth=2, label="convex hull")

    loop = np.vstack([corners, corners[0]])
    ax.plot(loop[:,0], loop[:,1], linewidth=2, label="enclosing square")
    ax.scatter([c[0]],[c[1]], marker="x", s=80, label="center")

    if show_axes:
        L = 0.6*s if s > 0 else 1.0
        ax.arrow(c[0], c[1], L*u[0], L*u[1], head_width=0.0, length_includes_head=True)
        ax.text(*(c + 1.05*L*u), "u", fontsize=10)
        ax.arrow(c[0], c[1], L*v[0], L*v[1], head_width=0.0, length_includes_head=True)
        ax.text(*(c + 1.05*L*v), "v", fontsize=10)

    if label_lengths:
        # Label s on an edge
        p1, p2 = corners[1], corners[2]
        ax.annotate(f"s = {s:.4g}", xy=p1, xytext=p2,
                    arrowprops=dict(arrowstyle="<->"), ha="center", va="bottom")

        # Label w along u at beta=b_mid
        A0 = a_min*u + b_mid*v
        A1 = a_max*u + b_mid*v
        ax.plot([A0[0], A1[0]], [A0[1], A1[1]], linestyle="--")
        ax.annotate(f"w = {w:.4g}", xy=A0, xytext=A1,
                    arrowprops=dict(arrowstyle="<->"), ha="center")

        # Label h along v at alpha=a_mid
        B0 = a_mid*u + b_min*v
        B1 = a_mid*u + b_max*v
        ax.plot([B0[0], B1[0]], [B0[1], B1[1]], linestyle="--")
        ax.annotate(f"h = {h:.4g}", xy=B0, xytext=B1,
                    arrowprops=dict(arrowstyle="<->"), va="center")

    ttl = title or f"Smallest Enclosing Square (θ = {res['theta_deg']:.2f}°)"
    ax.set_title(ttl)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend(loc="best")
    plt.show()
    return fig, ax

# ---------- example usage ----------
if __name__ == "__main__":
    # Replace with your own list of (x, y)
    pts = [
        (0.0, 0.0), (65, 0.0), (65, 10),
        (60, 35), (5, 35), (0, 10)
    ]

    res = smallest_enclosing_square(pts, use_hull=True, solver="ECOS")
    print("side:", res["side"])
    print("theta (deg):", res["theta_deg"])
    print("center:", res["center"])
    print("corners (world CCW):\n", res["corners_world_ccw"])
    print("first 3 points in local coords:\n", res["points_local_center_frame"][:3])

    plot_square_and_points(pts, res)
