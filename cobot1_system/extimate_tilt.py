def estimate_tilt_from_taper(mask, n_slices=10):
    """
    마스크의 장축을 따라 폭이 어떻게 변하는지 보고 기울기 추정.
    한쪽이 좁아진 정도(taper) → 기울기 각도.
    """
    import numpy as np
    centroid, angle_deg, elong = mask_pca_angle(mask)
    if centroid is None:
        return None

    # 장축 방향 단위벡터
    a = np.radians(angle_deg)
    major = np.array([np.cos(a), np.sin(a)])
    minor = np.array([-np.sin(a), np.cos(a)])

    # 마스크 픽셀 좌표
    ys, xs = np.nonzero(mask)
    pts = np.stack([xs, ys], axis=1) - centroid

    # 장축 좌표 (s), 단축 좌표 (t)
    s = pts @ major
    t = pts @ minor

    # 장축을 N개 구간으로 나누고, 각 구간의 단축 폭(t의 표준편차) 측정
    s_min, s_max = s.min(), s.max()
    edges = np.linspace(s_min, s_max, n_slices + 1)
    widths = []
    centers = []
    for i in range(n_slices):
        mask_slice = (s >= edges[i]) & (s < edges[i+1])
        if mask_slice.sum() < 3:
            continue
        widths.append(t[mask_slice].std() * 2)  # 폭 ≈ 2σ
        centers.append((edges[i] + edges[i+1]) / 2)

    if len(widths) < 3:
        return None

    # 폭의 선형 회귀 기울기 → 좌우 비대칭(taper) 정도
    centers = np.array(centers)
    widths = np.array(widths)
    slope, intercept = np.polyfit(centers, widths, 1)
    mean_width = widths.mean()

    # taper 비율: 한쪽 끝 폭 / 다른쪽 끝 폭
    w_near = intercept + slope * s_min  # 카메라에 가까운 쪽(보통 더 넓음)
    w_far  = intercept + slope * s_max
    taper_ratio = min(w_near, w_far) / max(w_near, w_far)

    # taper_ratio = 1.0 → 기울기 없음
    # taper_ratio = 0.5 → 많이 기울어짐
    # 근사: cos(θ) ≈ taper_ratio (광축 기준 기울기)
    tilt_deg = np.degrees(np.arccos(np.clip(taper_ratio, 0.1, 1.0)))

    # 어느 쪽이 더 멀리(아래로) 있는지: slope 부호로 판단
    far_side = "positive_major" if w_far < w_near else "negative_major"

    return {
        "tilt_deg": tilt_deg,
        "taper_ratio": taper_ratio,
        "tilt_axis_image_deg": angle_deg + 90,  # 기울어진 축(장축에 수직)
        "far_side": far_side,                    # 어느 쪽이 더 멀리 있는지
    }

import numpy as np

def mask_pca_angle(mask: np.ndarray):
    ys, xs = np.nonzero(mask)
    if len(xs) < 5:
        return None
    pts = np.stack([xs, ys], axis=1).astype(float)
    centroid = pts.mean(axis=0)
    cov = np.cov((pts - centroid).T)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, -1]
    
    raw_angle = np.degrees(np.arctan2(major[1], major[0]))
    
    # 정규화: (-90, 90) 로 맞춤
    angle_deg = raw_angle % 180
    if angle_deg > 90:
        angle_deg -= 180
    
    elong = float(np.sqrt(evals[-1] / max(evals[0], 1e-6)))
    return (centroid, angle_deg, elong)