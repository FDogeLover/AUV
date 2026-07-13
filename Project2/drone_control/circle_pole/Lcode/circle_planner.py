"""环绕飞行航点生成器 — 纯函数，不依赖飞控/雷达对象，方便独立单元测试。"""
import math


def generate_circle_waypoints(center_x, center_y, cur_x, cur_y,
                               radius=0.5, n_points=6, direction="cw", z=1.5):
    """生成围绕(center_x, center_y)半径radius的环绕航点列表(世界系[x,y,z])。

    起始点取"当前位置->圆心连线"与圆的交点(离当前位置最近的圆上一点)，避免
    先横穿到圆上任意一点。direction="cw"(顺时针,顶视)/"ccw"(逆时针,顶视)。
    返回n_points+1个点：绕满一圈(n_points个等分点)后再重复第一个点，确保
    闭合>=360度(否则n_points个点首尾不重合，实际只转了(n_points-1)/n_points圈)。
    """
    if n_points < 3:
        raise ValueError("n_points must be >= 3")
    if direction not in ("cw", "ccw"):
        raise ValueError("direction must be 'cw' or 'ccw'")

    dx = cur_x - center_x
    dy = cur_y - center_y
    dist = math.hypot(dx, dy)
    start_angle = math.atan2(dy, dx) if dist > 1e-6 else 0.0

    sign = -1.0 if direction == "cw" else 1.0
    step = sign * (2 * math.pi / n_points)

    points = []
    for i in range(n_points):
        angle = start_angle + step * i
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append([x, y, z])
    points.append(list(points[0]))
    return points
