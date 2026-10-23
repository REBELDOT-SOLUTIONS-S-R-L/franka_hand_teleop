"""Validate position-only fake-controller paths using the loaded URDF's kinematics."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .core import finite


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross @ cross)


def origin_transform(element):
    result = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        roll, pitch, yaw = [float(v) for v in element.get("rpy", "0 0 0").split()]
        result[:3, 3] = xyz
        result[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
    return result


def motion_matrices(kind, axis, values):
    """The scalar FK's joint transform, evaluated for all curve samples."""
    values = np.asarray(values)
    result = np.broadcast_to(np.eye(4), (len(values), 4, 4)).copy()
    axis = np.asarray(axis, dtype=float)
    if kind == "revolute":
        axis = axis/np.linalg.norm(axis)
        x, y, z = axis
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        result[:, :3, :3] += (np.sin(values)[:, None, None]*cross +
                              (1-np.cos(values))[:, None, None]*(cross @ cross))
    elif kind == "prismatic":
        result[:, :3, 3] = values[:, None]*axis
    return result


def mesh_bounds(mesh):
    filename = mesh.get("filename")
    if filename.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory
        package, relative = filename[len("package://"):].split("/", 1)
        path = Path(get_package_share_directory(package)) / relative
    else:
        path = Path(filename.removeprefix("file://"))
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little") if len(data) >= 84 else 0
    if len(data) == 84+count*50 and count:
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        vertices = np.frombuffer(data, dtype=dtype, offset=84, count=count)["vertices"].reshape(-1, 3)
    else:
        vertices = np.array([[float(v) for v in line.split()[1:]]
                             for line in data.decode().splitlines() if line.strip().startswith("vertex ")])
    vertices = vertices * np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError(f"Invalid collision mesh: {path}")
    return vertices.min(axis=0), vertices.max(axis=0)


class WorkspaceGuard:
    def __init__(self, urdf, limits, base="fr3_link0", tip="fr3_hand_tcp"):
        root = ET.fromstring(urdf)
        by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        chain = []
        while tip != base:
            joint = by_child[tip]
            chain.append(joint)
            tip = joint.find("parent").get("link")
        self.chain, self.names, self.bounds = [], [], {}
        reach = 0.0
        for joint in reversed(chain):
            origin = joint.find("origin")
            xyz = [float(x) for x in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(x) for x in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (rotation([0, 0, 1], rpy[2]) @
                                   rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0]))
            reach += np.linalg.norm(xyz)
            name, kind = joint.get("name"), joint.get("type")
            axis_node = joint.find("axis")
            axis = [float(x) for x in (axis_node.get("xyz") if axis_node is not None else "1 0 0").split()]
            if kind != "fixed":
                if kind not in ("revolute", "prismatic"):
                    raise ValueError(f"Unsupported workspace-guard joint type: {kind}")
                self.names.append(name)
                bound = joint.find("limit")
                self.bounds[name] = (float(bound.get("lower")), float(bound.get("upper")))
            self.chain.append((name, kind, transform, axis))
        self.reach = max(1.0, float(reach))
        self.limits = limits
        self.base = base
        self.boxes = []
        needed = {base}
        for link in root.findall("link"):
            name = link.get("name")
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                mesh, box = geometry.find("mesh"), geometry.find("box")
                sphere, cylinder = geometry.find("sphere"), geometry.find("cylinder")
                if mesh is not None:
                    low, high = mesh_bounds(mesh)
                elif box is not None:
                    high = np.array([float(v)/2 for v in box.get("size").split()])
                    low = -high
                elif sphere is not None:
                    high = np.full(3, float(sphere.get("radius")))
                    low = -high
                elif cylinder is not None:
                    high = np.array([float(cylinder.get("radius"))]*2 + [float(cylinder.get("length"))/2])
                    low = -high
                else:
                    raise ValueError(f"Unsupported collision geometry on {name}")
                origin = origin_transform(collision.find("origin"))
                center = origin[:3, :3] @ ((low+high)/2) + origin[:3, 3]
                half = np.abs(origin[:3, :3]) @ ((high-low)/2)
                # Enclose the entire 0–40 mm finger travel, not only its current opening.
                finger_pad = .02 if "finger" in name else 0.0
                self.boxes.append((name, center, half, finger_pad))
                ancestor = name
                while ancestor != base:
                    needed.add(ancestor)
                    ancestor = by_child[ancestor].find("parent").get("link")
        self.body_joints = []
        added = {base}
        while needed-added:
            before = len(added)
            for child in needed-added:
                joint = by_child[child]
                parent = joint.find("parent").get("link")
                if parent not in added:
                    continue
                kind, name = joint.get("type"), joint.get("name")
                axis_node = joint.find("axis")
                axis = np.array([float(v) for v in (axis_node.get("xyz") if axis_node is not None else "1 0 0").split()])
                mid = 0.0
                if kind == "prismatic" and name not in self.names:
                    limit = joint.find("limit")
                    mid = (float(limit.get("lower"))+float(limit.get("upper")))/2
                self.body_joints.append((name, kind, parent, child, origin_transform(joint.find("origin")), axis, mid))
                added.add(child)
            if len(added) == before:
                raise ValueError("Collision links do not connect to workspace base")
        if self.boxes:
            self.reach += max(float(np.linalg.norm(center)+np.linalg.norm(half)+pad)
                              for _, center, half, pad in self.boxes)

    def pose(self, positions):
        pose = np.eye(4)
        for name, kind, origin, axis in self.chain:
            pose = pose @ origin
            if kind != "fixed":
                motion = np.eye(4)
                if kind == "revolute":
                    motion[:3, :3] = rotation(axis, positions[name])
                else:
                    motion[:3, 3] = np.asarray(axis)*positions[name]
                pose = pose @ motion
        return pose

    def tcp(self, positions):
        return tuple(float(v) for v in self.pose(positions)[:3, 3])

    def inside(self, point, pad=0.001):
        return all(a+pad <= p <= b-pad for a, p, b in
                   zip(self.limits.minimum, point, self.limits.maximum))

    def body_bounds(self, positions):
        frames = {self.base: np.eye(4)}
        for name, kind, parent, child, origin, axis, mid in self.body_joints:
            motion = np.eye(4)
            if kind == "revolute":
                motion[:3, :3] = rotation(axis, positions[name])
            elif kind == "prismatic":
                motion[:3, 3] = axis*(positions[name] if name in self.names else mid)
            frames[child] = frames[parent] @ origin @ motion
        bounds = []
        for name, center, half, pad in self.boxes:
            transform = frames[name]
            center_world = transform[:3, :3] @ center + transform[:3, 3]
            half_world = np.abs(transform[:3, :3]) @ half + pad
            bounds.append((name, center_world-half_world, center_world+half_world))
        return bounds

    def body_inside(self, positions, pad=.001):
        low, high = np.array(self.limits.body_minimum), np.array(self.limits.body_maximum)
        for name, lower, upper in self.body_bounds(positions):
            if np.any(lower < low+pad) or np.any(upper > high-pad):
                return False, f"{name} would leave whole-arm workspace"
        return True, "Whole arm inside workspace"

    def samples_inside(self, samples, pad=.001, height_check=None):
        """Check identical TCP/body sample positions in a batch, with no coarsening."""
        samples = np.asarray(samples, dtype=float)
        if (samples.ndim != 2 or not len(samples) or samples.shape[1] != len(self.names)
                or not np.isfinite(samples).all()):
            return False
        positions = dict(zip(self.names, samples.T))
        pose = np.broadcast_to(np.eye(4), (len(samples), 4, 4))
        for name, kind, origin, axis in self.chain:
            pose = pose @ origin
            if kind != "fixed":
                pose = pose @ motion_matrices(kind, axis, positions[name])
        tcp = pose[:, :3, 3]
        if height_check is not None and not height_check(tcp[:, 2]):
            return False
        if (np.any(tcp < np.asarray(self.limits.minimum)+pad) or
                np.any(tcp > np.asarray(self.limits.maximum)-pad)):
            return False
        frames = {self.base: np.broadcast_to(np.eye(4), (len(samples), 4, 4))}
        for name, kind, parent, child, origin, axis, mid in self.body_joints:
            frames[child] = frames[parent] @ origin
            if kind != "fixed":
                values = positions[name] if name in positions else np.full(len(samples), mid)
                frames[child] = frames[child] @ motion_matrices(kind, axis, values)
        low, high = np.asarray(self.limits.body_minimum), np.asarray(self.limits.body_maximum)
        for name, center, half, finger_pad in self.boxes:
            transform = frames[name]
            center_world = transform[:, :3, :3] @ center + transform[:, :3, 3]
            half_world = np.abs(transform[:, :3, :3]) @ half + finger_pad
            if (np.any(center_world-half_world < low+pad) or
                    np.any(center_world+half_world > high-pad)):
                return False
        return True

    def check(self, current, waypoints):
        if not waypoints or len(waypoints) > 128:
            return False, "Invalid trajectory length"
        if not all(name in current and finite(current[name]) for name in self.names):
            return False, "Missing current joint feedback"
        previous = np.array([current[name] for name in self.names])
        if not self.inside(self.tcp(current), pad=0):
            return False, "Current TCP outside workspace"
        work = 0
        for positions in waypoints:
            if len(positions) != len(self.names) or not all(map(finite, positions)):
                return False, "Invalid trajectory positions"
            if not all(a <= v <= b for v, (a, b) in
                       zip(positions, (self.bounds[n] for n in self.names))):
                return False, "Joint limit in trajectory"
            target = np.array(positions)
            target_joints = dict(zip(self.names, positions))
            if not self.inside(self.tcp(target_joints)):
                return False, "Trajectory target outside tool workspace"
            body_ok, reason = self.body_inside(target_joints)
            if not body_ok:
                return False, reason
            # Upper bound on Cartesian arc length of a linear joint segment.
            # Every joint axis has reach <= sum of chain link lengths. Sampling
            # at <=2 mm of arc, with 1 mm clearance at samples, also bounds the
            # space between samples. Outgoing trajectories have positions only,
            # so the controller uses this linear joint interpolation.
            steps = max(1, math.ceil(self.reach * float(np.abs(target-previous).sum()) / .002))
            work += steps + 1
            if work > 1000:
                return False, "Trajectory exceeds validation work budget"
            for fraction in np.linspace(0, 1, steps+1):
                values = previous + fraction*(target-previous)
                point = self.tcp(dict(zip(self.names, values)))
                if not self.inside(point):
                    return False, "Trajectory would cross workspace boundary"
                body_ok, reason = self.body_inside(dict(zip(self.names, values)))
                if not body_ok:
                    return False, reason
            previous = target
        return True, "Trajectory inside workspace"
