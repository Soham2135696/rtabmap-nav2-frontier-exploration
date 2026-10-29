"""Frontier exploration for the rover: drive Nav2 to the edges of the known map.

A frontier is a free cell of /rtabmap/map that touches unknown space. Frontier
cells are grouped into clusters; each cluster gets a reachable, low-cost goal
backed off from the unknown edge, facing the frontier. The best cluster
(large, close, little turning) is sent to Nav2's NavigateToPose. Goals that
fail or stop making progress are blacklisted. When no frontier is left the
map is explored and the node exits.

Started by nav2.launch.py (explore:=true, default in mode:=sim).
Frontier markers for RViz: /explore/frontiers (MarkerArray).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

EIGHT_CONNECTED = np.ones((3, 3), dtype=bool)


class Grid:
    """Occupancy grid as a numpy array plus world <-> cell conversion."""

    def __init__(self, msg):
        info = msg.info
        self.res = info.resolution
        self.ox = info.origin.position.x
        self.oy = info.origin.position.y
        self.data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)

    def to_world(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def to_cell(self, x, y):
        col = int(math.floor((x - self.ox) / self.res))
        row = int(math.floor((y - self.oy) / self.res))
        if 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]:
            return row, col
        return None

    def value_at(self, x, y):
        cell = self.to_cell(x, y)
        return None if cell is None else int(self.data[cell])


class Frontier:
    def __init__(self, cells, size_m, target, goal, yaw):
        self.cells = cells      # (N, 2) world x, y of the frontier cells
        self.size_m = size_m    # frontier length, cells * resolution
        self.target = target    # world x, y of the frontier cell nearest the centroid
        self.goal = goal        # world x, y of the free, low-cost cell to drive to
        self.yaw = yaw          # goal heading: facing the frontier
        self.path_len = None
        self.score = None


class FrontierExplorer(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='frontier_explorer')
        p = self.declare_parameter
        self.map_topic = p('map_topic', '/rtabmap/map').value
        self.costmap_topic = p('costmap_topic', '/global_costmap/costmap').value
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_link').value
        # Clusters shorter than this (m) are noise, e.g. gaps in a wall
        self.min_frontier_size = p('min_frontier_size', 0.3).value
        # Goal is at least this far (m) from unknown/occupied cells
        self.goal_offset = p('goal_offset', 0.3).value
        # Look this far (m) around a frontier for a usable goal cell
        self.goal_search_radius = p('goal_search_radius', 1.0).value
        # Highest global costmap value (0-100) allowed at a goal; 99 = inscribed
        self.max_goal_cost = p('max_goal_cost', 60).value
        # Score = size_weight * size - distance_weight * path - turn_weight * |turn|
        self.size_weight = p('size_weight', 1.0).value
        self.distance_weight = p('distance_weight', 1.0).value
        self.turn_weight = p('turn_weight', 0.3).value
        # Only the nearest few clusters get a planned path (one planner call each)
        self.max_candidates = p('max_candidates', 5).value
        self.planner_period = 1.0 / p('planner_frequency', 1.0).value
        # Cancel and blacklist if the robot moves < progress_distance in progress_timeout
        self.progress_timeout = p('progress_timeout', 30.0).value
        self.progress_distance = p('progress_distance', 0.1).value
        self.blacklist_radius = p('blacklist_radius', 0.5).value
        # How many times the Nav2-failure blacklist is cleared for another try
        self.blacklist_retries = p('blacklist_retries', 1).value
        # Cancel the goal once no frontier cell is left this close to its target
        self.frontier_gone_radius = p('frontier_gone_radius', 0.5).value
        # After reaching a goal: blacklist its frontier if fewer than min_visit_gain
        # of the unknown cells within visit_check_radius (m) of it became known
        self.visit_check_radius = p('visit_check_radius', 1.0).value
        self.min_visit_gain = p('min_visit_gain', 0.1).value
        # 360 deg turn at start and after each goal: the camera only sees 60 deg
        self.spin_on_arrival = p('spin_on_arrival', True).value
        self.spin_time_allowance = p('spin_time_allowance', 40).value
        # Map must show no frontier this many planner cycles in a row to finish
        self.finish_checks = p('finish_checks', 3).value

        self.map = None
        self.costmap = None
        # Frontiers Nav2 could not reach: cleared blacklist_retries times before giving up,
        # since a goal that failed from one spot often works from another
        self.nav_failed = []
        # Frontiers the camera could not see from their goal: never retried
        self.unobservable = []
        # Transient local: RTAB-Map (latch, default true) and the Nav2 costmap
        # publish latched and go quiet while the robot stands still, so a
        # volatile subscriber started after their last publish never gets a map
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, qos)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self._on_costmap, qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'explore/frontiers', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_map(self, msg):
        self.map = Grid(msg)

    def _on_costmap(self, msg):
        self.costmap = Grid(msg)

    # --- Helpers ---------------------------------------------------------

    def wait(self, seconds):
        """Process callbacks for a while (map, costmap, TF)."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_pose(self):
        """(x, y, yaw) of the robot in the map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @property
    def blacklist(self):
        return self.nav_failed + self.unobservable

    def blacklisted(self, xy):
        return any(math.dist(xy, b) < self.blacklist_radius for b in self.blacklist)

    # --- Frontier detection ----------------------------------------------

    def frontier_mask(self, grid):
        free = grid.data == 0
        unknown = grid.data == -1
        return free & ndimage.binary_dilation(unknown, structure=EIGHT_CONNECTED)

    def robot_region(self, grid, pose):
        """Free cells connected to the robot (4-connected, so no diagonal leaks
        through walls). Excludes free cells that depth rays leaked beyond a wall."""
        free_labels, _ = ndimage.label(grid.data == 0)
        cell = grid.to_cell(pose[0], pose[1])
        if cell is None:
            return None
        # The cell under the robot may not be observed yet: use the nearest free cell
        rows, cols = np.nonzero(free_labels)
        d2 = (rows - cell[0]) ** 2 + (cols - cell[1]) ** 2
        k = int(np.argmin(d2)) if len(rows) else None
        if k is None or d2[k] * grid.res ** 2 > 0.5 ** 2:
            return None
        return free_labels == free_labels[rows[k], cols[k]]

    def find_frontiers(self, pose):
        grid, costmap = self.map, self.costmap
        # Clusters left out only because they are blacklisted (for the final report)
        self.skipped_blacklisted = 0
        region = self.robot_region(grid, pose)
        if region is None:
            return []
        # Frontier cells must touch the robot's free region (8-connected to it)
        mask = self.frontier_mask(grid) & region
        labels, count = ndimage.label(mask, structure=EIGHT_CONNECTED)
        if count == 0:
            return []

        # Distance (m) from each free cell to the nearest unknown or occupied cell
        clearance = ndimage.distance_transform_edt(grid.data == 0) * grid.res
        good = (clearance >= self.goal_offset) & region
        rows_all, cols_all = np.nonzero(good)

        frontiers = []
        for i in range(1, count + 1):
            rows, cols = np.nonzero(labels == i)
            size_m = len(rows) * grid.res
            if size_m < self.min_frontier_size:
                continue
            # Frontier cell nearest the cluster centroid (centroid may be off the frontier)
            cr, cc = rows.mean(), cols.mean()
            k = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
            target = grid.to_world(rows[k], cols[k])
            if self.blacklisted(target):
                self.skipped_blacklisted += 1
                continue

            # Goal: nearest free cell with enough clearance and low global cost
            d2 = (rows_all - rows[k]) ** 2 + (cols_all - cols[k]) ** 2
            near = np.nonzero(d2 <= (self.goal_search_radius / grid.res) ** 2)[0]
            goal = None
            for j in near[np.argsort(d2[near])]:
                xy = grid.to_world(rows_all[j], cols_all[j])
                cost = costmap.value_at(*xy)
                if cost is not None and 0 <= cost <= self.max_goal_cost:
                    goal = xy
                    break
            if goal is None:
                continue
            if self.blacklisted(goal):
                self.skipped_blacklisted += 1
                continue

            yaw = math.atan2(target[1] - goal[1], target[0] - goal[0])
            cells = np.column_stack([grid.ox + (cols + 0.5) * grid.res,
                                     grid.oy + (rows + 0.5) * grid.res])
            frontiers.append(Frontier(cells, size_m, target, goal, yaw))
        return frontiers

    def choose(self, frontiers, pose):
        """Score the nearest clusters by planned path length; return the best."""
        rx, ry, ryaw = pose
        frontiers.sort(key=lambda f: math.dist(f.goal, (rx, ry)))
        start = self.make_pose(rx, ry, ryaw)
        best = None
        for f in frontiers[:self.max_candidates]:
            # use_start=False: plan from the robot's current pose
            path = self.getPath(start, self.make_pose(*f.goal, f.yaw), use_start=False)
            if path is None or len(path.poses) == 0:
                self.nav_failed.append(f.target)
                continue
            pts = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
            f.path_len = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            heading = math.atan2(f.goal[1] - ry, f.goal[0] - rx)
            turn = abs(math.atan2(math.sin(heading - ryaw), math.cos(heading - ryaw)))
            f.score = (self.size_weight * f.size_m - self.distance_weight * f.path_len
                       - self.turn_weight * turn)
            if best is None or f.score > best.score:
                best = f
        return best

    def cells_near(self, mask, target, radius):
        """Number of mask cells within radius (m) of target."""
        rows, cols = np.nonzero(mask)
        xs = self.map.ox + (cols + 0.5) * self.map.res
        ys = self.map.oy + (rows + 0.5) * self.map.res
        return int(np.count_nonzero((xs - target[0]) ** 2 + (ys - target[1]) ** 2
                                    <= radius ** 2))

    def unknown_near(self, target):
        """Unknown cells around a frontier target; cells outside the map count
        as unknown, so a map that grows into the area still shows the gain."""
        inside = self.cells_near(np.ones_like(self.map.data, dtype=bool), target,
                                 self.visit_check_radius)
        total = math.pi * (self.visit_check_radius / self.map.res) ** 2
        unknown = self.cells_near(self.map.data == -1, target, self.visit_check_radius)
        return unknown + max(total - inside, 0.0)

    def frontier_still_there(self, target):
        return self.cells_near(self.frontier_mask(self.map), target,
                               self.frontier_gone_radius) > 0

    # --- RViz ------------------------------------------------------------

    def publish_markers(self, frontiers, chosen=None):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        def marker(ns, mid, mtype, scale, rgba):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            return m

        cells = marker('frontier_cells', 0, Marker.POINTS, 0.05, (0.0, 0.6, 1.0, 1.0))
        for f in frontiers:
            cells.points += [Point(x=float(x), y=float(y), z=0.05) for x, y in f.cells]
        markers.markers.append(cells)
        for i, f in enumerate(frontiers):
            g = marker('goals', i, Marker.SPHERE, 0.15, (1.0, 0.8, 0.0, 0.8))
            g.pose.position.x, g.pose.position.y = f.goal
            markers.markers.append(g)
        if chosen is not None:
            c = marker('chosen', 0, Marker.ARROW, 0.1, (0.0, 1.0, 0.0, 1.0))
            c.scale.x, c.scale.y, c.scale.z = 0.4, 0.08, 0.08
            c.pose = self.make_pose(*chosen.goal, chosen.yaw).pose
            markers.markers.append(c)
        for i, (x, y) in enumerate(self.blacklist):
            b = marker('blacklist', i, Marker.CYLINDER, 2 * self.blacklist_radius,
                       (1.0, 0.0, 0.0, 0.25))
            b.scale.z = 0.02
            b.pose.position.x, b.pose.position.y = x, y
            markers.markers.append(b)
        self.marker_pub.publish(markers)

    # --- Main loop -------------------------------------------------------

    def spin_in_place(self):
        if not self.spin_on_arrival:
            return
        self.spin(spin_dist=2.0 * math.pi, time_allowance=self.spin_time_allowance)
        while rclpy.ok() and not self.isTaskComplete():
            pass

    def drive_to(self, frontier):
        """Run one NavigateToPose goal. Returns True if it reached the goal."""
        if not self.goToPose(self.make_pose(*frontier.goal, frontier.yaw)):
            self.nav_failed.append(frontier.target)
            return False

        last_pose = self.robot_pose()
        last_progress = self.get_clock().now()
        last_check = self.get_clock().now()
        while rclpy.ok() and not self.isTaskComplete():
            now = self.get_clock().now()
            if (now - last_check).nanoseconds * 1e-9 < self.planner_period:
                continue
            last_check = now

            pose = self.robot_pose()
            if pose is not None and (last_pose is None or
                                     math.dist(pose[:2], last_pose[:2]) > self.progress_distance):
                last_pose, last_progress = pose, now
            if (now - last_progress).nanoseconds * 1e-9 > self.progress_timeout:
                self.warn('No progress towards the frontier: cancel and blacklist it')
                self.cancelTask()
                self.nav_failed.append(frontier.target)
                return False
            if not self.frontier_still_there(frontier.target):
                self.info('Frontier mapped on the way: picking the next one')
                self.cancelTask()
                return False

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            return True
        if result == TaskResult.FAILED:
            self.warn('Nav2 could not reach the frontier: blacklisting it')
            self.nav_failed.append(frontier.target)
        return False

    def run(self):
        # No AMCL here (RTAB-Map localizes), so wait on the planner instead
        self.waitUntilNav2Active(localizer='planner_server')
        self.info(f'Waiting for {self.map_topic}, {self.costmap_topic} and TF')
        while rclpy.ok() and (self.map is None or self.costmap is None
                              or self.robot_pose() is None):
            self.wait(0.5)

        self.info('Exploration started')
        self.spin_in_place()
        empty_checks = 0
        while rclpy.ok():
            self.wait(self.planner_period)
            pose = self.robot_pose()
            if pose is None:
                continue
            frontiers = self.find_frontiers(pose)
            best = self.choose(frontiers, pose) if frontiers else None
            self.publish_markers(frontiers, best)
            if best is None:
                empty_checks += 1
                if empty_checks >= self.finish_checks:
                    if self.skipped_blacklisted and self.nav_failed and self.blacklist_retries > 0:
                        self.blacklist_retries -= 1
                        self.info(f'Retrying {len(self.nav_failed)} frontier(s) Nav2 failed to '
                                  f'reach earlier ({self.blacklist_retries} retries left)')
                        self.nav_failed.clear()
                        empty_checks = 0
                        continue
                    if self.skipped_blacklisted:
                        self.warn(f'Exploration stopped: {self.skipped_blacklisted} frontier(s) '
                                  f'left, but all are blacklisted (unreachable or not '
                                  f'observable); the map may be incomplete')
                    else:
                        self.info(f'Exploration complete: no frontier left '
                                  f'({len(self.blacklist)} blacklisted on the way)')
                    return
                continue
            empty_checks = 0
            self.info(f'Next frontier: {best.size_m:.2f} m long, path {best.path_len:.2f} m, '
                      f'goal ({best.goal[0]:.2f}, {best.goal[1]:.2f})')
            before = self.unknown_near(best.target)
            if self.drive_to(best):
                self.spin_in_place()
                # Arrived and looked around, yet almost nothing around the frontier
                # became known: the camera cannot see it from here (e.g. behind a
                # wall), so don't come back
                self.wait(1.0)
                after = self.unknown_near(best.target)
                if (self.frontier_still_there(best.target)
                        and before - after < self.min_visit_gain * before):
                    self.warn(f'Visit revealed almost nothing ({before:.0f} -> {after:.0f} '
                              f'unknown cells): blacklisting the frontier')
                    self.unobservable.append(best.target)


def main():
    rclpy.init()
    explorer = FrontierExplorer()
    try:
        explorer.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C: stop the robot if it was still driving to a frontier
        if rclpy.ok():
            explorer.cancelTask()
    finally:
        # ros2 launch can deliver a second SIGINT while cleaning up
        try:
            explorer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == '__main__':
    main()
