"""SDF for the rescue_building / rescue_compound worlds, generated from the core scenario (single geometry source).

Everything that the planner treats as an obstacle (walls, props, victims) occupies blocked grid
cells; floor markings stay below 1 cm; exterior scenery sits outside the perimeter wall, where the
2D lidar (0.55 m above the floor) cannot see it. Third-party meshes: CC0 Gazebo Fuel models in
aegisnav_bringup/models (see THIRD_PARTY_NOTICES.md).
"""
from __future__ import annotations

import math
import random

from aegisnav_core.models import Cell

INTERIOR_WALL_M = 1.2
PERIMETER_WALL_M = 1.6
LIDAR_Z_ON_BASE = 0.40  # metres above base_link; base_link sits 0.15 m above the floor
FLAT_Z = 0.003
PALLET_HALF_SIZE_M = (0.40, 0.60)  # euro_pallet footprint half extents (x, y) at yaw 0, measured
PALLET_HALF_HEIGHT_M = 0.072


def _mat(r, g, b, emissive=0.0):
    return (f"<material><ambient>{r * 0.6:.2f} {g * 0.6:.2f} {b * 0.6:.2f} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular><emissive>{r * emissive:.2f} {g * emissive:.2f} {b * emissive:.2f} 1</emissive></material>")


CONCRETE = "model://jersey_barrier/materials/textures/jersey_barrier_diffuse.jpg"
MASONRY = "model://cinder_block/materials/textures/cinder_block_diffuse.png"
RUBBLE = "model://collapsed_industrial/materials/textures/rubble_diffuse.jpg"


def _tex(uri, r=1.0, g=1.0, b=1.0, roughness=0.9):
    """PBR material using a CC0 texture shipped inside a vendored Fuel model (tinted by rgb)."""
    return (f"<material><ambient>{r * 0.7:.2f} {g * 0.7:.2f} {b * 0.7:.2f} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.05 0.05 0.05 1</specular><pbr><metal><albedo_map>{uri}</albedo_map>"
            f"<roughness>{roughness}</roughness><metalness>0</metalness></metal></pbr></material>")


def _box_model(name, x, y, z, sx, sy, sz, rgb, yaw=0.0, roll=0.0, collision=True, texture=None):
    geom = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    col = f'<collision name="c">{geom}</collision>' if collision else ""
    mat = _tex(texture, *rgb) if texture else _mat(*rgb)
    return (f'    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} {roll:.3f} 0 {yaw:.3f}</pose>'
            f'<link name="l">{col}<visual name="v">{geom}{mat}</visual></link></model>\n')


def _flat(name, x, y, sx, sy, rgb, emissive=0.35, yaw=0.0):
    return (f'    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {FLAT_Z} 0 0 {yaw:.3f}</pose><link name="l">'
            f'<visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>{sx:.3f} {sy:.3f} 0.004</size></box></geometry>'
            f'{_mat(*rgb, emissive)}</visual></link></model>\n')


def _include(name, uri, x, y, z=0.0, yaw=0.0):
    # Static: scenery never moves, and dynamic mesh bodies resting on each other made physics
    # run at a real-time factor of ~0.008 (measured).
    return (f'    <include><name>{name}</name><uri>model://{uri}</uri><static>true</static>'
            f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose></include>\n')


def _walls(grid, skip: set):
    """One box per horizontal run of blocked cells, excluding cells drawn as props or victims."""
    out, r = [], grid.resolution_m
    wall = lambda x, y: grid.is_blocked(Cell(x, y)) and Cell(x, y) not in skip
    for y in range(grid.height):
        x, seg = 0, 0
        while x < grid.width:
            if not wall(x, y):
                x += 1
                continue
            start = x
            while x < grid.width and wall(x, y):
                x += 1
            perimeter = y in (0, grid.height - 1) or (x - start == 1 and start in (0, grid.width - 1))
            h = PERIMETER_WALL_M if perimeter else INTERIOR_WALL_M
            rgb = (0.55, 0.42, 0.36) if perimeter else (0.78, 0.76, 0.72)
            cx = grid.origin_x_m + (start + x) / 2 * r
            cy = grid.origin_y_m + (y + 0.5) * r
            # One collision box per run (physics); 0.5 m textured visual blocks so the masonry
            # texture keeps a realistic scale instead of stretching across the whole wall.
            tint = (0.85, 0.62, 0.52) if perimeter else (0.92, 0.92, 0.9)
            visuals = "".join(
                f'<visual name="b{k}"><pose>{(start + k + 0.5) * r - (x - start) * r / 2 - start * r:.3f} 0 0 0 0 0</pose>'
                f'<geometry><box><size>{r:.3f} {r:.3f} {h:.3f}</size></box></geometry>{_tex(MASONRY, *tint)}</visual>'
                for k in range(x - start))
            out.append(f'    <model name="wall_y{y:02d}_seg{seg:02d}"><static>true</static><pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>'
                       f'<link name="l"><collision name="c"><geometry><box><size>{(x - start) * r:.3f} {r:.3f} {h:.3f}</size></box></geometry></collision>'
                       f'{visuals}</link></model>\n')
            seg += 1
    return "".join(out)


def _props(scn):
    """Props fill their blocked cells; one model group per cluster of same-kind cells."""
    grid, r = scn.grid, scn.grid.resolution_m
    clusters: dict[tuple[str, int], list[Cell]] = {}
    seen: set[Cell] = set()
    kinds = dict(scn.props)
    for cell, kind in scn.props:
        if cell in seen:
            continue
        stack, members = [cell], []
        while stack:
            c = stack.pop()
            if c in seen or kinds.get(c) != kind:
                continue
            seen.add(c)
            members.append(c)
            stack += [Cell(c.x + 1, c.y), Cell(c.x - 1, c.y), Cell(c.x, c.y + 1), Cell(c.x, c.y - 1)]
        clusters[(kind, len(clusters))] = members
    rng = random.Random(7)
    out = []
    for (kind, i), cells in clusters.items():
        xs = [grid.cell_center_world(c)[0] for c in cells]
        ys = [grid.cell_center_world(c)[1] for c in cells]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        sx = (max(xs) - min(xs)) + r
        sy = (max(ys) - min(ys)) + r
        if kind == "barrels":
            out.append(_include(f"barrel_{i}", "construction_barrel", cx, cy, 0.0, rng.uniform(0, 3.14)))
        elif kind == "pallets":
            # Measured from meshes/pallet.dae including its node transform: the mesh is centred on the model
            # origin, x in [-0.40, 0.40], y in [-0.60, 0.60], z in [-0.072, 0.072] (collision pose z 0.05).
            # (An earlier corner-origin assumption shifted pallets 0.6 m into free cells: a robot drove into
            # one in rescue_compound.) Long side along the longer cluster axis, bottom on the floor.
            yaw = 0.0 if sy >= sx else math.pi / 2
            for k in range(3):
                out.append(_include(f"pallet_{i}_{k}", "euro_pallet", cx, cy, PALLET_HALF_HEIGHT_M + 0.144 * k, yaw))
        else:  # rubble / cinder: broken slabs + cinder blocks, all inside the cluster footprint
            for k in range(3 if kind == "rubble" else 1):
                out.append(_box_model(f"slab_{i}_{k}", cx + rng.uniform(-0.15, 0.15) * sx, cy + rng.uniform(-0.15, 0.15) * sy,
                                      0.12 + 0.1 * k, sx * 0.7, sy * 0.6, 0.12, (0.5, 0.49, 0.47),
                                      yaw=rng.uniform(-0.3, 0.3), roll=rng.uniform(-0.15, 0.15), texture=RUBBLE))
            for k in range(6 if kind == "cinder" else 3):
                out.append(_include(f"cinder_{i}_{k}", "cinder_block", cx + rng.uniform(-0.3, 0.3) * sx,
                                    cy + rng.uniform(-0.3, 0.3) * sy, 0.2 * (k % 2), rng.uniform(0, 3.14)))
    return "".join(out)


# Rescue Randy (sitting) footprint in model coordinates, measured from meshes/rescue_randy.dae (unit
# 0.0001 m, node transforms applied): off-centre, ~0.75 x 1.0 m, larger than the victim's 0.5 m cell.
RANDY_FOOTPRINT_M = ((-0.473, -0.714), (0.274, 0.304))


def _footprint_points(x, y, yaw, footprint, step=0.05):
    (x0, y0), (x1, y1) = footprint
    c, s = math.cos(yaw), math.sin(yaw)
    nx, ny = max(1, round((x1 - x0) / step)), max(1, round((y1 - y0) / step))
    for i in range(nx + 1):
        for j in range(ny + 1):
            px, py = x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny
            yield x + px * c - py * s, y + px * s + py * c


def _fit_yaw(grid, x, y, footprint):
    """Of the four axis yaws, the one whose footprint covers the fewest free cells (ties: first)."""
    def free_hits(yaw):
        hits = 0
        for px, py in _footprint_points(x, y, yaw, footprint):
            cx, cy = int((px - grid.origin_x_m) // grid.resolution_m), int((py - grid.origin_y_m) // grid.resolution_m)
            hits += not (grid.contains(Cell(cx, cy)) and grid.is_blocked(Cell(cx, cy)))
        return hits
    return min((0.0, math.pi / 2, math.pi, -math.pi / 2), key=free_hits)


def _victims(scn):
    out = []
    for i, c in enumerate(scn.victims):
        x, y = scn.grid.cell_center_world(c)
        # The mesh is larger than the victim's cell: turn it so the body lies on blocked cells (its own
        # cell and the adjacent walls), never across free cells the planner may route through.
        out.append(_include(f"victim_{i}", "rescue_randy_sitting", x, y, 0.0, _fit_yaw(scn.grid, x, y, RANDY_FOOTPRINT_M)))
    return "".join(out)


def _pads(scn):
    g, r = scn.grid, scn.grid.resolution_m
    out = [_flat("pad_base", *g.cell_center_world(scn.base), 2 * r, 2 * r, (0.10, 0.35, 0.95))]
    for i, c in enumerate(scn.refuges):
        out.append(_flat(f"pad_refuge_{i}", *g.cell_center_world(c), 2 * r, 2 * r, (0.05, 0.80, 0.30)))
    for i, c in enumerate(scn.mission_goals):
        out.append(_flat(f"pad_goal_{i}", *g.cell_center_world(c), 1.6 * r, 1.6 * r, (1.0, 0.80, 0.10)))
    return "".join(out)


def _exterior(scn):
    """Collapsed buildings, vehicles and barriers outside the walled area, placed relative to the grid's
    edges so they stay outside whatever the scenario's footprint (identical to the original placement for
    the 40 x 28 rescue_building grid)."""
    g = scn.grid
    x0, y0 = g.origin_x_m, g.origin_y_m
    x1, y1 = x0 + g.width * g.resolution_m, y0 + g.height * g.resolution_m
    out = [
        _include("collapsed_house", "collapsed_house", x0 - 14, y1 + 7, 0.0, 0.3),
        _include("collapsed_police_station", "collapsed_police_station", x1 + 17, y0 - 11, 0.0, 1.57),
        _include("collapsed_fire_station", "collapsed_fire_station", x1 + 20, y1 + 10, 0.0, 0.0),
        _include("collapsed_industrial", "collapsed_industrial", x0 - 20, y0 - 15, 0.0, 0.5),
        _include("fire_truck", "fire_truck", x0 + 4, y0 - 5.5, 0.0, 1.5708),
        _include("fire_hydrant", "fire_hydrant", x0 - 2, y0 - 1.6, 0.16, 0.0),
    ]
    for i, x in enumerate((9.0, 13.5, 18.0)):
        out.append(_include(f"jersey_{i}", "jersey_barrier", x0 + x, y0 - 2.2, 0.0, 0.0))
    cones = ((x0 - 2.5, y0 - 3.5), (x0 - 1.5, y0 - 4.5), (x0 + 12.0, y0 - 3.6), (x0 + 16.0, y0 - 3.8),
             (x1 + 2.5, y1 + 2.2), (x1 + 3.5, y1 + 1.6))
    for i, (x, y) in enumerate(cones):
        out.append(_include(f"cone_{i}", "construction_cone", x, y, 0.0, 0.0))
    return "".join(out)


def husky_robot(x: float, y: float) -> str:
    """Husky A200 look on a true differential drive (measured: a 4-wheel skid-steer body with full tyre
    friction did not rotate in place at all in Gazebo, so rotate-to-heading and odometry failed).

    Physics: two hidden drive wheels at the chassis centre (track 0.571 m, r 0.17775 m) + two
    frictionless caster spheres front/back, all bottoms at the same height. The four Husky wheels are
    visual-only links on revolute joints that DiffDrive also turns, so they spin on screen.
    All visuals stay below the lidar plane (0.40 m above base_link).
    """
    wz = 0.035          # wheel centre height relative to base_link
    r = 0.17775         # wheel radius
    inertia = '<inertial><mass>2.637</mass><inertia><ixx>0.02467</ixx><iyy>0.04411</iyy><izz>0.02467</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
    links, joints = [], []
    for name, wy in (("drive_left", 0.2855), ("drive_right", -0.2855)):
        links.append(
            f'<link name="{name}_wheel"><pose>0 {wy} {wz} 0 0 0</pose>{inertia}'
            f'<collision name="c"><pose>0 0 0 -1.5707 0 0</pose><geometry><cylinder><radius>{r}</radius><length>0.08</length></cylinder></geometry>'
            '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision></link>')
    for name, cx in (("front_caster", 0.40), ("rear_caster", -0.40)):
        cr = 0.05
        links.append(
            # 4 mm above the wheel bottoms: the drive wheels carry the load (measured: level casters let
            # the drive wheels slip and odometry over-read straight travel by ~39%).
            f'<link name="{name}"><pose>{cx} 0 {wz - r + cr + 0.004:.5f} 0 0 0</pose>'
            '<inertial><mass>0.2</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="c"><geometry><sphere><radius>{cr}</radius></sphere></geometry>'
            '<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface></collision></link>')
        joints.append(f'<joint name="{name}_joint" type="fixed"><parent>base_link</parent><child>{name}</child></joint>')
    for name, wx, wy in (("front_left", 0.256, 0.2855), ("front_right", 0.256, -0.2855),
                         ("back_left", -0.256, 0.2855), ("back_right", -0.256, -0.2855)):
        links.append(
            f'<link name="{name}_wheel"><pose>{wx} {wy} {wz} 0 0 0</pose>'
            '<inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            '<visual name="v"><pose>0 0 0 -3.14159 0 0</pose><geometry><mesh><uri>model://husky/meshes/wheel.stl</uri></mesh></geometry>'
            f'{_mat(0.08, 0.08, 0.08)}</visual></link>')
    for name in ("drive_left", "drive_right", "front_left", "front_right", "back_left", "back_right"):
        joints.append(
            f'<joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child>'
            '<axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>1000</effort></limit></axis></joint>')
    lid = LIDAR_Z_ON_BASE
    return f'''    <model name="aegis_bot" canonical_link="base_link">
      <pose>{x:.3f} {y:.3f} 0.15 0 0 0</pose>
      <link name="base_link">
        <inertial><pose>0 0 0.2 0 0 0</pose><mass>33.855</mass><inertia><ixx>2.2343</ixx><iyy>3.42518</iyy><izz>2.1241</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="chassis"><pose>0 0 0.1 0 0 0</pose><geometry><box><size>1.0074 0.5709 0.2675</size></box></geometry></collision>
        <visual name="base"><geometry><mesh><uri>model://husky/meshes/base_link.stl</uri></mesh></geometry>{_mat(0.07, 0.07, 0.08)}</visual>
        <visual name="top_plate"><geometry><mesh><uri>model://husky/meshes/top_plate.stl</uri></mesh></geometry>{_mat(0.95, 0.75, 0.05)}</visual>
        <visual name="front_bumper"><pose>0.47 0 0.091 0 0 0</pose><geometry><mesh><uri>model://husky/meshes/bumper.stl</uri></mesh></geometry>{_mat(0.1, 0.1, 0.1)}</visual>
        <visual name="rear_bumper"><pose>-0.47 0 0.091 0 0 3.141</pose><geometry><mesh><uri>model://husky/meshes/bumper.stl</uri></mesh></geometry>{_mat(0.1, 0.1, 0.1)}</visual>
        <visual name="user_rail"><pose>0.272 0 0.245 0 0 0</pose><geometry><mesh><uri>model://husky/meshes/user_rail.stl</uri></mesh></geometry>{_mat(0.6, 0.6, 0.62)}</visual>
        <visual name="lidar_housing"><pose>0.2 0 {lid - 0.045:.3f} 0 0 0</pose><geometry><cylinder><radius>0.05</radius><length>0.07</length></cylinder></geometry>{_mat(0.12, 0.12, 0.14)}</visual>
        <visual name="beacon"><pose>-0.3 0.18 0.31 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.05</length></cylinder></geometry>{_mat(1.0, 0.35, 0.05, 0.9)}</visual>
        <sensor name="front_lidar" type="gpu_lidar"><pose>0.2 0 {lid:.3f} 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate>
          <topic>/model/aegis_bot/scan</topic>
          <lidar><visibility_mask>4294967293</visibility_mask><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan>
            <range><min>0.3</min><max>12.0</max><resolution>0.02</resolution></range></lidar></sensor>
      </link>
      {"".join(links)}
      {"".join(joints)}
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>drive_left_joint</left_joint><left_joint>front_left_joint</left_joint><left_joint>back_left_joint</left_joint>
        <right_joint>drive_right_joint</right_joint><right_joint>front_right_joint</right_joint><right_joint>back_right_joint</right_joint>
        <wheel_separation>0.571</wheel_separation><wheel_radius>{r}</wheel_radius>
        <topic>/model/aegis_bot/cmd_vel</topic><odom_topic>/model/aegis_bot/odometry</odom_topic><tf_topic>/model/aegis_bot/tf</tf_topic>
        <frame_id>odom</frame_id><child_frame_id>base_footprint</child_frame_id><odom_publish_frequency>30</odom_publish_frequency>
      </plugin>
    </model>
'''


def _floor_tiles(grid, tile_m: float = 2.0) -> str:
    """Visual-only concrete tiles 1 mm above the ground slab (texture at a realistic scale)."""
    w, h = grid.width * grid.resolution_m, grid.height * grid.resolution_m
    nx, ny = round(w / tile_m), round(h / tile_m)
    visuals = "".join(
        f'<visual name="t{i}_{j}"><pose>{grid.origin_x_m + (i + 0.5) * w / nx:.3f} {grid.origin_y_m + (j + 0.5) * h / ny:.3f} 0.001 0 0 0</pose>'
        f'<cast_shadows>false</cast_shadows><geometry><box><size>{w / nx:.3f} {h / ny:.3f} 0.002</size></box></geometry>'
        f'{_tex(CONCRETE, 0.95, 0.94, 0.9)}</visual>'
        for i in range(nx) for j in range(ny))
    return f'    <model name="floor_tiles"><static>true</static><link name="l">{visuals}</link></model>\n'


def rescue_world_sdf(scn) -> str:
    g = scn.grid
    w, h = g.width * g.resolution_m, g.height * g.resolution_m
    cx, cy = g.origin_x_m + w / 2, g.origin_y_m + h / 2
    sx, sy = g.cell_center_world(scn.start)
    return f'''<?xml version="1.0" ?>
<!-- GENERATED by tools/generate_world_assets.py from aegisnav_core.scenarios ({scn.scenario_id}). Do not edit. -->
<sdf version="1.9">
  <world name="aegisnav_rescue_world">
    <gravity>0 0 -9.81</gravity>
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <scene><ambient>0.36 0.36 0.40 1</ambient><background>0.55 0.62 0.70 1</background><shadows>true</shadows><grid>false</grid></scene>
    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.95 0.88 0.78 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.45 0.35 -0.82</direction></light>
    <light name="fill" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.25 0.28 0.35 1</diffuse><specular>0 0 0 1</specular><direction>0.5 -0.4 -0.75</direction></light>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <!-- Renders the hazard-source particle effects spawned by visualization_node (display only). -->
    <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>
    <model name="ground"><static>true</static><pose>{cx:.3f} {cy:.3f} -0.1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>90 70 0.2</size></box></geometry></collision>
      <visual name="asphalt"><pose>0 0 -0.002 0 0 0</pose><geometry><box><size>90 70 0.2</size></box></geometry>{_mat(0.24, 0.24, 0.25)}</visual>
    </link></model>
{_floor_tiles(g)}
{_walls(g, {c for c, _ in scn.props} | set(scn.victims))}{_props(scn)}{_victims(scn)}{_pads(scn)}{_exterior(scn)}{husky_robot(sx, sy)}  </world>
</sdf>
'''
