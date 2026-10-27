"""Generate a polished Gazebo world for recording: checkerboard floor, posts, landing pad, coloured blocks.

Writes study/assets/checker.png and study/assets/showcase.sdf. The SDF points at the texture by absolute path in WSL
(TEXTURE_URI), so copy both files there with install_showcase.sh.
"""
import math
import os
import random

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, 'assets')
TEXTURE_URI = 'file:///home/abhishek/gz_assets/checker.png'

FLOOR_M = 80            # floor size in metres
TILE_M = 2              # checker tile size in metres
PX_PER_TILE = 100
LIGHT = (247, 249, 251)
BLUE = (168, 221, 240)
ORANGE = (255, 112, 45)
PALETTE = ['#ffd166', '#ef476f', '#06d6a0', '#9b5de5', '#4cc9f0', '#f8961e']


def make_texture():
    n = FLOOR_M // TILE_M
    size = n * PX_PER_TILE
    ii, jj = np.meshgrid(np.arange(n), np.arange(n))
    tile = ((ii + jj) % 2).astype(np.uint8)
    img = np.where(tile[..., None] == 0, np.array(LIGHT, np.uint8), np.array(BLUE, np.uint8))
    img = np.kron(img, np.ones((PX_PER_TILE, PX_PER_TILE, 1), np.uint8))
    c, w = size // 2, 5   # orange axes through the origin
    img[c - w:c + w, :, :] = ORANGE
    img[:, c - w:c + w, :] = ORANGE
    os.makedirs(ASSETS, exist_ok=True)
    Image.fromarray(img).save(os.path.join(ASSETS, 'checker.png'), optimize=True)
    return size


def rgb(hexcol):
    h = hexcol.lstrip('#')
    return ' '.join(f'{int(h[i:i + 2], 16) / 255:.3f}' for i in (0, 2, 4))


def visual_model(name, x, y, z, geom, color, emissive=None, yaw=0.0):
    em = f'<emissive>{rgb(emissive)} 1</emissive>' if emissive else ''
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose><link name="l">'
            f'<visual name="v"><geometry>{geom}</geometry><material><ambient>{rgb(color)} 1</ambient>'
            f'<diffuse>{rgb(color)} 1</diffuse><specular>0.3 0.3 0.3 1</specular>{em}</material></visual></link></model>')


def objects():
    out = []
    post = '<cylinder><radius>0.1</radius><length>2.6</length></cylinder>'
    ball = '<sphere><radius>0.22</radius></sphere>'
    for k, (x, y) in enumerate([(0, 4), (4, 4), (4, 0), (2, 2)]):
        col = '#ff702d' if k < 3 else '#2a7de1'
        out.append(visual_model(f'post{k}', x, y, 1.3, post, col))
        out.append(visual_model(f'beacon{k}', x, y, 2.75, ball, '#e0fbfc', emissive='#7fe3ff'))
    out.append(visual_model('pad_ring', 0, 0, 0.012, '<cylinder><radius>1.35</radius><length>0.02</length></cylinder>', '#ff702d'))
    out.append(visual_model('pad', 0, 0, 0.024, '<cylinder><radius>1.2</radius><length>0.02</length></cylinder>', '#e6eef5'))
    out.append(visual_model('pad_h', 0, 0, 0.036, '<box><size>0.9 0.16 0.02</size></box>', '#ff702d'))
    out.append(visual_model('pad_h2', 0, 0, 0.036, '<box><size>0.16 0.9 0.02</size></box>', '#ff702d'))
    rng = random.Random(7)
    for k in range(22):
        ang = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(13, 33)
        sx, sy, h = rng.uniform(1.2, 3.2), rng.uniform(1.2, 3.2), rng.uniform(1.0, 7.5)
        out.append(visual_model(f'block{k}', round(r * math.cos(ang), 2), round(r * math.sin(ang), 2), h / 2,
                                f'<box><size>{sx:.2f} {sy:.2f} {h:.2f}</size></box>', rng.choice(PALETTE),
                                yaw=round(rng.uniform(0, 3.14), 2)))
    return '\n    '.join(out)


def make_world():
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="showcase">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.62 0.64 0.68 1</ambient>
      <background>0.74 0.88 0.97 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="floor">
          <geometry><plane><normal>0 0 1</normal><size>{FLOOR_M} {FLOOR_M}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.15 0.15 0.15 1</specular>
            <pbr><metal><albedo_map>{TEXTURE_URI}</albedo_map><metalness>0.0</metalness><roughness>0.85</roughness></metal></pbr>
          </material>
        </visual>
        <visual name="outer_ground">
          <pose>0 0 -0.03 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
          <material><ambient>0.86 0.90 0.93 1</ambient><diffuse>0.86 0.90 0.93 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial><pose>0 0 0 0 -0 0</pose><mass>1</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia></inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    {objects()}
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.15</intensity>
      <direction>-0.45 0.55 -0.70</direction>
      <diffuse>1.0 0.96 0.88 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>488</elevation>
    </spherical_coordinates>
  </world>
</sdf>
'''


if __name__ == '__main__':
    px = make_texture()
    with open(os.path.join(ASSETS, 'showcase.sdf'), 'w', encoding='utf-8') as f:
        f.write(make_world())
    print('texture', px, 'px; world written to', ASSETS)
