import numpy as np
import matplotlib.pyplot as plt
import colorcet as cc
import os
from typing import Optional, Tuple

from planning_utils import create_grid

FILENAME = 'colliders.csv'
FILENAME2 = 'colliders.additional.csv'
DPI = 72

plt.rcParams['figure.figsize'] = 12, 10
plt.rcParams['figure.dpi'] = DPI

data = np.loadtxt(FILENAME, delimiter=',', dtype='Float64', skiprows=2)
data_additional = np.loadtxt(FILENAME2, delimiter=',', dtype='Float64', skiprows=2)


def configure_axes(grid: np.ndarray, ax: plt.Axes, north_offset: float, east_offset: float):
    east_ticks = np.linspace(0, np.array(grid).shape[1], 5)
    north_ticks = np.linspace(0, np.array(grid).shape[0], 5)

    ax.set_xticks(east_ticks)
    ax.set_yticks(north_ticks)
    ax.set_xticklabels([f'{t + east_offset:.1f}' for t in east_ticks])
    ax.set_yticklabels([f'{t + north_offset:.1f}' for t in north_ticks])

    ax.set_xlabel('EAST')
    ax.set_ylabel('NORTH')


def plot_grid(title: Optional[str], grid: np.ndarray, north_offset: float, east_offset: float, colormap, show_colorbar: bool=False, fig: Optional[plt.Figure]=None, ax: Optional[plt.Axes]=None, vmax: Optional[int]=None) -> plt.Figure:
    if fig is None or ax is None:
        fig, ax = plt.subplots()

    vmax = np.max(grid) if vmax is None else vmax

    im = ax.imshow(grid, origin='lower', cmap=colormap, vmin=0, vmax=vmax)
    configure_axes(grid, ax, north_offset, east_offset)

    if title:
        ax.set_title(title)

    if show_colorbar:
        fig.colorbar(im, ax=ax, shrink=0.6)
    return fig


_, height_map, north_offset, east_offset = create_grid(data, drone_altitude=0, safety_distance=0, safety_altitude=0)
figure = plot_grid('Height map', height_map, north_offset, east_offset, cc.cm.blues, show_colorbar=True)

figure.tight_layout()
figure.savefig(os.path.join('misc', 'heightmap.png'), dpi=DPI)

_, height_map, north_offset, east_offset = create_grid(data, drone_altitude=0, safety_distance=0, safety_altitude=0)
figure = plot_grid('Height map', height_map, north_offset, east_offset, cc.cm.blues, show_colorbar=True, vmax=25)

figure.tight_layout()
figure.savefig(os.path.join('misc', 'heightmap_25.png'), dpi=DPI)


_, height_map2, north_offset2, east_offset2 = create_grid(data_additional, 0, 0, 0)
assert height_map.shape == height_map2.shape
assert north_offset == north_offset2
assert east_offset == east_offset2

for i in range(height_map.shape[0]):
    for j in range(height_map.shape[1]):
        height_map[i, j] = max(height_map[i, j], height_map2[i, j])

figure = plot_grid('Height map (merged)', height_map, north_offset, east_offset, cc.cm.blues, show_colorbar=True, vmax=25)
figure.tight_layout()
figure.savefig(os.path.join('misc', 'heightmap_merged_25.png'), dpi=DPI)


altitudes = [0, 10, 20]
safety_distances = [0, 5]

fig, axs = plt.subplots(nrows=len(altitudes), ncols=len(safety_distances), figsize=(12, 16))
for (i, j) in [(a, s) for a in range(len(altitudes)) for s in range(len(safety_distances))]:
    altitude, safety = altitudes[i], safety_distances[j]
    grid, _, north_offset, east_offset = create_grid(data, drone_altitude=altitude, safety_distance=safety, safety_altitude=0)
    title = f'altitude={altitude}m, safety margin={safety}m' if altitude > 0 else f'ground, safety margin={safety}m'
    figure = plot_grid(title, grid, north_offset, east_offset, cc.cm.blues,
                       fig=fig, ax=axs[i][j])

figure.tight_layout()
figure.savefig(os.path.join('misc', 'obstacles.png'), dpi=DPI)
plt.show()
