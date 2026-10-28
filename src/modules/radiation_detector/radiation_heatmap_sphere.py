import os
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from scipy.interpolate import griddata


# KONFIGURACIJA

CSV_PATH = "/tmp/radiation_map.csv"
SDF_PATH = "Tools/simulation/gz/worlds/radiation_field.sdf"

GAMMA_CONSTANT_CS137 = 0.098  # µSv*m^2 / (h*MBq)
THRESHOLD = 3.0               # µSv/h


# UCITAVANJE MERENJA

if not os.path.exists(CSV_PATH):
    raise FileNotFoundError(
        f"CSV datoteka nije pronadjena: {CSV_PATH}"
    )

data = pd.read_csv(CSV_PATH)

required_columns = {"x", "y", "radiation"}

if not required_columns.issubset(data.columns):
    raise RuntimeError(
        "CSV mora da sadrzi kolone: x, y, radiation"
    )

x = data["x"].to_numpy(dtype=float)
y = data["y"].to_numpy(dtype=float)
radiation = data["radiation"].to_numpy(dtype=float)

if len(x) == 0:
    raise RuntimeError("CSV datoteka ne sadrzi merenja.")


# UCITAVANJE SDF DATOTEKE

if not os.path.exists(SDF_PATH):
    raise FileNotFoundError(
        f"SDF datoteka nije pronadjena: {SDF_PATH}"
    )

tree = ET.parse(SDF_PATH)
root = tree.getroot()


# STVARNI POLOZAJ IZVORA

source_model = root.find(".//model[@name='radiation_source']")

if source_model is None:
    raise RuntimeError(
        "Model 'radiation_source' nije pronadjen u SDF datoteci."
    )

pose = source_model.find("pose")

if pose is None or pose.text is None:
    raise RuntimeError(
        "Pose modela 'radiation_source' nije pronadjen."
    )

pose_values = list(map(float, pose.text.split()))

if len(pose_values) < 3:
    raise RuntimeError(
        "Neispravan pose element za radiation_source."
    )

sx_enu = pose_values[0]
sy_enu = pose_values[1]
sz_enu = pose_values[2]

# Gazebo ENU -> PX4 NED
sx = sy_enu
sy = sx_enu
sz = -sz_enu


# AKTIVNOST IZVORA IZ SDF KONFIGURACIJE

radiation_plugin = root.find(
    ".//plugin[@name='radiation_system::RadiationPlugin']"
)

if radiation_plugin is None:
    raise RuntimeError(
        "RadiationPlugin nije pronadjen u SDF datoteci."
    )

source_strength_element = radiation_plugin.find("source_strength")

if (
    source_strength_element is None
    or source_strength_element.text is None
):
    raise RuntimeError(
        "Parametar 'source_strength' nije pronadjen."
    )

source_activity_mbq = float(source_strength_element.text)


# PROCENJENI POLOZAJ IZVORA

max_idx = np.argmax(radiation)

est_x = x[max_idx]
est_y = y[max_idx]

estimation_error = np.sqrt(
    (est_x - sx) ** 2
    + (est_y - sy) ** 2
)


# TEORIJSKA GRANICA OBLASTI

supervised_radius = np.sqrt(
    GAMMA_CONSTANT_CS137
    * source_activity_mbq
    / THRESHOLD
)


# INTERPOLACIONA MREZA

margin = 1.0

x_min = x.min() - margin
x_max = x.max() + margin

y_min = y.min() - margin
y_max = y.max() + margin

xi = np.linspace(x_min, x_max, 500)
yi = np.linspace(y_min, y_max, 500)

X, Y = np.meshgrid(xi, yi)

# Linearna interpolacija iskljucivo na osnovu
# prikupljenih merenja.
Z = griddata(
    points=(x, y),
    values=radiation,
    xi=(X, Y),
    method="linear"
)


# LOGARITAMSKA SKALA

positive_radiation = radiation[
    np.isfinite(radiation) & (radiation > 0)
]

if len(positive_radiation) == 0:
    raise RuntimeError(
        "Nema pozitivnih vrednosti zracenja za prikaz."
    )

vmin = max(
    np.min(positive_radiation),
    0.1
)

vmax = np.max(positive_radiation)

if vmax <= vmin:
    vmax = vmin * 1.01


# FIGURA

fig, ax = plt.subplots(figsize=(9, 8))


# TOPLOTNA MAPA

heatmap = ax.pcolormesh(
    X,
    Y,
    Z,
    cmap="inferno",
    norm=LogNorm(
        vmin=vmin,
        vmax=vmax
    ),
    shading="auto",
    rasterized=True
)


# COLORBAR

cbar = fig.colorbar(
    heatmap,
    ax=ax,
    pad=0.025
)

cbar.set_label(
    "Dozna brzina [µSv/h]",
    fontsize=11
)

candidate_ticks = [
    0.1,
    0.2,
    0.5,
    1,
    2,
    3,
    5,
    10,
    20,
    30,
    50,
    80,
    100,
    200,
    500
]

tick_values = [
    value
    for value in candidate_ticks
    if vmin <= value <= vmax
]

if len(tick_values) >= 2:
    cbar.set_ticks(tick_values)
    cbar.set_ticklabels(
        [f"{value:g}" for value in tick_values]
    )


# PUTANJA LETA

ax.plot(
    x,
    y,
    "-",
    linewidth=1.3,
    label="Putanja letelice",
    zorder=5
)


# MERNE TACKE

ax.scatter(
    x,
    y,
    s=14,
    edgecolors="black",
    linewidths=0.25,
    label="Merne tačke",
    zorder=6
)



# TEORIJSKA GRANICA 3 µSv/h

theta = np.linspace(
    0,
    2 * np.pi,
    600
)

boundary_x = (
    sx
    + supervised_radius * np.cos(theta)
)

boundary_y = (
    sy
    + supervised_radius * np.sin(theta)
)

ax.plot(
    boundary_x,
    boundary_y,
    "--",
    linewidth=1.7,
    alpha=0.8,
    label=(
        f"Teorijska granica {THRESHOLD:g} µSv/h "
        f"(r = {supervised_radius:.2f} m)"
    ),
    zorder=4
)


# STVARNI POLOZAJ IZVORA

ax.scatter(
    sx,
    sy,
    marker="X",
    s=170,
    edgecolors="black",
    linewidths=1.1,
    label="Stvarni položaj izvora",
    zorder=9
)


# PROCENJENI POLOZAJ IZVORA

ax.scatter(
    est_x,
    est_y,
    marker="*",
    s=220,
    edgecolors="black",
    linewidths=1.0,
    label="Procenjeni položaj izvora",
    zorder=10
)


# GRESKA LOKALIZACIJE

ax.annotate(
    "",
    xy=(est_x, est_y),
    xytext=(sx, sy),
    arrowprops=dict(
        arrowstyle="<->",
        linewidth=1.4,
        shrinkA=8,
        shrinkB=8
    ),
    zorder=8
)

error_mid_x = (sx + est_x) / 2
error_mid_y = (sy + est_y) / 2

ax.annotate(
    f"$e = {estimation_error:.3f}\\,\\mathrm{{m}}$",
    xy=(error_mid_x, error_mid_y),
    xytext=(12, 10),
    textcoords="offset points",
    fontsize=10,
    bbox=dict(
        boxstyle="round,pad=0.25",
        facecolor="white",
        edgecolor="0.7",
        alpha=0.9
    ),
    zorder=11
)


# OSE

ax.set_xlabel("X [m]", fontsize=11)
ax.set_ylabel("Y [m]", fontsize=11)

ax.set_aspect(
    "equal",
    adjustable="box"
)

ax.tick_params(
    axis="both",
    labelsize=10
)

# Veoma diskretna mreza
ax.grid(
    alpha=0.12,
    linewidth=0.5
)


# GRANICE PRIKAZA

plot_margin = 2.0

display_x_min = min(
    x.min(),
    sx - supervised_radius
) - plot_margin

display_x_max = max(
    x.max(),
    sx + supervised_radius
) + plot_margin

display_y_min = min(
    y.min(),
    sy - supervised_radius
) - plot_margin

display_y_max = max(
    y.max(),
    sy + supervised_radius
) + plot_margin

ax.set_xlim(
    display_x_min,
    display_x_max
)

ax.set_ylim(
    display_y_min,
    display_y_max
)


# LEGENDA

ax.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.10),
    ncol=2,
    fontsize=9,
    frameon=False
)


# CUVANJE

SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

OUTPUT_PATH = os.path.join(
    SCRIPT_DIR,
    "radiation_heatmap_sphere.png"
)

fig.tight_layout()

plt.savefig(
    OUTPUT_PATH,
    dpi=300,
    bbox_inches="tight"
)

#plt.show()
