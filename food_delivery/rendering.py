"""Offline plots/MP4 plus an HTML replay with optional interactive OSM tiles."""
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def render(result, output: Path, video=True):
    payload = json.dumps(result, allow_nan=False).replace('<', '\\u003c')
    template = Path(__file__).with_name('replay.html').read_text(encoding='utf-8')
    (output / 'replay.html').write_text(template.replace('__FLIGHT_DATA__', payload), encoding='utf-8')
    samples = result['samples']
    true = np.array([s['true'] for s in samples])
    estimated = np.array([s['estimated'] for s in samples])
    times = np.array([s['time_s'] for s in samples])
    errors = np.array([s['error_m'] for s in samples])
    cfg = result['scenario']
    plt.style.use('dark_background')
    fig, (ax, chart) = plt.subplots(1, 2, figsize=(12.8, 7.2), dpi=100,
                                  gridspec_kw={'width_ratios': [1.2, 1]})
    fig.patch.set_facecolor('#101b29')
    for axis in (ax, chart):
        axis.set_facecolor('#142235')
        axis.grid(alpha=0.15)
    ax.plot([cfg['origin'][1], cfg['destination'][1]],
            [cfg['origin'][0], cfg['destination'][0]], '--', color='#94a3b8', label='Direct route')
    ax.scatter([cfg['origin'][1], cfg['destination'][1]],
               [cfg['origin'][0], cfg['destination'][0]], color=['#4ade80', '#fbbf24'], s=70)
    ax.annotate('Departure', xy=cfg['origin'][::-1], xytext=(5, -18), textcoords='offset points')
    ax.annotate('Destination', xy=cfg['destination'][::-1], xytext=(-75, 12), textcoords='offset points')
    all_points = np.concatenate([true, estimated, np.array([cfg['origin'], cfg['destination']])])
    ax.set_xlim(all_points[:, 1].min() - .06, all_points[:, 1].max() + .06)
    ax.set_ylim(all_points[:, 0].min() - .06, all_points[:, 0].max() + .06)
    ax.set_aspect(1 / np.cos(np.deg2rad(true[:, 0].mean())))
    ax.set_xlabel('Longitude (degrees)')
    ax.set_ylabel('WGS84 latitude (degrees)')
    ax.set_title('Flight track · geographic coordinates')
    truth_line, = ax.plot([], [], color='#38bdf8', lw=2, label='Actual')
    estimate_line, = ax.plot([], [], color='#fb923c', lw=1.5, label='Estimated')
    marker, = ax.plot([], [], 'o', color='#38bdf8', ms=8)
    ax.legend(loc='lower right')
    chart.set_title('Navigation error')
    chart.set_xlabel('Simulated time (minutes)')
    chart.set_ylabel('Position error (metres)')
    chart.set_xlim(0, max(times[-1] / 60, 1))
    chart.set_ylim(0, max(errors.max() * 1.15, 30))
    chart.axvspan(cfg['outage_start_s'] / 60, cfg['outage_end_s'] / 60, color='#fb923c', alpha=.15, label='GPS outage')
    error_line, = chart.plot([], [], color='#fb923c')
    cursor = chart.axvline(0, color='#38bdf8', alpha=.6)
    chart.legend()
    status = fig.text(.13, .035, '', fontsize=12, linespacing=1.8)
    fig.suptitle('LARKFEN / FOOD DELIVERY\nJSBSim fixed-wing navigation experiment', fontsize=18, y=.98)
    fig.subplots_adjust(top=.82, bottom=.22, wspace=.3)

    def frame(index):
        s = samples[index]
        truth_line.set_data(true[:index + 1, 1], true[:index + 1, 0])
        estimate_line.set_data(estimated[:index + 1, 1], estimated[:index + 1, 0])
        marker.set_data([true[index, 1]], [true[index, 0]])
        error_line.set_data(times[:index + 1] / 60, errors[:index + 1])
        cursor.set_xdata([times[index] / 60] * 2)
        status.set_text(f"t = {s['time_s'] / 60:.1f} min    GPS: {'ON' if s['gps_available'] else 'OUTAGE'}\n"
                        f"Altitude: {s['altitude_m']:.0f} m MSL    Remaining: {s['remaining_m'] / 1000:.1f} km")
        fig.canvas.draw()
        return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()

    frame(len(samples) - 1)
    fig.savefig(output / 'overview.png', facecolor=fig.get_facecolor())
    if video:
        # About 45 seconds independent of mission duration; encoding is separate
        # from physics so the benchmark excludes all plotting/video overhead.
        indices = np.linspace(0, len(samples) - 1, min(len(samples), 900), dtype=int)
        with imageio.get_writer(output / 'flight.mp4', fps=20, codec='libx264',
                                quality=7, macro_block_size=16) as writer:
            for index in indices:
                writer.append_data(frame(index))
    plt.close(fig)
