#!/usr/bin/env python3
"""
Self-Regulated Multi-Agent Motion Algorithm
Complete Simulation from Scratch

Author  : Namrata Kumari
Guide   : Prof. Dr. Sohom Chakrabarty
Institute: Indian Institute of Technology Roorkee
Year    : 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

# ======================== CONFIGURATION ========================
N = 8
DT = 0.05
SIM_TIME = 45.0
R = 1.8
V0 = 0.22
OMEGA_MAX = 1.6

T1 = 0.35
T2 = 0.18
GAMMA = 2.2
KAPPA = 1.6
G_ALIGN = 1.3
GAMMA_S = 0.07
LAMBDA_FB = 1.9
S_BASE = 0.0

K_SEP = 1.4
K_OBS = 2.8
SOCIAL_SIGMA = 0.45

ARENA_SIZE = 12.0
OBSTACLES = [
    (3.5, 4.0, 0.70),
    (7.5, 3.0, 0.60),
    (5.0, 7.5, 0.80),
    (9.0, 8.0, 0.55),
]

# ===============================================================

class SwarmSimulation:
    def __init__(self):
        self.N = N
        self.pos = np.random.uniform(1.5, 4.0, size=(N, 2))
        self.heading = np.random.uniform(-np.pi, np.pi, size=N)
        self.speed = np.full(N, V0)
        self.s = np.full(N, S_BASE)
        self.mx = np.zeros((N, N))
        self.my = np.zeros((N, N))
        self.mz = np.zeros((N, N))
        self.time = 0.0
        self.history = []

    def get_neighbors(self):
        delta = self.pos[None, :, :] - self.pos[:, None, :]
        dist = np.linalg.norm(delta, axis=2)
        adj = (dist < R) & (dist > 1e-6)
        return adj, dist, delta

    def update_fast_bloch(self, adj):
        e = np.stack([np.cos(self.heading), np.sin(self.heading)], axis=1)
        cons = e @ e.T
        hz = KAPPA * self.s[:, None] + G_ALIGN * cons
        hz = np.where(adj, hz, 0.0)
        mz_eq = np.tanh(hz)
        dmx =  2.0 * hz * self.my - self.mx / T2
        dmy = -2.0 * hz * self.mx + 2.0 * GAMMA * self.mz - self.my / T2
        dmz = -2.0 * GAMMA * self.my - (self.mz - mz_eq) / T1
        self.mx = np.where(adj, self.mx + DT * dmx, 0.0)
        self.my = np.where(adj, self.my + DT * dmy, 0.0)
        self.mz = np.clip(np.where(adj, self.mz + DT * dmz, 0.0), -1.0, 1.0)

    def update_slow_state(self, adj):
        Mi = np.where(adj, self.mz, 0.0).sum(1) / np.maximum(adj.sum(1), 1)
        s_target = np.tanh(LAMBDA_FB * Mi)
        self.s += DT * (-GAMMA_S * (self.s - S_BASE) + GAMMA_S * s_target)
        self.s = np.clip(self.s, -1.0, 1.0)

    def compute_desired_heading(self, adj, dist, delta):
        e = np.stack([np.cos(self.heading), np.sin(self.heading)], axis=1)
        w = np.where(adj, 0.5 * (self.mz + 1.0), 0.0)
        D = w @ e
        for i in range(self.N):
            for j in range(self.N):
                if adj[i, j] and dist[i, j] < 0.55:
                    force = (0.55 - dist[i, j]) / 0.55
                    D[i] -= K_SEP * force * delta[i, j] / (dist[i, j] + 1e-6)
        for ox, oy, orad in OBSTACLES:
            dvec = self.pos - np.array([ox, oy])
            d = np.linalg.norm(dvec, axis=1) - orad
            mask = (d > 0) & (d < 1.3)
            strength = np.exp(-np.maximum(d, 0) / 0.4)
            for i in np.where(mask)[0]:
                D[i] += K_OBS * strength[i] * dvec[i] / (np.linalg.norm(dvec[i]) + 1e-6)
        norms = np.linalg.norm(D, axis=1, keepdims=True)
        norms = np.maximum(norms, 1e-6)
        return D / norms

    def update_velocity(self, adj, dist):
        sigma = SOCIAL_SIGMA * R
        W = np.exp(-(dist**2) / (2 * sigma**2)) * adj
        N_eff = W.sum(1)
        N_ref = np.percentile(N_eff, 85) + 1e-6
        cohesion_risk = np.clip(1.0 - N_eff / N_ref, 0.0, 1.0)
        terrain_risk = np.zeros(self.N)
        for ox, oy, orad in OBSTACLES:
            d = np.linalg.norm(self.pos - np.array([ox, oy]), axis=1) - orad
            influence = np.exp(-np.maximum(d, 0) / 0.35)
            terrain_risk = np.maximum(terrain_risk, influence)
        self.speed = V0 * (1.0 - 0.75 * terrain_risk) * (1.0 - 0.35 * cohesion_risk)
        self.speed = np.clip(self.speed, 0.05, V0 * 1.15)

    def step(self):
        adj, dist, delta = self.get_neighbors()
        self.update_fast_bloch(adj)
        self.update_slow_state(adj)
        target = self.compute_desired_heading(adj, dist, delta)
        self.update_velocity(adj, dist)
        current_e = np.stack([np.cos(self.heading), np.sin(self.heading)], axis=1)
        cross = current_e[:, 0]*target[:, 1] - current_e[:, 1]*target[:, 0]
        dot = np.sum(current_e * target, axis=1)
        angle_err = np.arctan2(cross, dot)
        turn = np.clip(angle_err, -OMEGA_MAX*DT, OMEGA_MAX*DT)
        self.heading += turn
        self.heading = (self.heading + np.pi) % (2*np.pi) - np.pi
        self.pos[:, 0] += self.speed * np.cos(self.heading) * DT
        self.pos[:, 1] += self.speed * np.sin(self.heading) * DT
        self.pos = np.clip(self.pos, 0.3, ARENA_SIZE-0.3)
        self.time += DT
        self.history.append((self.pos.copy(), self.heading.copy(), self.s.copy()))

    def run(self):
        steps = int(SIM_TIME / DT)
        print(f"Running simulation for {SIM_TIME} seconds...")
        for i in range(steps):
            self.step()
            if i % 50 == 0:
                print(f"t = {self.time:5.1f}s | mean |s| = {np.mean(np.abs(self.s)):.3f}")
        print("Simulation completed successfully.")
        return self.history


def animate(history):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, ARENA_SIZE)
    ax.set_ylim(0, ARENA_SIZE)
    ax.set_aspect('equal')
    ax.set_title("Self-Regulated Multi-Agent Platoon\nAuthor: Namrata Kumari | Guide: Prof. Dr. Sohom Chakrabarty", fontsize=12)
    ax.grid(True, alpha=0.3)
    for ox, oy, orad in OBSTACLES:
        ax.add_patch(Circle((ox, oy), orad, color='dimgray', alpha=0.75))
    scat = ax.scatter([], [], s=130, edgecolors='k', zorder=5)
    quiv = ax.quiver([], [], [], [], scale=14, width=0.006, color='crimson')
    trails = [ax.plot([], [], '-', alpha=0.35, lw=1.2)[0] for _ in range(N)]
    trail_data = [[] for _ in range(N)]

    def update(frame):
        pos, heading, s = history[frame]
        scat.set_offsets(pos)
        scat.set_color(plt.cm.coolwarm((s + 1)/2))
        u, v = np.cos(heading), np.sin(heading)
        quiv.set_offsets(pos)
        quiv.set_UVC(u, v)
        for i in range(N):
            trail_data[i].append(pos[i].copy())
            if len(trail_data[i]) > 90:
                trail_data[i].pop(0)
            t = np.array(trail_data[i])
            trails[i].set_data(t[:,0], t[:,1])
        ax.set_xlabel(f"Time = {frame*DT:.1f} s")
        return [scat, quiv] + trails

    ani = FuncAnimation(fig, update, frames=len(history), interval=30, blit=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("="*65)
    print("  Self-Regulated Multi-Agent Motion Algorithm")
    print("  Author : Namrata Kumari")
    print("  Guide  : Prof. Dr. Sohom Chakrabarty")
    print("  IIT Roorkee")
    print("="*65)
    sim = SwarmSimulation()
    history = sim.run()
    print("\nOpening animation window...")
    animate(history)
