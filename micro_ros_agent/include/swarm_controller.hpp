/**
 * @file swarm_controller.hpp
 * @brief Self-Regulated Multi-Agent Controller using Bloch-type Perceptual Memory
 * 
 * Author  : Namrata Kumari
 * Guide   : Prof. Dr. Sohom Chakrabarty
 * Institute: Indian Institute of Technology Roorkee
 * Year    : 2026
 */

#ifndef SWARM_CONTROLLER_HPP
#define SWARM_CONTROLLER_HPP

#include <Arduino.h>
#include <math.h>

#define MAX_NEIGHBORS 8

struct AgentState {
    float x;
    float y;
    float heading;
    float speed;
    float s;          // slow regulatory state
    bool  valid;
};

class SwarmController {
public:
    SwarmController(int agent_id) {
        id = agent_id;
        my_state.x = 0.0f;
        my_state.y = 0.0f;
        my_state.heading = 0.0f;
        my_state.speed = 0.18f;
        my_state.s = 0.0f;
        my_state.valid = true;
        num_neighbors = 0;

        for (int i = 0; i < MAX_NEIGHBORS; i++) {
            mx[i] = 0.0f;
            my[i] = 0.0f;
            mz[i] = 0.0f;
            neighbors[i].valid = false;
        }
    }

    void clearNeighbors() {
        num_neighbors = 0;
        for (int i = 0; i < MAX_NEIGHBORS; i++) {
            neighbors[i].valid = false;
        }
    }

    void setNeighbor(int index, float x, float y, float heading, float s) {
        if (index < 0 || index >= MAX_NEIGHBORS) return;
        neighbors[index].x = x;
        neighbors[index].y = y;
        neighbors[index].heading = heading;
        neighbors[index].s = s;
        neighbors[index].valid = true;
        if (index + 1 > num_neighbors) {
            num_neighbors = index + 1;
        }
    }

    void update(float obstacle_distance) {
        updateFastBloch();
        updateSlowState();

        float desired_heading = computeDesiredHeading(obstacle_distance);
        float desired_speed   = computeDesiredSpeed(obstacle_distance);

        // Rotate heading towards desired (bounded)
        float dh = desired_heading - my_state.heading;
        while (dh >  PI) dh -= 2 * PI;
        while (dh < -PI) dh += 2 * PI;

        float max_turn = omega_max * 0.05f;   // assuming 20 Hz
        if (dh >  max_turn) dh =  max_turn;
        if (dh < -max_turn) dh = -max_turn;

        my_state.heading += dh;
        my_state.speed = desired_speed;
    }

    float getHeading() const { return my_state.heading; }
    float getSpeed() const { return my_state.speed; }
    float getSlowState() const { return my_state.s; }
    AgentState getState() const { return my_state; }

private:
    int id;
    AgentState my_state;
    AgentState neighbors[MAX_NEIGHBORS];
    int num_neighbors;

    float mx[MAX_NEIGHBORS];
    float my[MAX_NEIGHBORS];
    float mz[MAX_NEIGHBORS];

    // Parameters from the internship report
    float T1 = 0.35f;
    float T2 = 0.18f;
    float Gamma = 2.2f;
    float kappa = 1.6f;
    float g_align = 1.3f;
    float gamma_s = 0.07f;
    float lambda_fb = 1.9f;
    float s_base = 0.0f;
    float omega_max = 1.6f;
    float v0 = 0.20f;

    void updateFastBloch() {
        for (int j = 0; j < num_neighbors; j++) {
            if (!neighbors[j].valid) continue;

            // Heading consistency
            float cons = cosf(my_state.heading) * cosf(neighbors[j].heading)
                       + sinf(my_state.heading) * sinf(neighbors[j].heading);

            // Effective field
            float heff = kappa * my_state.s + g_align * cons;
            float mz_eq = tanhf(heff);

            // Bloch derivatives
            float dmx =  2.0f * heff * my[j] - mx[j] / T2;
            float dmy = -2.0f * heff * mx[j] + 2.0f * Gamma * mz[j] - my[j] / T2;
            float dmz = -2.0f * Gamma * my[j] - (mz[j] - mz_eq) / T1;

            // Integrate
            mx[j] += dmx * 0.05f;
            my[j] += dmy * 0.05f;
            mz[j] += dmz * 0.05f;

            // Bound mz
            if (mz[j] >  1.0f) mz[j] =  1.0f;
            if (mz[j] < -1.0f) mz[j] = -1.0f;
        }
    }

    void updateSlowState() {
        float sum_mz = 0.0f;
        int count = 0;
        for (int j = 0; j < num_neighbors; j++) {
            if (neighbors[j].valid) {
                sum_mz += mz[j];
                count++;
            }
        }
        float Mi = (count > 0) ? (sum_mz / count) : 0.0f;
        float s_target = tanhf(lambda_fb * Mi);

        my_state.s += 0.05f * (-gamma_s * (my_state.s - s_base) + gamma_s * s_target);

        if (my_state.s >  1.0f) my_state.s =  1.0f;
        if (my_state.s < -1.0f) my_state.s = -1.0f;
    }

    float computeDesiredHeading(float obs_dist) {
        float ux = 0.4f * cosf(my_state.heading);
        float uy = 0.4f * sinf(my_state.heading);

        // Alignment from neighbours
        for (int j = 0; j < num_neighbors; j++) {
            if (!neighbors[j].valid) continue;
            float w = 0.5f * (mz[j] + 1.0f);
            ux += w * cosf(neighbors[j].heading);
            uy += w * sinf(neighbors[j].heading);
        }

        // Simple obstacle avoidance (front)
        if (obs_dist < 0.45f) {
            float strength = (0.45f - obs_dist) / 0.45f;
            ux -= strength * 1.8f * cosf(my_state.heading);
            uy -= strength * 1.8f * sinf(my_state.heading);
        }

        float norm = sqrtf(ux*ux + uy*uy);
        if (norm < 1e-4f) return my_state.heading;
        return atan2f(uy / norm, ux / norm);
    }

    float computeDesiredSpeed(float obs_dist) {
        float terrain_risk = 0.0f;
        if (obs_dist < 0.60f) {
            terrain_risk = expf(-obs_dist / 0.28f);
        }

        float speed = v0 * (1.0f - 0.75f * terrain_risk);

        // Slow down if no neighbours (cohesion risk)
        if (num_neighbors < 1) {
            speed *= 0.65f;
        }

        if (speed < 0.04f) speed = 0.04f;
        if (speed > v0 * 1.1f) speed = v0 * 1.1f;
        return speed;
    }
};

#endif
