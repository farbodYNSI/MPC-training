import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t += v_t*dt*np.cos(psi_t)
        y_t += v_t*dt*np.sin(psi_t)
        psi_t += v_t*np.tan(steering)/2.5*dt
        v_t += pedal*dt - v_t/25
        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0
        for i in range(self.horizon):
            state = self.plant_model(state,self.dt,u[i*2],u[i*2+1])
            x_cost = (state[0] - ref[0])**2
            y_cost = (state[1] - ref[1])**2
            heading_cost = (state[2] - ref[2])**2
            cost += x_cost + y_cost + heading_cost
        return cost

sim_run(options, ModelPredictiveControl)
