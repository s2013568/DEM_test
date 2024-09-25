### Don't be Alarmed, Takodachi6969 is my old acc with free co-pilot!!!! (Im Peter btw)
Go to test zone, run Test.ipynb

This is the method by 
DISCRETE PARTICLE SIMULATION OF BUBBLE AND SLUG
FORMATION IN A TWO-DIMENSIONAL GAS-FLUIDISED
BED: A HARD-SPHERE APPROACH 

Currently bare minimum, gonna add in seuqnece of collision in dense limit, and hydrodynamic force. 

The code structure is as follows

DEM_mains is where the main loop fo the simulation runs. Particle_definition has the parameters for particles and walls. Collision method gives you different method of collision that might be able to try out, they are the maths behind the simulation. calculation tools will be misc functions that i don't want to write in the main loop

## Next Step
1. add collision list to resolve to smallest timestep of collision, and resolve issues with collision sequence
2. using analytical method to calculate gravity, but it is worth killing it after all
3. visualise angular velocity
4. make a cut off where the velocity below certain speed will be set to 0 to avoid perturbation costing computing resource
5. generalise it to other methods, attempt to generalise this onto a crowd simulation model
