Go to test zone, run Test.ipynb

This is the method by 
DISCRETE PARTICLE SIMULATION OF BUBBLE AND SLUG
FORMATION IN A TWO-DIMENSIONAL GAS-FLUIDISED
BED: A HARD-SPHERE APPROACH 

Currently bare minimum, gonna add in seuqnece of collision in dense limit, and hydrodynamic force. 

The code structure is as follows

DEM_mains is where the main loop fo the simulation runs. Particle_definition has the parameters for particles and walls. Collision method gives you different method of collision that might be able to try out, they are the maths behind the simulation. calculation tools will be misc functions that i don't want to write in the main loop