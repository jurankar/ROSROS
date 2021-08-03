-These model files are created in gazebo when you have .dae mesh and you want to create model in gazebo.

-How it differs from .dae file? It specifies links, mass, inertia i.e. all the other things besides mesh that are needen for physics simulation.

-Lastly one interesting "bug" is that if you import .dae mesh with texture, gazebo creaes Gray material on top of it which you have to manually delete in SDF for texture to be visible.