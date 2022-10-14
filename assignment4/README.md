# Assignment 4

Edit this 'README.md' file to report all your results. There is no need to write lengthy reports, just show the requested outputs and screenshots and quickly summarize your observations. Please add your additional files or notes in the folder 'assignment4/results' and refer to or directly show them in this page.

## Required results

* Screenshots of the parameterizations and textured (checkerboard) models for all the implemented methods and boundary conditions (models: cathead.obj, hemisphere.off, hemisphere_non_convex_boundary.off,Octo_cut2.obj)
* Several examples of the distortion visualizations.

For the free boundary condtion, I took the first vertex and the "middle" vertex from igl::boundary_loop. Since the edges in the boundary loop don't differ in length that much, they make an good approximation for the most distance vertices.

#### Cathead
Uniform Laplacian\
![](results/cat_uni_lap.png)

Cotangent Laplacian

![](results/cat_cot_lap.png)

LSCM free boundary

![](results/cat_lscm.png)

LSCM fix boundary

![](results/cat_lscm_fixbound.png)

Uniform Laplacian UV

![](results/cat_uni_lap_uv.png)

Cotangent Laplacian UV

![](results/cat_cot_lap_uv.png)

LSCM free boundary UV

![](results/cat_lscm_uv.png)

LSCM fix boundary UV

![](results/cat_lscm_fixbound_uv.png)

#### Hemisphere
Uniform Laplacian\
![](results/hemisphere_uni_lap.png)

Cotangent Laplacian

![](results/hemisphere_cot_lap.png)

LSCM free boundary

![](results/hemisphere_lscm.png)

LSCM fix boundary

![](results/hemisphere_lscm_fixbound.png)

Uniform Laplacian UV

![](results/hemisphere_uni_lap_uv.png)

Cotangent Laplacian UV

![](results/hemisphere_cot_lap_uv.png)

LSCM free boundary UV

![](results/hemisphere_lscm_uv.png)

LSCM fix boundary UV

![](results/hemisphere_lscm_fixbound_uv.png)

#### Hemisphere Non Convex Boundary
Uniform Laplacian\
![](results/hemisphere_nonconvex_uni_lap.png)

Cotangent Laplacian

![](results/hemisphere_nonconvex_cot_lap.png)

LSCM free boundary

![](results/hemisphere_nonconvex_lscm.png)

Uniform Laplacian UV

![](results/hemisphere_nonconvex_uni_lap_uv.png)

Cotangent Laplacian UV

![](results/hemisphere_nonconvex_cot_lap_uv.png)

LSCM free boundary UV

![](results/hemisphere_nonconvex_lscm_uv.png)

#### Octo
Uniform Laplacian\
![](results/octo_uni_lap.png)

Cotangent Laplacian

![](results/octo_cot_lap.png)

LSCM free boundary

![](results/octo_lscm.png)

LSCM fix boundary

![](results/octo_lscm_fixbound.png)

Uniform Laplacian UV

![](results/octo_uni_lap_uv.png)

Cotangent Laplacian UV

![](results/octo_cot_lap_uv.png)

LSCM free boundary UV

![](results/octo_lscm_uv.png)

LSCM fix boundary UV

![](results/octo_lscm_fixbound_uv.png)


ARAP distortion doesn't seem to work...\
Haven't visualize the distortion...
