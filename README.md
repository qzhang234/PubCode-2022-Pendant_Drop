# aps_droplet
This repository consists a documentation for the "Droplet" experiment to be performed at APS, Argonne National Laboratory.

## Figure Scripts
`/Data_Analysis/Figure_scripts/` 

- Generate SAXS 1D figure. `/Data_Analysis/Figure_scripts/Plot_SAXS.ipynb`
- Generate g2 vs. τ and τ0(Q) vs. Q figure. `/Data_Analysis/Figure_scripts/Plot_g2_tauQ.ipynb`
- Generate casing vs noncasing droplet figure. `/Data_Analysis/Figure_scripts/Plot_noisy_vs_isolated_droplet.ipynb`
- Generate SamX vs SamZ figure. `/Data_Analysis/Figure_scripts/Plot_samx_vs_samz.ipynb`
- Generate Droplet setup figure. `/Data_Analysis/Figure_scripts/Plot_droplet_setup.ipynb`

## Figures
`/Data_Analysis/Figures/`
## Data
- `/Data_Analysis/Data`
Assembly of .hdf result files from APS Beamline 8-ID-I containing g2 and I vs. Q data for plotting the figures in the manuscript.
## Python script
- `/ur3_driver/ur3_driver/ur3_driver.py` 
This program performs the full experiment by controlling UR3e robot, tool changer, electronic pipette and the camera. 
# Installation 

To install the URx and PyEPICS libraries 

- `pip install urx`
- `pip install pyepics`
