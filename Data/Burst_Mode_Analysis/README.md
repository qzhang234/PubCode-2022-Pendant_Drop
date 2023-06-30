
This folder contains the burst mode data downloaded from:
`/home/8ididata/2022-2/babnigg202207/Test_Burst`.

The `.mat` result files were generated in the following manner:

1. Q Map is extracted from 50kHz results `./cluster_results/'F091_D100_Capillary_Post_att00_Lq0_Rq0_00001_0001-100000` and converted into `mask_full_09_01.mat` form using code `Get_Mask_From_HDF.m`.

2. The 50 burst mode `.bin` files located at `/babnigg202207/Test_Burst_D100/` are processed on kouga using code `run_UFXC_loop_burst_kouga.m`. Analysis takes roughly a day because the Matlab procedures are very insufficient and the Q Map is large. This results in 42 files (some of the measurements were corrupted), which is averaged using Matlab XPCSGUI into `D100_Burst_avg.mat`. 

3. g2 from ROI2 is read from `D100_Burst_avg.mat` into `burst_plot.csv` using `Collapse_g2.m`.

4. g2 from burst mode is plotted together with averaged `F091` files using Matplotlib code which becomes Figure 4. 

