% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 560.748767210676192 ; 560.014818379457097 ];

%-- Principal point:
cc = [ 319.245612429835319 ; 240.106391319233296 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.077675677500739 ; -0.128921893178592 ; 0.000053331095470 ; 0.000952595317256 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.116168306081537 ; 1.209532671798660 ];

%-- Principal point uncertainty:
cc_error = [ 1.865361338925625 ; 1.369513501488809 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.009295730029754 ; 0.028365971950667 ; 0.000991942896796 ; 0.001324391442701 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.815072e+00 ; -2.163196e+00 ; -3.207882e-01 ];
Tc_1  = [ -6.952293e-02 ; -9.182096e-02 ; 3.096497e-01 ];
omc_error_1 = [ 3.474289e-03 ; 4.437727e-03 ; 7.546153e-03 ];
Tc_error_1  = [ 1.056968e-03 ; 7.927908e-04 ; 9.007405e-04 ];

%-- Image #2:
omc_2 = [ -2.470555e+00 ; -1.685166e+00 ; -5.928678e-01 ];
Tc_2  = [ -1.181748e-01 ; -3.485768e-02 ; 3.116991e-01 ];
omc_error_2 = [ 4.568502e-03 ; 3.403550e-03 ; 8.427686e-03 ];
Tc_error_2  = [ 1.080366e-03 ; 8.030875e-04 ; 1.027170e-03 ];

%-- Image #3:
omc_3 = [ 1.953393e+00 ; 1.945973e+00 ; -4.205216e-01 ];
Tc_3  = [ -8.018774e-02 ; -7.993657e-02 ; 3.141095e-01 ];
omc_error_3 = [ 2.532986e-03 ; 3.409543e-03 ; 5.489474e-03 ];
Tc_error_3  = [ 1.057725e-03 ; 7.750994e-04 ; 7.250802e-04 ];

%-- Image #4:
omc_4 = [ -2.048677e+00 ; -2.008707e+00 ; -9.706109e-01 ];
Tc_4  = [ -7.460472e-02 ; -4.862614e-02 ; 1.647692e-01 ];
omc_error_4 = [ 2.116573e-03 ; 2.944993e-03 ; 5.002835e-03 ];
Tc_error_4  = [ 5.837688e-04 ; 4.363760e-04 ; 5.353393e-04 ];

%-- Image #5:
omc_5 = [ -1.966312e+00 ; -1.971721e+00 ; -6.244822e-01 ];
Tc_5  = [ -8.987967e-02 ; -8.650521e-02 ; 3.599530e-01 ];
omc_error_5 = [ 4.016700e-03 ; 4.322705e-03 ; 7.820264e-03 ];
Tc_error_5  = [ 1.234483e-03 ; 9.312730e-04 ; 1.136882e-03 ];

%-- Image #6:
omc_6 = [ -1.761376e+00 ; -1.830967e+00 ; -3.722246e-01 ];
Tc_6  = [ -9.650920e-02 ; -6.378081e-02 ; 2.120461e-01 ];
omc_error_6 = [ 2.190219e-03 ; 2.925525e-03 ; 4.397247e-03 ];
Tc_error_6  = [ 7.220845e-04 ; 5.472479e-04 ; 5.829967e-04 ];

%-- Image #7:
omc_7 = [ -1.564387e+00 ; -1.680910e+00 ; -6.174959e-01 ];
Tc_7  = [ -9.538864e-02 ; -5.324773e-02 ; 1.911713e-01 ];
omc_error_7 = [ 2.033016e-03 ; 3.002785e-03 ; 3.815198e-03 ];
Tc_error_7  = [ 6.494938e-04 ; 4.968039e-04 ; 5.513000e-04 ];

%-- Image #8:
omc_8 = [ -1.853395e+00 ; -1.793712e+00 ; -1.086881e+00 ];
Tc_8  = [ -6.394673e-02 ; -2.736792e-02 ; 1.875881e-01 ];
omc_error_8 = [ 2.133079e-03 ; 3.176156e-03 ; 4.532893e-03 ];
Tc_error_8  = [ 6.405600e-04 ; 4.784915e-04 ; 5.728681e-04 ];

%-- Image #9:
omc_9 = [ 2.024537e+00 ; 1.698942e+00 ; -4.152836e-02 ];
Tc_9  = [ -8.033424e-02 ; -6.371638e-02 ; 1.947329e-01 ];
omc_error_9 = [ 2.368248e-03 ; 2.627241e-03 ; 4.376001e-03 ];
Tc_error_9  = [ 6.658979e-04 ; 4.838684e-04 ; 5.200921e-04 ];

%-- Image #10:
omc_10 = [ 2.040515e+00 ; 2.233432e+00 ; -7.202834e-01 ];
Tc_10  = [ -8.881933e-02 ; -7.336003e-02 ; 2.409624e-01 ];
omc_error_10 = [ 1.750152e-03 ; 3.025907e-03 ; 5.001909e-03 ];
Tc_error_10  = [ 8.154499e-04 ; 5.995491e-04 ; 5.284488e-04 ];

%-- Image #11:
omc_11 = [ 1.992198e+00 ; 1.978069e+00 ; -3.489358e-01 ];
Tc_11  = [ -8.560446e-02 ; -7.656333e-02 ; 2.227846e-01 ];
omc_error_11 = [ 2.056304e-03 ; 2.917413e-03 ; 4.742159e-03 ];
Tc_error_11  = [ 7.576916e-04 ; 5.529611e-04 ; 5.545095e-04 ];

%-- Image #12:
omc_12 = [ 2.367389e+00 ; 1.924502e+00 ; 4.830967e-01 ];
Tc_12  = [ -9.108993e-02 ; -6.466432e-02 ; 3.281360e-01 ];
omc_error_12 = [ 4.724334e-03 ; 3.992114e-03 ; 9.478199e-03 ];
Tc_error_12  = [ 1.136880e-03 ; 8.351843e-04 ; 1.156456e-03 ];

%-- Image #13:
omc_13 = [ -1.917417e+00 ; -1.923904e+00 ; -5.400047e-01 ];
Tc_13  = [ -8.278589e-02 ; -5.244978e-02 ; 1.816094e-01 ];
omc_error_13 = [ 2.082461e-03 ; 2.941669e-03 ; 4.649700e-03 ];
Tc_error_13  = [ 6.253316e-04 ; 4.723127e-04 ; 5.202900e-04 ];

%-- Image #14:
omc_14 = [ -2.040246e+00 ; -2.196699e+00 ; 3.018526e-02 ];
Tc_14  = [ -5.428493e-02 ; -9.342273e-02 ; 3.600091e-01 ];
omc_error_14 = [ 5.523042e-03 ; 6.233369e-03 ; 1.191914e-02 ];
Tc_error_14  = [ 1.217436e-03 ; 8.972851e-04 ; 1.095446e-03 ];

%-- Image #15:
omc_15 = [ -1.787545e+00 ; -1.964898e+00 ; -2.972842e-01 ];
Tc_15  = [ -6.897575e-02 ; -8.420797e-02 ; 3.270347e-01 ];
omc_error_15 = [ 3.084818e-03 ; 4.026629e-03 ; 6.400109e-03 ];
Tc_error_15  = [ 1.107545e-03 ; 8.257670e-04 ; 8.679308e-04 ];

%-- Image #16:
omc_16 = [ -2.228060e+00 ; -2.101952e+00 ; -4.906238e-01 ];
Tc_16  = [ -8.858560e-02 ; -6.526698e-02 ; 3.084449e-01 ];
omc_error_16 = [ 4.333121e-03 ; 4.181887e-03 ; 8.767227e-03 ];
Tc_error_16  = [ 1.064454e-03 ; 7.902715e-04 ; 1.023817e-03 ];

%-- Image #17:
omc_17 = [ 2.128885e+00 ; 1.641822e+00 ; 9.648629e-03 ];
Tc_17  = [ -9.224719e-02 ; -7.978743e-02 ; 3.405866e-01 ];
omc_error_17 = [ 3.281926e-03 ; 3.524739e-03 ; 6.150665e-03 ];
Tc_error_17  = [ 1.154289e-03 ; 8.471210e-04 ; 9.941583e-04 ];

%-- Image #18:
omc_18 = [ 2.028618e+00 ; 2.055835e+00 ; -1.011752e+00 ];
Tc_18  = [ -7.513895e-02 ; -5.512537e-02 ; 2.846439e-01 ];
omc_error_18 = [ 1.956200e-03 ; 3.006903e-03 ; 4.976401e-03 ];
Tc_error_18  = [ 9.561106e-04 ; 6.996808e-04 ; 5.096179e-04 ];

%-- Image #19:
omc_19 = [ -1.509578e+00 ; -2.045627e+00 ; -1.019273e-01 ];
Tc_19  = [ -6.957043e-02 ; -8.417710e-02 ; 2.519082e-01 ];
omc_error_19 = [ 2.160316e-03 ; 3.335338e-03 ; 4.477815e-03 ];
Tc_error_19  = [ 8.523572e-04 ; 6.312305e-04 ; 6.120532e-04 ];

%-- Image #20:
omc_20 = [ -2.028112e+00 ; -1.798078e+00 ; -1.292303e+00 ];
Tc_20  = [ -5.148411e-02 ; -2.467981e-02 ; 1.548915e-01 ];
omc_error_20 = [ 2.054828e-03 ; 3.117032e-03 ; 4.736026e-03 ];
Tc_error_20  = [ 5.356576e-04 ; 3.959545e-04 ; 5.110909e-04 ];

