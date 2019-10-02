% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 602.223021815652032 ; 592.539724440434384 ];

%-- Principal point:
cc = [ 296.752809861117953 ; 232.993808175030438 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.233067050098747 ; 0.149808068837481 ; -0.006799777950109 ; -0.002012706225493 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 10.810770425787776 ; 11.091303297396987 ];

%-- Principal point uncertainty:
cc_error = [ 20.435236922590025 ; 19.503297415236972 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.079051224817217 ; 0.251487713618682 ; 0.005442074877919 ; 0.005670648926147 ; 0.000000000000000 ];

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
omc_1 = [ 1.627651e+00 ; 1.644151e+00 ; -7.236355e-01 ];
Tc_1  = [ -8.477858e-01 ; -3.634894e-01 ; 4.038686e+00 ];
omc_error_1 = [ 2.854540e-02 ; 3.572075e-02 ; 4.377024e-02 ];
Tc_error_1  = [ 1.372984e-01 ; 1.342642e-01 ; 7.321121e-02 ];

%-- Image #2:
omc_2 = [ 1.827250e+00 ; 1.901107e+00 ; -4.062569e-01 ];
Tc_2  = [ -7.420373e-01 ; -7.670382e-01 ; 3.528837e+00 ];
omc_error_2 = [ 3.022570e-02 ; 3.505264e-02 ; 5.397921e-02 ];
Tc_error_2  = [ 1.205723e-01 ; 1.171130e-01 ; 7.293113e-02 ];

%-- Image #3:
omc_3 = [ 1.732354e+00 ; 2.085126e+00 ; -4.827406e-01 ];
Tc_3  = [ -5.926411e-01 ; -8.578324e-01 ; 3.596264e+00 ];
omc_error_3 = [ 2.817088e-02 ; 3.712338e-02 ; 5.700558e-02 ];
Tc_error_3  = [ 1.226612e-01 ; 1.193081e-01 ; 7.121616e-02 ];

%-- Image #4:
omc_4 = [ 1.839723e+00 ; 2.120481e+00 ; -1.114589e+00 ];
Tc_4  = [ -3.042288e-01 ; -7.465490e-01 ; 3.686020e+00 ];
omc_error_4 = [ 2.493864e-02 ; 3.896041e-02 ; 5.115473e-02 ];
Tc_error_4  = [ 1.256929e-01 ; 1.212776e-01 ; 5.610354e-02 ];

%-- Image #5:
omc_5 = [ 1.064575e+00 ; 1.928858e+00 ; -2.689722e-01 ];
Tc_5  = [ -4.110004e-01 ; -1.126336e+00 ; 3.470282e+00 ];
omc_error_5 = [ 2.390491e-02 ; 3.501343e-02 ; 4.079986e-02 ];
Tc_error_5  = [ 1.194856e-01 ; 1.156440e-01 ; 6.899181e-02 ];

%-- Image #6:
omc_6 = [ -1.724358e+00 ; -1.946472e+00 ; -7.496174e-01 ];
Tc_6  = [ -7.357629e-01 ; -3.706971e-01 ; 2.032347e+00 ];
omc_error_6 = [ 2.244726e-02 ; 3.658031e-02 ; 4.957646e-02 ];
Tc_error_6  = [ 6.941464e-02 ; 6.949139e-02 ; 6.022867e-02 ];

%-- Image #7:
omc_7 = [ 1.968384e+00 ; 1.910520e+00 ; 1.294034e+00 ];
Tc_7  = [ -3.692122e-01 ; -3.531567e-01 ; 1.935016e+00 ];
omc_error_7 = [ 4.198307e-02 ; 2.235297e-02 ; 5.096223e-02 ];
Tc_error_7  = [ 6.729953e-02 ; 6.532348e-02 ; 6.209764e-02 ];

%-- Image #8:
omc_8 = [ 1.942374e+00 ; 1.813823e+00 ; 1.308175e+00 ];
Tc_8  = [ -7.956044e-01 ; -4.782142e-01 ; 2.007263e+00 ];
omc_error_8 = [ 4.015569e-02 ; 2.254173e-02 ; 4.935780e-02 ];
Tc_error_8  = [ 7.317101e-02 ; 7.048940e-02 ; 6.927202e-02 ];

%-- Image #9:
omc_9 = [ -1.359314e+00 ; -1.972905e+00 ; 3.245395e-01 ];
Tc_9  = [ -3.667271e-03 ; -1.090845e+00 ; 3.354138e+00 ];
omc_error_9 = [ 2.813359e-02 ; 3.655952e-02 ; 4.554284e-02 ];
Tc_error_9  = [ 1.149455e-01 ; 1.112892e-01 ; 7.135023e-02 ];

%-- Image #10:
omc_10 = [ -1.502803e+00 ; -2.093589e+00 ; 2.591910e-01 ];
Tc_10  = [ -9.813491e-02 ; -1.451973e+00 ; 4.098348e+00 ];
omc_error_10 = [ 3.587493e-02 ; 4.316250e-02 ; 6.253414e-02 ];
Tc_error_10  = [ 1.431515e-01 ; 1.365227e-01 ; 9.806200e-02 ];

%-- Image #11:
omc_11 = [ -1.799884e+00 ; -2.071749e+00 ; -4.104113e-01 ];
Tc_11  = [ -7.377701e-01 ; -1.129290e+00 ; 3.245543e+00 ];
omc_error_11 = [ 3.267931e-02 ; 4.024795e-02 ; 6.941152e-02 ];
Tc_error_11  = [ 1.136122e-01 ; 1.138558e-01 ; 9.702110e-02 ];

%-- Image #12:
omc_12 = [ -1.879842e+00 ; -2.130255e+00 ; -4.490654e-01 ];
Tc_12  = [ -6.481794e-01 ; -8.652543e-01 ; 2.793229e+00 ];
omc_error_12 = [ 2.810299e-02 ; 3.862970e-02 ; 6.588086e-02 ];
Tc_error_12  = [ 9.713129e-02 ; 9.643795e-02 ; 8.330507e-02 ];

%-- Image #13:
omc_13 = [ -1.961471e+00 ; -2.164351e+00 ; -4.973686e-01 ];
Tc_13  = [ -6.492353e-01 ; -7.118361e-01 ; 2.549205e+00 ];
omc_error_13 = [ 2.615359e-02 ; 3.777883e-02 ; 6.471748e-02 ];
Tc_error_13  = [ 8.832242e-02 ; 8.735009e-02 ; 7.633314e-02 ];

%-- Image #14:
omc_14 = [ -1.979978e+00 ; -2.146923e+00 ; -5.128021e-01 ];
Tc_14  = [ -6.070818e-01 ; -6.712887e-01 ; 2.284867e+00 ];
omc_error_14 = [ 2.394804e-02 ; 3.640993e-02 ; 6.120421e-02 ];
Tc_error_14  = [ 7.927261e-02 ; 7.826063e-02 ; 6.764497e-02 ];

%-- Image #15:
omc_15 = [ -2.141805e+00 ; -2.282522e+00 ; -4.136384e-01 ];
Tc_15  = [ -9.918954e-01 ; -6.560815e-01 ; 2.226655e+00 ];
omc_error_15 = [ 2.753693e-02 ; 3.387927e-02 ; 6.633750e-02 ];
Tc_error_15  = [ 7.868247e-02 ; 7.810358e-02 ; 7.240653e-02 ];

%-- Image #16:
omc_16 = [ 1.875661e+00 ; 2.333079e+00 ; -1.828890e-01 ];
Tc_16  = [ -5.019436e-02 ; -8.147912e-01 ; 3.199850e+00 ];
omc_error_16 = [ 3.704799e-02 ; 3.690155e-02 ; 7.888094e-02 ];
Tc_error_16  = [ 1.096191e-01 ; 1.054348e-01 ; 8.328865e-02 ];

%-- Image #17:
omc_17 = [ -1.630773e+00 ; -1.968465e+00 ; -2.858730e-01 ];
Tc_17  = [ -6.648735e-01 ; -6.746691e-01 ; 2.281762e+00 ];
omc_error_17 = [ 2.322611e-02 ; 3.502254e-02 ; 4.859960e-02 ];
Tc_error_17  = [ 7.826022e-02 ; 7.749194e-02 ; 5.746047e-02 ];

%-- Image #18:
omc_18 = [ -1.347652e+00 ; -1.685613e+00 ; -2.651589e-01 ];
Tc_18  = [ -9.225748e-01 ; -7.635809e-01 ; 2.050221e+00 ];
omc_error_18 = [ 2.624698e-02 ; 3.324339e-02 ; 3.701163e-02 ];
Tc_error_18  = [ 7.122890e-02 ; 7.059194e-02 ; 5.383369e-02 ];

%-- Image #19:
omc_19 = [ -1.949325e+00 ; -1.856511e+00 ; -1.424199e+00 ];
Tc_19  = [ -5.111791e-01 ; -3.687074e-01 ; 1.458752e+00 ];
omc_error_19 = [ 2.203863e-02 ; 3.966919e-02 ; 4.917536e-02 ];
Tc_error_19  = [ 5.197703e-02 ; 5.092682e-02 ; 5.176131e-02 ];

%-- Image #20:
omc_20 = [ 1.868894e+00 ; 1.575925e+00 ; 1.465816e+00 ];
Tc_20  = [ -6.673460e-01 ; -4.052553e-01 ; 1.724208e+00 ];
omc_error_20 = [ 4.067279e-02 ; 2.279185e-02 ; 4.408556e-02 ];
Tc_error_20  = [ 6.371416e-02 ; 6.044464e-02 ; 6.171080e-02 ];

