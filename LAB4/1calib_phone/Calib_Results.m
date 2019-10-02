% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 419.395636993367702 ; 419.953095345567021 ];

%-- Principal point:
cc = [ 180.158068663903407 ; 239.959755953868211 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.076135731574438 ; -0.122408520508506 ; 0.001177295479719 ; 0.000570786123310 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.533668808629222 ; 0.539516045452719 ];

%-- Principal point uncertainty:
cc_error = [ 0.504292792656737 ; 0.626701345253101 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003405772241587 ; 0.008052309037836 ; 0.000535560713327 ; 0.000418599946092 ; 0.000000000000000 ];

%-- Image size:
nx = 360;
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
omc_1 = [ -1.953639e+00 ; -1.969059e+00 ; -4.457274e-01 ];
Tc_1  = [ -8.693497e-02 ; -1.170802e-01 ; 2.244808e-01 ];
omc_error_1 = [ 1.624930e-03 ; 1.566281e-03 ; 2.838131e-03 ];
Tc_error_1  = [ 2.917245e-04 ; 3.707590e-04 ; 4.016796e-04 ];

%-- Image #2:
omc_2 = [ 1.953318e+00 ; 1.940471e+00 ; -4.884322e-01 ];
Tc_2  = [ -8.396150e-02 ; -1.720578e-02 ; 2.739906e-01 ];
omc_error_2 = [ 1.535773e-03 ; 1.598868e-03 ; 2.852916e-03 ];
Tc_error_2  = [ 3.259796e-04 ; 4.158041e-04 ; 3.599984e-04 ];

%-- Image #3:
omc_3 = [ -1.901336e+00 ; -1.959693e+00 ; 1.594837e-01 ];
Tc_3  = [ -5.812025e-02 ; -9.946231e-02 ; 2.262726e-01 ];
omc_error_3 = [ 1.376249e-03 ; 1.128266e-03 ; 2.219198e-03 ];
Tc_error_3  = [ 2.796675e-04 ; 3.426374e-04 ; 3.386459e-04 ];

%-- Image #4:
omc_4 = [ 2.147539e+00 ; 2.222744e+00 ; 5.449877e-01 ];
Tc_4  = [ -5.570270e-02 ; -8.604431e-02 ; 1.772254e-01 ];
omc_error_4 = [ 1.282585e-03 ; 1.138748e-03 ; 2.578381e-03 ];
Tc_error_4  = [ 2.309497e-04 ; 2.816792e-04 ; 3.081998e-04 ];

%-- Image #5:
omc_5 = [ 2.037551e+00 ; 2.215557e+00 ; 3.980743e-01 ];
Tc_5  = [ -5.251239e-02 ; -1.199566e-01 ; 2.461328e-01 ];
omc_error_5 = [ 1.529702e-03 ; 1.693267e-03 ; 3.350560e-03 ];
Tc_error_5  = [ 3.172235e-04 ; 3.821620e-04 ; 4.370811e-04 ];

%-- Image #6:
omc_6 = [ -1.864365e+00 ; -1.989268e+00 ; 4.074000e-01 ];
Tc_6  = [ -3.146742e-02 ; -1.218530e-01 ; 2.744502e-01 ];
omc_error_6 = [ 1.510445e-03 ; 1.226172e-03 ; 2.493747e-03 ];
Tc_error_6  = [ 3.441349e-04 ; 4.074322e-04 ; 3.859947e-04 ];

%-- Image #7:
omc_7 = [ -1.308579e+00 ; -1.646779e+00 ; -2.946676e-01 ];
Tc_7  = [ -7.059582e-02 ; -9.699608e-02 ; 1.856341e-01 ];
omc_error_7 = [ 1.275962e-03 ; 1.172987e-03 ; 1.570787e-03 ];
Tc_error_7  = [ 2.346589e-04 ; 2.914493e-04 ; 3.205906e-04 ];

%-- Image #8:
omc_8 = [ -1.841113e+00 ; -1.796348e+00 ; -1.078113e+00 ];
Tc_8  = [ -5.773829e-02 ; -8.296004e-02 ; 1.757992e-01 ];
omc_error_8 = [ 1.152446e-03 ; 1.510380e-03 ; 2.097723e-03 ];
Tc_error_8  = [ 2.287986e-04 ; 2.905951e-04 ; 3.517024e-04 ];

%-- Image #9:
omc_9 = [ 2.040647e+00 ; 2.059589e+00 ; 3.968094e-01 ];
Tc_9  = [ -8.636314e-02 ; -1.114193e-01 ; 3.428446e-01 ];
omc_error_9 = [ 2.293696e-03 ; 2.417059e-03 ; 4.615639e-03 ];
Tc_error_9  = [ 4.369218e-04 ; 5.275931e-04 ; 6.200581e-04 ];

%-- Image #10:
omc_10 = [ -1.885705e+00 ; -1.945564e+00 ; 4.970104e-02 ];
Tc_10  = [ -4.863996e-02 ; -1.201514e-01 ; 3.699760e-01 ];
omc_error_10 = [ 2.299497e-03 ; 1.951585e-03 ; 3.851051e-03 ];
Tc_error_10  = [ 4.564662e-04 ; 5.607117e-04 ; 6.059925e-04 ];

%-- Image #11:
omc_11 = [ 2.122117e+00 ; 2.150688e+00 ; -1.727822e-01 ];
Tc_11  = [ -5.782121e-02 ; -8.040118e-02 ; 3.188589e-01 ];
omc_error_11 = [ 2.081587e-03 ; 1.969779e-03 ; 4.309582e-03 ];
Tc_error_11  = [ 3.868142e-04 ; 4.749140e-04 ; 4.827506e-04 ];

%-- Image #12:
omc_12 = [ -1.912192e+00 ; -2.086367e+00 ; 4.700097e-01 ];
Tc_12  = [ -6.537383e-02 ; -9.091815e-02 ; 2.081364e-01 ];
omc_error_12 = [ 1.301258e-03 ; 9.484008e-04 ; 2.179580e-03 ];
Tc_error_12  = [ 2.569743e-04 ; 3.114617e-04 ; 2.918813e-04 ];

%-- Image #13:
omc_13 = [ 2.036370e+00 ; 2.057864e+00 ; 2.906821e-01 ];
Tc_13  = [ -6.766383e-02 ; -1.119826e-01 ; 2.124019e-01 ];
omc_error_13 = [ 1.297179e-03 ; 1.305453e-03 ; 2.570742e-03 ];
Tc_error_13  = [ 2.745719e-04 ; 3.278655e-04 ; 3.623384e-04 ];

%-- Image #14:
omc_14 = [ -1.950259e+00 ; -2.005459e+00 ; -3.478532e-01 ];
Tc_14  = [ -6.518210e-02 ; -1.079551e-01 ; 2.102441e-01 ];
omc_error_14 = [ 1.496763e-03 ; 1.354230e-03 ; 2.649598e-03 ];
Tc_error_14  = [ 2.697897e-04 ; 3.361130e-04 ; 3.663853e-04 ];

%-- Image #15:
omc_15 = [ 2.172500e+00 ; 2.160291e+00 ; -1.579736e-01 ];
Tc_15  = [ -6.329340e-02 ; -9.285838e-02 ; 2.005767e-01 ];
omc_error_15 = [ 1.088595e-03 ; 1.153483e-03 ; 2.340240e-03 ];
Tc_error_15  = [ 2.484095e-04 ; 3.020178e-04 ; 3.071885e-04 ];

%-- Image #16:
omc_16 = [ -1.790035e+00 ; -1.898087e+00 ; -3.915109e-01 ];
Tc_16  = [ -6.370927e-02 ; -8.181912e-02 ; 1.697278e-01 ];
omc_error_16 = [ 1.198149e-03 ; 1.223863e-03 ; 1.970063e-03 ];
Tc_error_16  = [ 2.140813e-04 ; 2.691879e-04 ; 2.833566e-04 ];

%-- Image #17:
omc_17 = [ 2.083110e+00 ; 2.106657e+00 ; 1.806809e-01 ];
Tc_17  = [ -6.197612e-02 ; -1.004415e-01 ; 2.081914e-01 ];
omc_error_17 = [ 1.283604e-03 ; 1.244917e-03 ; 2.528957e-03 ];
Tc_error_17  = [ 2.653257e-04 ; 3.194941e-04 ; 3.402533e-04 ];

%-- Image #18:
omc_18 = [ -2.202194e+00 ; -2.168192e+00 ; 3.299427e-01 ];
Tc_18  = [ -6.122739e-02 ; -9.034146e-02 ; 2.175121e-01 ];
omc_error_18 = [ 1.221292e-03 ; 1.037965e-03 ; 2.449994e-03 ];
Tc_error_18  = [ 2.654874e-04 ; 3.238473e-04 ; 3.180436e-04 ];

%-- Image #19:
omc_19 = [ -1.853492e+00 ; -1.998543e+00 ; -2.930114e-01 ];
Tc_19  = [ -5.855059e-02 ; -1.103502e-01 ; 2.400452e-01 ];
omc_error_19 = [ 1.596838e-03 ; 1.473602e-03 ; 2.782425e-03 ];
Tc_error_19  = [ 3.024668e-04 ; 3.780486e-04 ; 4.105883e-04 ];

%-- Image #20:
omc_20 = [ 2.151620e+00 ; 2.124506e+00 ; -2.145154e-01 ];
Tc_20  = [ -7.999491e-02 ; -5.583297e-02 ; 3.183672e-01 ];
omc_error_20 = [ 2.037865e-03 ; 2.023906e-03 ; 4.141404e-03 ];
Tc_error_20  = [ 3.834164e-04 ; 4.758673e-04 ; 4.866343e-04 ];

