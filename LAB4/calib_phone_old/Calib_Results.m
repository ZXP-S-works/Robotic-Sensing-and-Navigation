% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 560.799085011999182 ; 560.351213978360306 ];

%-- Principal point:
cc = [ 320.564911278903423 ; 239.746691219735339 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.082966686599920 ; -0.144961923235848 ; -0.000489728459570 ; 0.001555368075999 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.462534692306623 ; 0.501274647302129 ];

%-- Principal point uncertainty:
cc_error = [ 0.768174946526683 ; 0.566379639378533 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003850048626956 ; 0.011762086971560 ; 0.000410421412007 ; 0.000545653528540 ; 0.000000000000000 ];

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
omc_1 = [ -1.815467e+00 ; -2.164326e+00 ; -3.229591e-01 ];
Tc_1  = [ -7.026090e-02 ; -9.157787e-02 ; 3.096666e-01 ];
omc_error_1 = [ 1.438336e-03 ; 1.833058e-03 ; 3.118742e-03 ];
Tc_error_1  = [ 4.350917e-04 ; 3.278239e-04 ; 3.728608e-04 ];

%-- Image #2:
omc_2 = [ -2.470425e+00 ; -1.685810e+00 ; -5.949053e-01 ];
Tc_2  = [ -1.189451e-01 ; -3.463739e-02 ; 3.117573e-01 ];
omc_error_2 = [ 1.891854e-03 ; 1.404743e-03 ; 3.491439e-03 ];
Tc_error_2  = [ 4.447967e-04 ; 3.322664e-04 ; 4.253755e-04 ];

%-- Image #3:
omc_3 = [ 1.952205e+00 ; 1.943695e+00 ; -4.198901e-01 ];
Tc_3  = [ -8.091913e-02 ; -7.968320e-02 ; 3.141746e-01 ];
omc_error_3 = [ 1.044616e-03 ; 1.405091e-03 ; 2.261359e-03 ];
Tc_error_3  = [ 4.353712e-04 ; 3.205387e-04 ; 2.999992e-04 ];

%-- Image #4:
omc_4 = [ -2.048426e+00 ; -2.009942e+00 ; -9.727196e-01 ];
Tc_4  = [ -7.501471e-02 ; -4.849379e-02 ; 1.647934e-01 ];
omc_error_4 = [ 8.764625e-04 ; 1.213549e-03 ; 2.068338e-03 ];
Tc_error_4  = [ 2.402495e-04 ; 1.806659e-04 ; 2.218353e-04 ];

%-- Image #5:
omc_5 = [ -1.966131e+00 ; -1.972650e+00 ; -6.266989e-01 ];
Tc_5  = [ -9.073824e-02 ; -8.621976e-02 ; 3.599936e-01 ];
omc_error_5 = [ 1.662601e-03 ; 1.785341e-03 ; 3.234452e-03 ];
Tc_error_5  = [ 5.081238e-04 ; 3.852037e-04 ; 4.706208e-04 ];

%-- Image #6:
omc_6 = [ -1.761960e+00 ; -1.832522e+00 ; -3.741926e-01 ];
Tc_6  = [ -9.700910e-02 ; -6.360751e-02 ; 2.119876e-01 ];
omc_error_6 = [ 9.077601e-04 ; 1.207362e-03 ; 1.817023e-03 ];
Tc_error_6  = [ 2.971117e-04 ; 2.263071e-04 ; 2.412622e-04 ];

%-- Image #7:
omc_7 = [ -1.564859e+00 ; -1.682745e+00 ; -6.192702e-01 ];
Tc_7  = [ -9.583667e-02 ; -5.307427e-02 ; 1.910905e-01 ];
omc_error_7 = [ 8.423472e-04 ; 1.237713e-03 ; 1.575765e-03 ];
Tc_error_7  = [ 2.672206e-04 ; 2.054862e-04 ; 2.280963e-04 ];

%-- Image #8:
omc_8 = [ -1.852972e+00 ; -1.795296e+00 ; -1.088795e+00 ];
Tc_8  = [ -6.439685e-02 ; -2.723752e-02 ; 1.875957e-01 ];
omc_error_8 = [ 8.831495e-04 ; 1.309085e-03 ; 1.871898e-03 ];
Tc_error_8  = [ 2.636640e-04 ; 1.979659e-04 ; 2.372698e-04 ];

%-- Image #9:
omc_9 = [ 2.023188e+00 ; 1.697114e+00 ; -4.115656e-02 ];
Tc_9  = [ -8.081181e-02 ; -6.355684e-02 ; 1.948131e-01 ];
omc_error_9 = [ 9.745974e-04 ; 1.081799e-03 ; 1.807449e-03 ];
Tc_error_9  = [ 2.742673e-04 ; 2.001752e-04 ; 2.152786e-04 ];

%-- Image #10:
omc_10 = [ 2.039577e+00 ; 2.231170e+00 ; -7.194450e-01 ];
Tc_10  = [ -8.935661e-02 ; -7.315050e-02 ; 2.409794e-01 ];
omc_error_10 = [ 7.260535e-04 ; 1.254501e-03 ; 2.061892e-03 ];
Tc_error_10  = [ 3.356467e-04 ; 2.481278e-04 ; 2.185768e-04 ];

%-- Image #11:
omc_11 = [ 1.990945e+00 ; 1.975930e+00 ; -3.486376e-01 ];
Tc_11  = [ -8.612638e-02 ; -7.636879e-02 ; 2.228523e-01 ];
omc_error_11 = [ 8.490213e-04 ; 1.204479e-03 ; 1.957438e-03 ];
Tc_error_11  = [ 3.120102e-04 ; 2.287972e-04 ; 2.294595e-04 ];

%-- Image #12:
omc_12 = [ 2.366281e+00 ; 1.924190e+00 ; 4.840353e-01 ];
Tc_12  = [ -9.190476e-02 ; -6.444725e-02 ; 3.283491e-01 ];
omc_error_12 = [ 1.951916e-03 ; 1.654472e-03 ; 3.931144e-03 ];
Tc_error_12  = [ 4.685014e-04 ; 3.456886e-04 ; 4.793158e-04 ];

%-- Image #13:
omc_13 = [ -1.917464e+00 ; -1.925031e+00 ; -5.418909e-01 ];
Tc_13  = [ -8.322570e-02 ; -5.230066e-02 ; 1.815649e-01 ];
omc_error_13 = [ 8.627286e-04 ; 1.214292e-03 ; 1.920973e-03 ];
Tc_error_13  = [ 2.572942e-04 ; 1.953752e-04 ; 2.152909e-04 ];

%-- Image #14:
omc_14 = [ -2.041823e+00 ; -2.198350e+00 ; 3.050167e-02 ];
Tc_14  = [ -5.512234e-02 ; -9.316776e-02 ; 3.602628e-01 ];
omc_error_14 = [ 2.287323e-03 ; 2.575849e-03 ; 4.935369e-03 ];
Tc_error_14  = [ 5.014366e-04 ; 3.710518e-04 ; 4.534841e-04 ];

%-- Image #15:
omc_15 = [ -1.785878e+00 ; -1.972782e+00 ; -2.911331e-01 ];
Tc_15  = [ -6.947747e-02 ; -8.471661e-02 ; 3.267437e-01 ];
omc_error_15 = [ 1.281593e-03 ; 1.669644e-03 ; 2.656668e-03 ];
Tc_error_15  = [ 4.555094e-04 ; 3.411074e-04 ; 3.589312e-04 ];

%-- Image #16:
omc_16 = [ -2.228060e+00 ; -2.102760e+00 ; -4.918100e-01 ];
Tc_16  = [ -8.935025e-02 ; -6.505866e-02 ; 3.085858e-01 ];
omc_error_16 = [ 1.795700e-03 ; 1.726989e-03 ; 3.632639e-03 ];
Tc_error_16  = [ 4.384790e-04 ; 3.270659e-04 ; 4.240591e-04 ];

%-- Image #17:
omc_17 = [ 2.127401e+00 ; 1.639958e+00 ; 9.558547e-03 ];
Tc_17  = [ -9.308022e-02 ; -7.952546e-02 ; 3.407717e-01 ];
omc_error_17 = [ 1.351752e-03 ; 1.453261e-03 ; 2.538989e-03 ];
Tc_error_17  = [ 4.755170e-04 ; 3.503911e-04 ; 4.113011e-04 ];

%-- Image #18:
omc_18 = [ 2.028057e+00 ; 2.053792e+00 ; -1.009788e+00 ];
Tc_18  = [ -7.579099e-02 ; -5.491256e-02 ; 2.846282e-01 ];
omc_error_18 = [ 8.091747e-04 ; 1.243618e-03 ; 2.048046e-03 ];
Tc_error_18  = [ 3.933373e-04 ; 2.894008e-04 ; 2.109723e-04 ];

%-- Image #19:
omc_19 = [ -1.510495e+00 ; -2.047543e+00 ; -1.037075e-01 ];
Tc_19  = [ -7.016421e-02 ; -8.398899e-02 ; 2.518880e-01 ];
omc_error_19 = [ 8.949367e-04 ; 1.377695e-03 ; 1.850051e-03 ];
Tc_error_19  = [ 3.507364e-04 ; 2.609146e-04 ; 2.534388e-04 ];

%-- Image #20:
omc_20 = [ -2.027556e+00 ; -1.799410e+00 ; -1.294256e+00 ];
Tc_20  = [ -5.187757e-02 ; -2.456909e-02 ; 1.549611e-01 ];
omc_error_20 = [ 8.507835e-04 ; 1.285258e-03 ; 1.955090e-03 ];
Tc_error_20  = [ 2.205833e-04 ; 1.638723e-04 ; 2.117566e-04 ];

