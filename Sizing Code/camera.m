%% CAMERA DATA
npixels   = 12.1e6;
sensor_l  = 6.17e-3;
sensor_w  = 4.55e-3;

sensor_A  = sensor_l*sensor_w;
pixel_A   = sensor_A/npixels;
pixel_l   = sqrt(pixel_A);
npixels_l = sensor_l/pixel_l;
npixels_w = sensor_w/pixel_l;

%% COVERAGE
f          = 4.3e-3;
H          = 50;

S          = f/H;
GSD        = pixel_l/S;
coverage_l = npixels_l*GSD;
coverage_w = npixels_w*GSD;

%% OVERLAPPING
P_endlap   = 60;
P_sidelap  = 30;

exposure_l = coverage_l*(1-P_endlap/100);
exposure_w = coverage_w*(1-P_sidelap/100);

V          = 14;
t          = exposure_l/V;
