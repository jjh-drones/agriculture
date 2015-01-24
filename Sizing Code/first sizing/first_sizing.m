%% DESIGN REQUIREMENTS

%Constraints
npixels      = 16e6;
sensor_l     = 6.16e-3;
sensor_w     = 4.62e-3;
npixels_seed = 2;

%Assumptions
f          = 5e-3;
exposure_t = 3;
P_endlap   = 60;
P_sidelap  = 30;
seedband   = 6e-2;
window_t   = 8*3600;
field_S    = 250e4;

%% CALCULATIONS

% Altitude
sensor_A  = sensor_l*sensor_w;
pixel_A   = sensor_A/npixels;
pixel_l   = sqrt(pixel_A);
npixels_l = sensor_l/pixel_l;
npixels_w = sensor_w/pixel_l;
GSD       = seedband/npixels_seed;
S         = pixel_l/GSD;
H         = f/S;

% Velocity
coverage_l = npixels_l*GSD;                  %length covered by a photo
coverage_w = npixels_w*GSD;                  %width covered by a photo
exposure_l = coverage_l*(1-P_endlap/100);    %longitudinal space between camera exposure
exposure_w = coverage_w*(1-P_sidelap/100);   %lateral space between camera exposure
V          = exposure_l/exposure_t;

% Flight Time
field_l   = sqrt(field_S);
nstrips   = round(field_l/exposure_w);
footprint = 2*H/sin(pi/4) +  nstrips*field_l + (nstrips-1)*pi*0.5*exposure_w;
flight_t  = footprint/V;
