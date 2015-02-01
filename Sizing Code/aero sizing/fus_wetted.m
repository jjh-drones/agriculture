function Swetted = fus_wetted(D,l_nose,l_body,l_rear)

R        = 0.5*D;

%% Nose
% Conical
Swetted_nose =  pi*R*sqrt(l_nose^2+R^2);

%% Body
% Square
Swetted_body = 4*D*l_body;

%% Rear
% Triangular pyramid
phi          = atan(D/l_rear);
l_bottom     = sqrt(l_rear^2+D^2);
Swetted_rear = 2*D*l_rear/2 + D*l_rear + D*l_bottom;

%% Overall
Swetted = Swetted_nose + Swetted_body + Swetted_rear;

end

