% Roof Optimisation - Cantilever and Main Arch combined PIN ROLLER
% 3rd Year Design Project 3.0
% Name: Alicia Jiayun LAW

clear all
close all
clc

%% CONSTANTS
% Partial Load Factors
dead = 1.35;
imposed = 1.5;
% Typical Loads
unitW_clad = 2.5/1000^2; % kg/mm2 - mass per meter2 of cladding
UDL_imposed = (0.6 + 0.4)*1000/9.81/1000^2; %kg/mm2 (input as kN/m2) - Snow/Rain and Servicing

% Perry Robertson buckling check (curve a)
alpha = 0.21; % imperfection factor
gammaM1 = 1.0; % Partial safety factor

% Constants
g = 9.81; % m/s2 - gravitational acceleration
E = 2e5; % N/mm2 - Youngs Modulus of Steel

%% INITIAL ASSUMPTIONS
% (i) MEMBER CROSS SECTION + PROPERTIES FOR CANTILEVER TRUSS & BRACING
% USE: Member Bars - 168.3 x 5.0 CHS
fy_can = 275; % N/mm2 - Steel grade
A_can = 25.7*100; % mm2 - cross-sectional area of each member
I_can = 856*10^(4); % mm4 - second moment of area
EA_can = E*A_can; % N - axial rigidity of all members
EI_can = E*I_can; % Nmm2 - flexural rigidity of all members
unitW_can = 20.1/1000; % kg/mm - mass per meter of CHS

% (ii) MEMBER CROSS SECTION + PROPERTIES FOR MAIN ARCH
% USE: Members Bars - 406.4 x 16 CHS
fy_arch = 275; % N/mm - steel grade
A_arch = 196*100; % mm2 - cross-sectional area of each member
I_arch = 37400*10^(4); % mm4 - second moment of area
EA_arch = E*A_arch; % N - axial rigidity of all members
EI_arch = E*I_arch; % Nmm2 - flexural rigidity of all members
unitW_arch = 154/1000; % kg/mm2 - mass per meter of CHS

% (iii) TIE PROPERTIES
fy_tie = 275; % N/mm - steel grade
EA_tie = 3780*10^6; % N - axial rigidity of tie
A_tie = 22900; % mm2 - cross sectional area
unitW_tie = 184/1000 ; %kg/mm - mass per meter

% (iv) FIXED ROOF GEOMETRIES
span_arch = 105*1000; % mm - arch span
length_roof = 32*1000; % mm - roof width

%% INITIALISE OPTIMISATION
% Store the properties of configurations that pass all checks
pass = [];
longbrace = [];
FOSpass = [];
count = 0;

ri = 105000;
d = 8500;
restrainlong = [7 8 12];

m = round(span_arch/d);
if mod(m,2) ~= 0
    m = m+1;
end

% Geometry of Main Arch
% [] - Number of Arch Nodes
nodes_arch = 2*m + 2;
% Number of Elements
tie = 1;
elements_arch = 4*m + 1 +tie;

% inner chord
xi = -span_arch/2: span_arch/m : span_arch/2; % mm - x-coord of inner nodes
yi = sqrt(ri^2 - xi.^2); % mm - y-coord of inner nodes

% outer chord
xo = xi; % mm - x-coord of inner nodes
yo = yi + d; % mm - y-coord of inner nodes

% Specifying nodal properties - DOFs and Coordinates
NODESARCH.coords = zeros (nodes_arch,2);
NODESARCH.dofs = zeros (nodes_arch,3);

for a = 1 : nodes_arch
    % (i)   Specifying nodal x-y coordinates
    if a >= 1 && a <= m+1
        NODESARCH.coords (a,:) = [xo(a), yo(a)];
    else
        NODESARCH.coords (a,:) = [xi(a-m-1), yi(a-m-1)];
    end
    
    % (ii)  Specifying nodal dofs
    NODESARCH.dofs (a,:) = [3*a - 2 , 3*a - 1, 3*a];
    
end

% Specifying Element-Nodal connectivity
% Initialize
ELEMENTSARCH.nodes = zeros (elements_arch,2);
for a = 1: elements_arch
    
    % For elements on the upper chord
    if a >=1 && a <= m
        ELEMENTSARCH.nodes (a,:) = [a,a+1];
        ELEMENTSARCH.EAEI (a,:) = [EA_arch,EI_arch];
        
        % For elements on the lower chord
    elseif a >=m+1 && a <= 2*m
        ELEMENTSARCH.nodes (a,:) = [a+1,a+2];
        ELEMENTSARCH.EAEI (a,:) = [EA_arch,EI_arch];
        
        % For vertical elements
    elseif a >= 2*m+1 && a <= 3*m+1
        ELEMENTSARCH.nodes (a,:) = [a-2*m, a-m+1];
        ELEMENTSARCH.EAEI (a,:) = [EA_arch,0];
        
        % For left diagonal elements
    elseif a >= 3*m+2 && a <= 3*m + 1 + m/2
        ELEMENTSARCH.nodes (a,:) = [ELEMENTSARCH.nodes(a-3*m-1,1),ELEMENTSARCH.nodes(a-2*m-1,2)];
        ELEMENTSARCH.EAEI (a,:) = [EA_arch,0];
        
        % For right diagonal elements
    elseif a >= 3*m + 2 + m/2 && a <= elements_arch-1
        ELEMENTSARCH.nodes (a,:) = [ELEMENTSARCH.nodes(a-3*m-1,2),ELEMENTSARCH.nodes(a-2*m-1,1)];
        ELEMENTSARCH.EAEI (a,:) = [EA_arch,0];
        
        % For the tie
    else
        ELEMENTSARCH.nodes (a,:) = [m+2, nodes_arch];
        ELEMENTSARCH.EAEI (a,:) = [EA_tie, 0];
    end
end

% Boundary Conditions - Restraints of DOFs
dofsarch_free = [1:3*(m+1),3*(m+2):3*nodes_arch-2,3*nodes_arch];
dofsarch_restrained = [3*(m+2)-2,3*(m+2)-1,3*nodes_arch-1];

% Tributary Area
width_can = span_arch/m; % width of roofing supported by each cantilever section

% ----------------- C A N T I L E V E R ---------------------- %
span_can = (length_roof-d)/2; % mm - cantilever span
theta = atan((d-1000)/span_can); % rad - inclination of cantilever
angle = 45/180*pi; % angle of diagonal bars

xnext = 0;
x_can = [0];
stop = 0;
h = d; % First x-coord

% Obtain x-coord of config
while stop == 0
    bar = h*(sin(pi/2 - theta))/(sin(pi/2 - angle +theta));
    xnext = xnext + bar*cos(pi/2-angle) ;
    x_can = [x_can xnext];
    
    if xnext <=span_can
        stop = 0;
        h = d - xnext*tan(theta);
    else
        stop = 1; % stop
    end
end

n = length(x_can)-1;

% Obtain y-coord
yu_can = d; % mm - y-coord of upper cantilever nodes
yl_can = tan(theta)*x_can; % mm - y-coord of lower cantilever nodes

% Obtain length of lower chord members
xlength = diff(x_can);
ylength = diff(yl_can);

% mm - length of lower chord cantilever member
for a = 1:n
    llowchord_can (a) = sqrt(xlength(a)^2 + ylength(a)^2);
end

% [] - Number of Cantilever Nodes
nodes_can = 2*n + 2;

% [] - Number of Cantilever Elements
elements_can = 4*n;

% Specifying cantilever's nodal properties - DOFs and Coordinates
NODESCAN.coords = zeros (nodes_can,2);
NODESCAN.dofs = zeros (nodes_can,3);
for a = 1 : nodes_can
    
    % (i)   Specifying cantilever's nodal x-y coordinates
    if a >= 1 && a <= n+1
        NODESCAN.coords (a,:) = [x_can(a), yu_can];
    else
        NODESCAN.coords (a,:) = [x_can(a-n-1), yl_can(a-n-1)];
    end
    
    % (ii)  Specifying cantilever's nodal dofs
    NODESCAN.dofs (a,:) = [3*a - 2, 3*a - 1 , 3*a];
    
end

% Specifying Cantilever's element-nodal connectivity
% Initialize
ELEMENTSCAN.nodes = zeros (elements_can,2);
ELEMENTSCAN.EAEI = zeros (elements_can,2);

for a = 1: elements_can
    
    % For elements on the upper chord
    if a >=1 && a <= n
        ELEMENTSCAN.nodes (a,:) = [a,a+1];
        ELEMENTSCAN.EAEI (a,:) = [EA_can,EI_can];
        
        % For elements on the lower chord
    elseif a >=n+1 && a <= 2*n
        ELEMENTSCAN.nodes (a,:) = [a+1,a+2];
        ELEMENTSCAN.EAEI (a,:) = [EA_can,EI_can];
        
        % For vertical elements
    elseif a >= 2*n+1 && a <= 3*n
        ELEMENTSCAN.nodes (a,:) = [a-2*n+1, a-n+2];
        ELEMENTSCAN.EAEI (a,:) = [EA_can,0];
        
        % For diagonal elements
        % Take the first node of the upper element and second node of the
        % lower element
    else
        ELEMENTSCAN.nodes (a,:) = [ELEMENTSCAN.nodes(a-3*n,1),ELEMENTSCAN.nodes(a-2*n,2)];
        ELEMENTSCAN.EAEI (a,:) = [EA_can,0];
    end
end


% Cantilever Boundary Conditions - Restraints of DOFs
dofscan_free = [4:3*(n+1), 3*(n+3)-2:3*(nodes_can)];
dofscan_restrained = [1,2,3,3*(n+2)-2,3*(n+2)-1,3*(n+2)];

% Establish matrix of additional vertical loads on nodes, excluding SW and lower chord longtitudinal bracing
% Upper chord longitudinal bracing load contribution per node
load_upperlong = width_can * unitW_can * g;

% N - Total applied point load per upper node
load_uppercan = width_can * span_can * g * (dead*unitW_clad + imposed*UDL_imposed)/(n+1) + load_upperlong;

% Additional Point Loads on top of cantilever SW - Input in a matrix
load_additionalcan = zeros(3*nodes_can,1);
for i = 1:n+1
    load_additionalcan (3*i-1) = load_uppercan;
end

%% Loop 3: Loop through lower chord longitudinal bracing configuration
% Using combination:
% Initialise for FOS of configs that pass
minFOS_canyield = 1000; % Arbitrary minimum FOS
minFOS_canbuck = 1000; % Arbitrary minimum FOS
fail = 0;

% Lower chord longitudinal bracing load contribution per node
load_longtruss = width_can * unitW_can * g;
load_additionalcan (3*(restrainlong(1))-1) = load_longtruss; % first node definitely restrained
load_additionalcan (3*(restrainlong(end))-1) = (width_can + sqrt(width_can^2 + d^2)) * unitW_can * g; % last node definitely restrained
load_additionalcan (3*(restrainlong(2:end-1))-1) = load_longtruss;

% ----------------- RESTRAINT CONFIG BEGINS ---------------------- %
% Solve Cantilever
amp = 1;
obj1 = BEAM('Cantilever', NODESCAN, ELEMENTSCAN, dofscan_restrained, dofscan_free);
obj1 = obj1.assemble (unitW_can,unitW_can, load_additionalcan,dead,dofscan_restrained);
obj1 = obj1.solve ();
obj1 = obj1.newcoord (amp);
obj1 = obj1.FindMaxVertDisp ();
obj1 = obj1.axial ();
obj1 = obj1.length ();
obj1 = obj1.plotting(amp);

memberfail_canbuck = [];
memberfail_canyield = [];
% ---------------- CANTILEVER CHECKS ---------------- %
% ---------------CHECK #1a: Deflections -------------- %
if obj1.maxvertdisp > span_can/180 % failed deflection
    
    fail = 1;
    
else
    fail = 0; % continue other checks
end


% -------- CHECK #2a+#3a: Yielding + InBuckling --------- %
for p = 1:elements_can
    
    % Effective Length for in plane buckling
    if ELEMENTSCAN.EAEI (p,2) == 0 % pinned members
        leff = obj1.Length(p);
    else
        leff = 0.75*obj1.Length(p); % continuous members
    end
    
    Ncr = EI_can*pi^2/leff^2 ;
    lambda = sqrt(A_can*fy_can / Ncr); % Slenderness
    phi = 0.5*(1 + alpha*(lambda - 0.2) + lambda^2);
    chi = 1 / (phi + sqrt(phi^2 - lambda^2));
    NbRd = chi*A_can*fy_can/gammaM1; % Buckling resistance
    
    FOS_canyield = A_can*fy_can/abs(obj1.Faxial(p));
    
    if obj1.Faxial(p) < 0 % Member in compression
        FOS_caninbuck = NbRd/-obj1.Faxial(p); % FOS for buckling
    else
        FOS_caninbuck = 1000000; % High FOS for tension member
    end
    
    if FOS_caninbuck < minFOS_canbuck
        minFOS_canbuck = FOS_caninbuck;
    end
    
    if FOS_canyield < minFOS_canyield
        minFOS_canyield = FOS_canyield;
    end
    
    % Sieve #2: In Plane Buckling using Perry-Robertson a curve
    if FOS_caninbuck < 1
        
        fail = 1;
        memberfail_canbuck = [memberfail_canbuck, p];
        
        % Sieve #3: Yielding
    elseif FOS_canyield < 1
        
        fail = 1;
        memberfail_canyield = [memberfail_canyield, p];
        
    else
        
        fail = 0; % Pass --> Continue
        
    end
       
end


% ----------- CHECK #4a: Out of Plane Buckling --------- %
distance = diff(restrainlong); % number of members between each restrain
first = n+1; %first element within the restrained nodes

for q = 1:length(distance) % Checking between longitudinal restrains
    
    last = first + distance(q) - 1; %last element
    
    leff = 0.75 * sum(llowchord_can ((first-n):(last-n))); % effective length
    Ncr = EI_can*pi^2/leff^2 ; % vector
    lambda = sqrt(A_can*fy_can / Ncr); % Slenderness
    phi = 0.5*(1 + alpha*(lambda - 0.2) + lambda^2);
    chi = 1 / (phi + sqrt(phi^2 - lambda^2));
    NbRd = chi*A_can*fy_can/gammaM1; % Buckling resistance
    
    for r = first:last
        
        if obj1.Faxial(r) < 0
            FOS_outbuck = NbRd/-obj1.Faxial(r);
        else
            FOS_outbuck = 10000;
        end
        
        if FOS_outbuck < minFOS_canbuck
            minFOS_canbuck = FOS_outbuck;
        end
        
        if FOS_outbuck < 1 % Compression member has buckled - failed
            fail = 1;
            break %  failed --> out of restraint loop
            
        else
            fail = 0; % pass --> continue checking other members in the restrain
            
        end
    end
    
    if fail == 1
         memberfail_canbuck = [memberfail_canbuck, r];
         
    else
        first = last+1; % When all members within restrain pass, check next restrain
    end
end

obj1 = obj1.buckleplot(memberfail_canbuck);
obj1 = obj1.yieldplot(memberfail_canyield);

% ------------------- A R C H C H E C K ------------------- %
minFOS_archyield = 10000;
minFOS_archbuck = 10000;
% Transfer cantilever reactions as arch inputs to begin
reaction_topvert = obj1.F (2); % N - vertical reaction by cantilever on top nodes
reaction_botvert = obj1.F (3*(n+2)-1); % N - vertical reaction by cantilever on bottom nodes

load_additionalarch = zeros(3*nodes_arch,1);
for i = 1: m+1
    load_additionalarch (3*i-1) = reaction_topvert + dead*(d+sqrt(2)*d + sqrt((span_arch/m)^2+d^2))/2*unitW_arch*g + span_arch/m*d*g * (dead*unitW_clad + imposed*UDL_imposed);
end

for i = m+2: nodes_arch
    load_additionalarch (3*i-1) = reaction_botvert + dead*(d+sqrt(2)*d + sqrt((span_arch/m)^2+d^2))/2*unitW_arch*g;
end

% Solve the configuration
amp = 1;
obj2 = BEAM('main arch', NODESARCH, ELEMENTSARCH, dofsarch_restrained, dofsarch_free);
obj2 = obj2.assemble (unitW_arch,unitW_tie, load_additionalarch,dead,dofsarch_restrained);
obj2 = obj2.solve ();
obj2 = obj2.newcoord (amp);
obj2 = obj2.FindMaxVertDisp ();
obj2 = obj2.FindMaxLatDisp ();
obj2 = obj2.axial ();
obj2 = obj2.length ();
obj2 = obj2.plotting(amp);

memberfail_archbuck = [];
memberfail_archyield = [];
% ----------- CHECK #1b: Deflections --------- %
if obj2.maxvertdisp > span_arch/180 % failed deflection
    
    fail = 1;
    
else
    fail = 0; % continue other checks
    
end

% ----------- CHECK #2b: Lat Disp --------- %
if obj2.maxlatdisp > 1000 % too much lateral displacement
    
    fail = 1;
    
else
    fail = 0; % continue other checks
    
end

% ----------- CHECK #3b: Tie members --------- %
FOS_tie = A_tie*fy_tie/obj2.Faxial(elements_arch);

if FOS_tie < 1
    
    fail = 1;
    
else
    fail = 0; % pass --> continue
end

% -------- CHECK #4b+#5b: Yielding + InBuckling --------- %
for p = 1:elements_arch-1
    
    % Effective Length for in plane buckling
    if ELEMENTSARCH.EAEI (p,2) == 0 % pinned members
        leff = obj2.Length(p);
    else
        leff = 0.75*obj2.Length(p); % continuous members
    end
    
    Ncr = EI_arch*pi^2/leff^2 ;
    lambda = sqrt(A_arch*fy_arch / Ncr); % Slenderness
    phi = 0.5*(1 + alpha*(lambda - 0.2) + lambda^2);
    chi = 1 / (phi + sqrt(phi^2 - lambda^2));
    NbRd = chi*A_arch*fy_arch/gammaM1; % Buckling resistance
    
    FOS_archyield = A_arch*fy_arch/abs(-obj2.Faxial(p));
    if obj2.Faxial(p) < 0 % Compression Buckling
        FOS_archinbuck = NbRd/-obj2.Faxial(p);
    else
        FOS_archinbuck = 10000; %Tension dont buckle
    end
    
    if FOS_archinbuck < minFOS_archbuck
        minFOS_archbuck = FOS_archinbuck;
    end
    
    if FOS_archyield < minFOS_archyield
        minFOS_archyield = FOS_archyield;
    end
    
    % Sieve: In Plane Buckling using Perry-Robertson a curve
    if FOS_archinbuck < 1
        
        fail = 1;
        memberfail_archbuck = [memberfail_archbuck, p];
        
        % Check #3: Yielding
    elseif FOS_archyield < 1
        
        fail = 1;
        memberfail_archyield = [memberfail_archyield, p];
        
    else
        fail = 0;
        
    end
end

obj2 = obj2.buckleplot(memberfail_archbuck);
obj2 = obj2.yieldplot(memberfail_archyield);

mass = (d+sqrt(2)*d + sqrt((span_arch/m)^2+d^2)) * unitW_arch *(m+1) + ...
       sum(obj1.Length) * unitW_can* (m+1) + ...
       sum(obj2.Length(1:end-1)) * unitW_arch + ...
       load_longtruss*(n+length(restrainlong))/g + ...
       unitW_tie*obj2.Length(end);
tonage = mass*1000000/(span_arch*length_roof/2);
vertreaction = obj2.F(3*(m+2)-1);
forceintie = obj2.Faxial(end); 