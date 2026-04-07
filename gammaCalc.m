function Gamma =gammaCalc(aircraft_parameters)
Ix=aircraft_parameters.Ix;
Iy=aircraft_parameters.Iy;
Iz=aircraft_parameters.Iz;
Ixz=aircraft_parameters.Ixz;

g=Ix*Ix-Ixz^2;

Gamma.one=Ixz*(Ix-Iy+Iz)./g;
Gamma.two=(Iz*(Iz-Iy)+Ixz^2)./g;
Gamma.three=Iz./g;
Gamma.four=Ixz/g;
Gamma.five=(Iz-Ix)./Iy;
Gamma.six=Ixz./Iy;
Gamma.seven=(Ix*(Ix-Iy)+Ixz^2)./g;
Gamma.eight=Ix./g;
end