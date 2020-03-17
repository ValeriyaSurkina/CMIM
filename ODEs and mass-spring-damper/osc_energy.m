function [U, K] = osc_energy(u, v, omega)
    U = 0.5*omega.^2*u.^2;
    K = 0.5*v.^2;
end