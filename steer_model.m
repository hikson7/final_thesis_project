function [varphi_d_out] = steer_model(t, phi)

varphi_d = theta_d.*sin(phi);

end