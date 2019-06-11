function [x_hat,P_hat] = BARO_update_const_a(p,x_tilde,P_tilde,R_baro,g)

    H = zeros(1, size(x_tilde, 1));
    H(1) = -x_tilde(5) * g * x_tilde(6) * exp(-x_tilde(5)*g*(x_tilde(1)-x_tilde(4)));
    H(4) = x_tilde(5) * g * x_tilde(6) * exp(-x_tilde(5)*g*(x_tilde(1)-x_tilde(4)));
    H(5) = -g * (x_tilde(1) - x_tilde(4)) * x_tilde(6) * exp(-x_tilde(5)*g*(x_tilde(1)-x_tilde(4)));
    H(6) = exp(-x_tilde(5) * g * (x_tilde(1)-x_tilde(4)));
    
    h_x = x_tilde(6) * exp(-x_tilde(5)*g*(x_tilde(1)-x_tilde(4)));
    
    K = P_tilde * H' * (H * P_tilde * H' + R_baro)^-1;
    x_hat = x_tilde + K * (p - h_x);
    P_hat = (eye(size(x_tilde, 1)) - K * H) * P_tilde;

end