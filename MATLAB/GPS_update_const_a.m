function [x_hat,P_hat] = GPS_update_const_a(z,x_tilde,P_tilde,R_gps)
    
    H = zeros(1, size(x_tilde, 1));
    H(1, 1) = 1;
    
    K = P_tilde * H' * (H * P_tilde * H' + R_gps)^-1;
    x_hat = x_tilde + K * (z - H * x_tilde);
    P_hat = (eye(size(x_tilde, 1)) - K * H) * P_tilde;

end

