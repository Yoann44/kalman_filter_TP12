function [time_last,x_tilde,P_tilde] = prediction(time_now,time_last,x_hat,P_hat,Q,F,G)

    dt = time_now - time_last;
    [Q_w,phi] = compute_Q_w(Q, F, G, dt);
    
    x_tilde = phi * x_hat;
    P_tilde = phi * P_hat * phi' + Q_w;
    
    time_last = time_now;

end

