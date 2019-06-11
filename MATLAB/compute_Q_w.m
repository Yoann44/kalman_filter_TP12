function [Q_w,PHI] = compute_Q_w(Q,F,G,dt)
A = [...
    -F,G*Q*G';...
    zeros(size(F)),F']*dt;

B = expm(A);
PHI = B((end/2)+1:end,(end/2)+1:end)';
Q_w = PHI*B(1:end/2,(end/2)+1:end);
end
