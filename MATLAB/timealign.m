function [t,a1,a2] = timealign(t1,t2,u1,u2,res)
%TIMEALIGN   Align 2 data matrices in time.
%   The presumption is that t1 and t2 vectors are associated
%   with data matrices that must be time aligned.  There is
%   no assumption of even time spacing, but times are assumed
%   to be monotonically increasing.
%
%   Matrices u1 and u2 must be column-based, i.e.,
%     length(t1) == size(u1,1) and length(t2) == size(u2,1)
%
%   [t,a1,a2] = timealign(t1,t2,u1,u2) returns
%     t    time, the union of t1 & t2
%     a1   u1 at points where t == t1 (NaN otherwise)
%     a2   u2 at points where t == t2 (NaN otherwise)
%
%   [...] = timealign(t1,t2,u1,u2,res) aligns using a fixed 
%     resolution res
%
%
%   EXAMPLE:
%     t1 = [1 2 3]';  u1 = [6 7 8]';
%     t2 = [2 4 5]';  u2 = [3 4; -1 2; 9 12];
%     [t,a1,a2] = timealign(t1,t2,u1,u2)
%      t =         a1 =         a2 =    
%          1              6          NaN   NaN
%          2              7            3     4
%          3              8          NaN   NaN
%          4            NaN           -1     2
%          5            NaN            9    12
%
%
%   Developed under Matlab version 7.10.0.499 (R2010a)
%   Created by Qi An
%   anqi2000@gmail.com
%   QA 2/22/2013 initial skeleton

% must be column oriented
if length(t1) ~= size(u1,1) || length(t2) ~= size(u2,1),
  fprintf('%s: u1 or u2 not column oriented\n',mfilename)
  [t,a1,a2] = deal( [] );
  return;
end

t = min([min(t1) min(t2)]):res:max([max(t1) max(t2)]);
a1 = zeros(size(t, 2), 1);
a2 = zeros(size(t, 2), 1);

for step = 1:size(t, 2)
    temp_t1 = abs(t1 - t(step));
    idx_t1 = temp_t1 < res / 2;
    if sum(idx_t1) > 0
        a1(step) = sum(u1(idx_t1)) / sum(idx_t1);
    else
        a1(step) = nan;
    end
    
    temp_t2 = abs(t2 - t(step));
    idx_t2 = temp_t2 < res / 2;
    if sum(idx_t2) > 0
        a2(step) = sum(u2(idx_t2)) / sum(idx_t2);
    else
        a2(step) = nan;
    end
end