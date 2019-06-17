function [out] = control_fcn(in)
controller = evalin('base', 'controller');

t = in(1);
x1 = in(2);
x2 = in(3);

[u, p12, p22] = controller.control(t, [x1; x2]);

out = zeros(1, 3);
out(1) = u;
out(2) = p12;
out(3) = p22;
end

