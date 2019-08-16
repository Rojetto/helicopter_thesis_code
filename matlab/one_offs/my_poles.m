function the_poles = my_poles( coeffs)
if coeffs(end) == 1
    the_poles = roots(coeffs(end:-1:1));
else
    the_poles = roots([1, coeffs(end:-1:1)]);
end
end

