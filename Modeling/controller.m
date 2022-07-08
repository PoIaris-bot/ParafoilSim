function u = controller(t)
delta_s = 0;
delta_a = 1.6;
u = [
    delta_s;  % delta_s
    delta_a * (t > 50);  % delta_a
];
end