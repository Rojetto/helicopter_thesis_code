data_pos_low = [
    2, 10;
    2.25, 11;
    2.5, 12.5;
    2.75, 15;
    3, 19;
    3.25, 23;
    3.5, 28;
    3.75, 33;
    4, 39;
    4.25, 45;
    4.5, 50;
    4.75, 56;
    5, 62;
    5.25, 69;
    5.5, 77;
    5.75, 84;
    6, 92;
    6.25, 100
];

data_pos_low(:,2) = data_pos_low(:,2)*0.01; % g to N

data_pos_high = [
    4, 45;
    4.25, 50;
    4.5, 57;
    4.75, 63;
    5, 68;
    5.25, 73;
    5.5, 80;
    5.75, 90;
    6, 97;
    6.25, 105;
    6.5, 112;
    6.75, 120;
    7, 125;
    7.25, 135;
    7.5, 145;
    7.75, 150;
    8, 160;
    8.25, 167;
    8.5, 175;
    8.75, 180;
    9, 190;
    9.25, 200;
    9.5, 202;
    9.75, 205;
    10, 210
];

data_pos_high(:,2) = data_pos_high(:,2)*0.01;

data_neg = [
    2, 10;
    2.25, 11;
    2.5, 12;
    2.75, 12;
    3, 13;
    3.25, 15;
    3.5, 17;
    3.75, 20;
    4, 22;
    4.25, 25;
    4.5, 28;
    4.75, 30;
    5, 33;
    5.25, 36;
    5.5, 39;
    5.75, 42;
    6, 45;
    6.25, 48;
    6.5, 51;
    6.75, 54;
    7, 56;
    7.25, 59;
    7.5, 62;
    7.75, 65;
    8, 66;
    8.25, 70;
    8.5, 72;
    8.75, 75;
    9, 77;
    9.25, 80;
    9.5, 84;
    9.75, 87;
    10, 89    
];

l_h = 0.66;
l_c = 0.49;

data_neg(:, 1) = -data_neg(:, 1);
data_neg(:, 2) = -l_c/l_h*0.01*data_neg(:, 2);

figure(1)
title('Original Data')
hold on
plot(data_pos_low(:,1), data_pos_low(:,2), 'bx')
plot(data_pos_high(:,1), data_pos_high(:,2), 'rx')
plot(data_neg(:,1), data_neg(:,2), 'gx')
xlim([-10, 10])
ylim([-1, 2.5])
grid on


overlap_i_low = 9;
overlap_i_high = 10;

overlap_high = data_pos_high(1:overlap_i_high,:);
overlap_low = data_pos_low(overlap_i_low:end,:);
pos_data_offset = mean(overlap_high(:,2) - overlap_low(:,2));

overlap_data = ([overlap_high(:, 1), overlap_high(:,2) - pos_data_offset] + overlap_low)/2;

data_pos = [data_pos_low(1:overlap_i_low-1, :);
            overlap_data;
            data_pos_high(overlap_i_high+1:end,1), data_pos_high(overlap_i_high+1:end,2)-pos_data_offset];

param_guess = [1, 1, 0.1];
opts = optimoptions('lsqnonlin', 'Jacobian', 'on', 'TolFun', 1e-12);
param1 = lsqnonlin(@(p) compute_error_vector(p, data_pos(1:end-3,1), data_pos(1:end-3,2)), param_guess, [], [], opts);
param2 = lsqnonlin(@(p) compute_error_vector(p, -data_neg(1:end,1), -data_neg(1:end,2)), param_guess, [], [], opts);

p1_opt = param1(1)
q1_opt = param1(2)
d1_opt = param1(3)

p2_opt = param2(1)
q2_opt = param2(2)
d2_opt = param2(3)

Vs_plot = 0:0.1:10;

figure(2)
title('Fitted Data')
hold on
plot(data_pos(:,1), data_pos(:,2)-d1_opt, 'bx')
plot(data_neg(:,1), data_neg(:,2)+d2_opt, 'bx')
plot(Vs_plot, Fs(Vs_plot, p1_opt, q1_opt), 'r')
plot(-Vs_plot, -Fs(Vs_plot, p2_opt, q2_opt), 'r')
line([2*q1_opt/p1_opt, 2*q1_opt/p1_opt], [0, 10])
line([-2*q2_opt/p2_opt, -2*q2_opt/p2_opt], [-10, 0])
xlim([-10, 10])
ylim([-1, 2.5])
xlabel('Vs [V]')
ylabel('Fs [g]')
grid on