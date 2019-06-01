close all
h = 0.002;
t = numdiff_test_data.getElement('meas_phi').Values.Time;
phi_orig = numdiff_test_data.getElement('meas_phi').Values.Data;
phi_noise = phi_orig + randn(numel(t), 1)*0.001;
ddphi = numdiff_test_data.getElement('sim_ddphi').Values.Data;

cutoff = 1.5;
[b, a] = butter(5, 2*cutoff*h);
freqz(b, a)
phi_filter = filter(b, a, phi);

ddphi_num1 = numdiff(phi_filter, h, 2, 1);
ddphi_num2 = numdiff(phi_filter, h, 2, 2);

figure
subplot(311)
hold on
plot(t, phi_orig)
plot(t, phi_noise)
plot(t, phi_filter)

subplot(312)
hold on
plot(t, ddphi)
plot(t(2:end-1)- 0.4, ddphi_num1)
plot(t(3:end-2)- 0.4, ddphi_num2)

subplot(313)
hold on
plot(t(2:end-1), ddphi(2:end-1) - ddphi_num1)
plot(t(3:end-2), ddphi(3:end-2) - ddphi_num2)


function diff_data = numdiff(data, h, order, accuracy)
    if order == 1
        if accuracy == 1
            numerator = -1/2*data(1:end-2) + 1/2*data(3:end);
        elseif accuracy == 2
            numerator = 1/12*data(1:end-4) - 2/3*data(2:end-3) + 2/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h;
    elseif order == 2
        if accuracy == 1
            numerator = 1*data(1:end-2) - 2*data(2:end-1) + 1*data(3:end);
        elseif accuracy == 2
            numerator = -1/12*data(1:end-4) + 4/3*data(2:end-3) - 5/2*data(3:end-2) + 4/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h^2;
    end
end